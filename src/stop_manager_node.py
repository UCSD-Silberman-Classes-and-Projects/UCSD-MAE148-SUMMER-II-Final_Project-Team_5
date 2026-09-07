"""Single final velocity arbiter and Team 5 pit-state ROS wrapper."""

import copy
import time

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

from .command_ramp import AccelerationRamp
from .pit_state_machine import (
    PIT_DWELL,
    PitStateMachine,
    PRE_PIT_STOP,
    WAIT_FOR_BATTERY,
)


DEFAULT_STOP_DURATION = 4.0
DEFAULT_PIT_DRIVE_DURATION = 10.0
DEFAULT_PIT_DWELL_DURATION = 5.0
SIGN_CLEAR_TIME = 1.0
LANE_COMMAND_TIMEOUT = 0.5
OUTPUT_HZ = 20.0
ACCELERATION_SLEW_RATE = 2.0
BATTERY_STATUS_TIMEOUT = 2.5


class StopManagerNode(Node):
    """Arbitrate lane commands through the deterministic pit cycle."""

    def __init__(self):
        super().__init__('stop_manager_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('stop_duration', DEFAULT_STOP_DURATION),
                ('pit_drive_duration', DEFAULT_PIT_DRIVE_DURATION),
                ('pit_dwell_duration', DEFAULT_PIT_DWELL_DURATION),
                ('sign_clear_time', SIGN_CLEAR_TIME),
                ('acquisition_confirmation_frames', 3),
                ('lane_status_timeout', 0.5),
                ('transition_drive_timeout', 5.0),
                ('lane_command_timeout', LANE_COMMAND_TIMEOUT),
                ('battery_status_timeout', BATTERY_STATUS_TIMEOUT),
                ('acceleration_slew_rate', ACCELERATION_SLEW_RATE),
                ('output_hz', OUTPUT_HZ),
                ('pit_lane_enabled', False),
            ],
        )

        def parameter(name):
            return self.get_parameter(name).value

        self.machine = PitStateMachine(
            pre_pit_stop_duration=parameter('stop_duration'),
            pit_drive_duration=parameter('pit_drive_duration'),
            pit_dwell_duration=parameter('pit_dwell_duration'),
            sign_clear_time=parameter('sign_clear_time'),
            acquisition_confirmation_frames=parameter(
                'acquisition_confirmation_frames'
            ),
            lane_status_timeout=parameter('lane_status_timeout'),
            transition_drive_timeout=parameter(
                'transition_drive_timeout'
            ),
            battery_status_timeout=parameter('battery_status_timeout'),
            pit_lane_enabled=parameter('pit_lane_enabled'),
        )
        self.stop_duration = self.machine.pre_pit_stop_duration
        self.lane_command_timeout = float(parameter('lane_command_timeout'))
        self.output_hz = float(parameter('output_hz'))
        self.acceleration_ramp = AccelerationRamp(
            parameter('acceleration_slew_rate')
        )
        self.acceleration_ramp.reset(time.monotonic())

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/pit_state', 10)
        self.lane_mode_pub = self.create_publisher(
            String,
            '/lane_mode',
            10,
        )
        self.lane_sub = self.create_subscription(
            Twist,
            '/lane_cmd_vel',
            self.lane_callback,
            10,
        )
        self.stop_sub = self.create_subscription(
            Bool,
            '/stop_sign_detected',
            self.stop_callback,
            10,
        )
        self.battery_sub = self.create_subscription(
            Bool,
            '/battery_low',
            self.battery_callback,
            10,
        )
        self.lane_status_sub = self.create_subscription(
            String,
            '/lane_status',
            self.lane_status_callback,
            10,
        )

        self.latest_lane_cmd = Twist()
        self.last_lane_cmd_time = None
        self.active_mode_ready = False
        self.transition_zero_pending = False
        self.last_logged_state = self.machine.state
        self.transition_hold_warned = False
        self.timer = self.create_timer(1.0 / self.output_hz, self.control_loop)

        self.get_logger().info(
            'Pit manager ready: one /cmd_vel arbiter; '
            f'pre-stop={self.stop_duration:.1f}s, '
            f'pit-drive={self.machine.pit_drive_duration:.1f}s, '
            f'dwell={self.machine.pit_dwell_duration:.1f}s; '
            f'acceleration={parameter("acceleration_slew_rate"):.2f}/s; '
            f'pit lane enabled={self.machine.pit_lane_enabled}'
        )

    @property
    def state(self):
        """Expose state for diagnostics and node-level tests."""
        return self.machine.state

    @property
    def stop_started_time(self):
        """Expose the latched pre-pit stop start time."""
        return self.machine.state_entered_at

    @property
    def stop_end_time(self):
        """Expose the latched pre-pit stop deadline."""
        return self.stop_started_time + self.stop_duration

    def lane_callback(self, msg):
        """Cache the latest high-level lane command."""
        self.latest_lane_cmd = copy.deepcopy(msg)
        self.last_lane_cmd_time = time.monotonic()

    def battery_callback(self, msg):
        """Update the battery-conditioned pit trigger."""
        previous_state = self.machine.state
        previous_mode = self.machine.lane_mode
        self.machine.update_battery(msg.data, time.monotonic())
        self._prepare_callback_transition(previous_state, previous_mode)
        self._log_transition()

    def stop_callback(self, msg):
        """Update STOP/PIT-sign detection without cancelling a latch."""
        previous_state = self.machine.state
        previous_mode = self.machine.lane_mode
        self.machine.update_stop_detection(msg.data, time.monotonic())
        self._prepare_callback_transition(previous_state, previous_mode)
        self._log_transition()

    def lane_status_callback(self, msg):
        """Consume an atomic MODE:ACQUIRED or MODE:LOST detector status."""
        try:
            mode, status = msg.data.upper().split(':', 1)
        except ValueError:
            self.get_logger().warn(
                f'Ignoring malformed lane status: {msg.data!r}'
            )
            return
        if status not in ('ACQUIRED', 'LOST'):
            return
        accepted = self.machine.observe_lane_status(
            mode,
            status == 'ACQUIRED',
            time.monotonic(),
        )
        if accepted and not self.active_mode_ready:
            self.last_lane_cmd_time = None
            self.active_mode_ready = True

    def control_loop(self):
        """Advance the FSM and publish exactly one final velocity command."""
        now = time.monotonic()
        previous_state = self.machine.state
        previous_mode = self.machine.lane_mode
        self.machine.advance(now)
        state_changed = previous_state != self.machine.state
        lane_mode_changed = previous_mode != self.machine.lane_mode
        if lane_mode_changed:
            self.last_lane_cmd_time = None
            self.active_mode_ready = False
        if state_changed or lane_mode_changed:
            self.acceleration_ramp.reset(now)
            self.transition_zero_pending = True
        self._publish_status()
        self._log_transition()

        just_left_stop = (
            previous_state in (
                WAIT_FOR_BATTERY,
                PRE_PIT_STOP,
                PIT_DWELL,
            )
            and previous_state != self.machine.state
        )
        motion_allowed = self.machine.motion_allowed(now)
        lane_command_is_fresh = (
            self.last_lane_cmd_time is not None
            and now - self.last_lane_cmd_time <= self.lane_command_timeout
        )

        must_stop = (
            just_left_stop
            or lane_mode_changed
            or self.transition_zero_pending
            or not motion_allowed
            or not lane_command_is_fresh
            or not self.active_mode_ready
            or self.latest_lane_cmd.linear.x == 0.0
        )
        if must_stop:
            self.acceleration_ramp.reset(now)
            output_command = Twist()
        else:
            output_command = copy.deepcopy(self.latest_lane_cmd)
            output_command.linear.x = self.acceleration_ramp.apply(
                output_command.linear.x,
                now,
            )
        self.cmd_pub.publish(output_command)
        self.transition_zero_pending = False

        safety_hold = (
            self.machine.state
            not in (WAIT_FOR_BATTERY, PRE_PIT_STOP, PIT_DWELL)
            and not motion_allowed
        )
        if safety_hold and not self.transition_hold_warned:
            self.get_logger().warn(
                f'{self.machine.state} lane acquisition lost/timed out; '
                'holding zero'
            )
            self.transition_hold_warned = True
        elif motion_allowed:
            self.transition_hold_warned = False

    def _publish_status(self):
        state_msg = String()
        state_msg.data = self.machine.state
        self.state_pub.publish(state_msg)

        mode_msg = String()
        mode_msg.data = self.machine.lane_mode
        self.lane_mode_pub.publish(mode_msg)

    def _prepare_callback_transition(self, previous_state, previous_mode):
        """Latch one exact-zero output for callback-driven transitions."""
        state_changed = previous_state != self.machine.state
        mode_changed = previous_mode != self.machine.lane_mode
        if not (state_changed or mode_changed):
            return
        now = time.monotonic()
        self.acceleration_ramp.reset(now)
        self.transition_zero_pending = True
        if mode_changed:
            self.last_lane_cmd_time = None
            self.active_mode_ready = False

    def _log_transition(self):
        if self.machine.state == self.last_logged_state:
            return
        self.get_logger().info(
            f'Pit state: {self.last_logged_state} -> {self.machine.state}; '
            f'lane mode={self.machine.lane_mode}'
        )
        self.last_logged_state = self.machine.state
        self.transition_hold_warned = False


def main(args=None):
    """Run the pit-state manager node."""
    rclpy.init(args=args)
    node = StopManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
