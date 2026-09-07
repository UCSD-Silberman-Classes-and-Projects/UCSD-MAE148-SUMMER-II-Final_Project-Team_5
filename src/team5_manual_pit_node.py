import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import SetParametersResult


IDLE = 'IDLE'
TURN_IN = 'TURN_IN'
STRAIGHT_IN = 'STRAIGHT_IN'
ALIGN_IN = 'ALIGN_IN'
DWELL = 'DWELL'
TURN_OUT = 'TURN_OUT'
STRAIGHT_OUT = 'STRAIGHT_OUT'
ALIGN_OUT = 'ALIGN_OUT'


class Team5ManualPitNode(Node):

    def __init__(self):
        super().__init__('team5_manual_pit_node')

        # We KNOW these command magnitudes work on Team 5.
        # We do NOT yet know the exact physical timing for
        # 45 degrees or one foot, so timing starts at zero.
        self.declare_parameter('pit_throttle', 0.40)
        self.declare_parameter('pit_steering', 0.35)

        self.declare_parameter('turn_seconds', 0.80)
        self.declare_parameter('straight_seconds', 0.70)

        self.declare_parameter('dwell_seconds', 60.0)

        self.pit_throttle = float(
            self.get_parameter('pit_throttle').value
        )

        self.pit_steering = float(
            self.get_parameter('pit_steering').value
        )

        self.turn_seconds = float(
            self.get_parameter('turn_seconds').value
        )

        self.straight_seconds = float(
            self.get_parameter('straight_seconds').value
        )

        self.dwell_seconds = float(
            self.get_parameter('dwell_seconds').value
        )

        # Allow calibration parameters to be changed while running.
        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        self.request_sub = self.create_subscription(
            Bool,
            '/team5_pit_request',
            self.request_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/team5_pit_cmd_vel',
            10
        )

        self.active_pub = self.create_publisher(
            Bool,
            '/team5_pit_active',
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/team5_pit_state',
            10
        )

        self.state = IDLE
        self.state_start = time.monotonic()

        self.trigger_armed = True

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Team 5 manual pit node started'
        )

        self.get_logger().info(
            'Sequence: RIGHT -> STRAIGHT -> LEFT -> '
            'WAIT 60s -> LEFT -> STRAIGHT -> RIGHT'
        )

        if not self.is_calibrated():
            self.get_logger().warning(
                'Pit timing NOT calibrated yet. '
                'turn_seconds or straight_seconds are invalid. '
                'Pit movement is locked out.'
            )


    def parameter_callback(self, params):

        new_turn = self.turn_seconds
        new_straight = self.straight_seconds
        new_dwell = self.dwell_seconds
        new_throttle = self.pit_throttle
        new_steering = self.pit_steering

        for param in params:

            if param.name == 'turn_seconds':
                new_turn = float(param.value)

            elif param.name == 'straight_seconds':
                new_straight = float(param.value)

            elif param.name == 'dwell_seconds':
                new_dwell = float(param.value)

            elif param.name == 'pit_throttle':
                new_throttle = float(param.value)

            elif param.name == 'pit_steering':
                new_steering = float(param.value)

        if new_turn <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='turn_seconds must be greater than 0'
            )

        if new_straight <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='straight_seconds must be greater than 0'
            )

        if new_dwell < 0.0:
            return SetParametersResult(
                successful=False,
                reason='dwell_seconds cannot be negative'
            )

        self.turn_seconds = new_turn
        self.straight_seconds = new_straight
        self.dwell_seconds = new_dwell
        self.pit_throttle = new_throttle
        self.pit_steering = new_steering

        self.get_logger().info(
            'PIT CALIBRATION UPDATED: '
            f'turn={self.turn_seconds:.2f}s | '
            f'straight={self.straight_seconds:.2f}s | '
            f'dwell={self.dwell_seconds:.1f}s | '
            f'throttle={self.pit_throttle:.2f} | '
            f'steering={self.pit_steering:.2f}'
        )

        return SetParametersResult(
            successful=True
        )


    def is_calibrated(self):
        return (
            self.turn_seconds > 0.0
            and
            self.straight_seconds > 0.0
        )


    def request_callback(self, msg):

        request = bool(msg.data)

        # Must go FALSE before another pit cycle can start.
        if not request:
            self.trigger_armed = True
            return

        if (
            self.state == IDLE
            and
            self.trigger_armed
        ):

            if not self.is_calibrated():
                self.get_logger().error(
                    'PIT REQUEST received, but maneuver '
                    'timing has not been calibrated.'
                )
                self.trigger_armed = False
                return

            self.trigger_armed = False

            self.transition(
                TURN_IN
            )


    def transition(self, new_state):

        self.get_logger().info(
            f'PIT {self.state} -> {new_state}'
        )

        self.state = new_state
        self.state_start = time.monotonic()


    def elapsed(self):
        return (
            time.monotonic()
            -
            self.state_start
        )


    def make_command(
        self,
        throttle,
        steering
    ):

        msg = Twist()

        msg.linear.x = float(throttle)
        msg.angular.z = float(steering)

        return msg


    def control_loop(self):

        elapsed = self.elapsed()

        #
        # STATE TRANSITIONS
        #

        if (
            self.state == TURN_IN
            and
            elapsed >= self.turn_seconds
        ):
            self.transition(
                STRAIGHT_IN
            )

        elif (
            self.state == STRAIGHT_IN
            and
            elapsed >= self.straight_seconds
        ):
            self.transition(
                ALIGN_IN
            )

        elif (
            self.state == ALIGN_IN
            and
            elapsed >= self.turn_seconds
        ):
            self.transition(
                DWELL
            )

        elif (
            self.state == DWELL
            and
            elapsed >= self.dwell_seconds
        ):
            self.transition(
                TURN_OUT
            )

        elif (
            self.state == TURN_OUT
            and
            elapsed >= self.turn_seconds
        ):
            self.transition(
                STRAIGHT_OUT
            )

        elif (
            self.state == STRAIGHT_OUT
            and
            elapsed >= self.straight_seconds
        ):
            self.transition(
                ALIGN_OUT
            )

        elif (
            self.state == ALIGN_OUT
            and
            elapsed >= self.turn_seconds
        ):
            self.transition(
                IDLE
            )

            self.get_logger().info(
                'PIT COMPLETE'
            )

        #
        # COMMAND FOR CURRENT STATE
        #

        if self.state == TURN_IN:

            # Bank RIGHT into pit.
            cmd = self.make_command(
                self.pit_throttle,
                +self.pit_steering
            )

        elif self.state == STRAIGHT_IN:

            # Travel roughly one foot after calibration.
            cmd = self.make_command(
                self.pit_throttle,
                0.0
            )

        elif self.state == ALIGN_IN:

            # Counter-steer LEFT so car becomes straight.
            cmd = self.make_command(
                self.pit_throttle,
                -self.pit_steering
            )

        elif self.state == DWELL:

            # Stay stopped for one minute.
            cmd = Twist()

        elif self.state == TURN_OUT:

            # Start mirrored exit: LEFT.
            cmd = self.make_command(
                self.pit_throttle,
                -self.pit_steering
            )

        elif self.state == STRAIGHT_OUT:

            cmd = self.make_command(
                self.pit_throttle,
                0.0
            )

        elif self.state == ALIGN_OUT:

            # Counter-steer RIGHT back to original heading.
            cmd = self.make_command(
                self.pit_throttle,
                +self.pit_steering
            )

        else:

            cmd = Twist()

        self.cmd_pub.publish(
            cmd
        )

        active_msg = Bool()
        active_msg.data = (
            self.state != IDLE
        )

        self.active_pub.publish(
            active_msg
        )

        state_msg = String()
        state_msg.data = self.state

        self.state_pub.publish(
            state_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = Team5ManualPitNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Explicit zero command on shutdown.
        node.cmd_pub.publish(
            Twist()
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
