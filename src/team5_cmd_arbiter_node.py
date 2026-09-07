import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Team5CmdArbiterNode(Node):

    def __init__(self):
        super().__init__('team5_cmd_arbiter_node')

        self.declare_parameter(
            'lane_timeout',
            0.30
        )

        self.declare_parameter(
            'pit_timeout',
            0.30
        )

        self.declare_parameter(
            'pit_active_timeout',
            0.50
        )

        self.lane_timeout = float(
            self.get_parameter(
                'lane_timeout'
            ).value
        )

        self.pit_timeout = float(
            self.get_parameter(
                'pit_timeout'
            ).value
        )

        self.pit_active_timeout = float(
            self.get_parameter(
                'pit_active_timeout'
            ).value
        )

        self.latest_lane_cmd = Twist()
        self.latest_pit_cmd = Twist()

        self.last_lane_time = None
        self.last_pit_time = None

        self.pit_active = False
        self.last_pit_active_time = None

        self.previous_mode = None

        self.lane_sub = self.create_subscription(
            Twist,
            '/team5_lane_cmd_vel',
            self.lane_callback,
            10
        )

        self.pit_sub = self.create_subscription(
            Twist,
            '/team5_pit_cmd_vel',
            self.pit_callback,
            10
        )

        self.pit_active_sub = self.create_subscription(
            Bool,
            '/team5_pit_active',
            self.pit_active_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Team 5 command arbiter started'
        )

        self.get_logger().info(
            'NORMAL: /team5_lane_cmd_vel -> /cmd_vel'
        )

        self.get_logger().info(
            'PIT: /team5_pit_cmd_vel -> /cmd_vel'
        )


    def lane_callback(self, msg):

        self.latest_lane_cmd = msg
        self.last_lane_time = time.monotonic()


    def pit_callback(self, msg):

        self.latest_pit_cmd = msg
        self.last_pit_time = time.monotonic()


    def pit_active_callback(self, msg):

        self.pit_active = bool(msg.data)
        self.last_pit_active_time = time.monotonic()


    def fresh(self, timestamp, timeout):

        return (
            timestamp is not None
            and
            time.monotonic() - timestamp
            <= timeout
        )


    def publish_zero(self):

        self.cmd_pub.publish(
            Twist()
        )


    def control_loop(self):

        pit_status_fresh = self.fresh(
            self.last_pit_active_time,
            self.pit_active_timeout
        )

        #
        # PIT MODE
        #
        if self.pit_active:

            # If the pit node disappears while a pit is active,
            # STOP rather than accidentally returning control
            # to the lane follower.
            if not pit_status_fresh:

                mode = 'PIT_FAILSAFE'

                self.publish_zero()

            elif self.fresh(
                self.last_pit_time,
                self.pit_timeout
            ):

                mode = 'PIT'

                self.cmd_pub.publish(
                    self.latest_pit_cmd
                )

            else:

                mode = 'PIT_CMD_STALE'

                self.publish_zero()

        #
        # NORMAL LANE MODE
        #
        else:

            if self.fresh(
                self.last_lane_time,
                self.lane_timeout
            ):

                mode = 'LANE'

                self.cmd_pub.publish(
                    self.latest_lane_cmd
                )

            else:

                mode = 'LANE_CMD_STALE'

                self.publish_zero()

        if mode != self.previous_mode:

            self.get_logger().info(
                f'CONTROL MODE = {mode}'
            )

            self.previous_mode = mode


def main(args=None):

    rclpy.init(args=args)

    node = Team5CmdArbiterNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Explicit stop when arbiter shuts down.
        node.cmd_pub.publish(
            Twist()
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
