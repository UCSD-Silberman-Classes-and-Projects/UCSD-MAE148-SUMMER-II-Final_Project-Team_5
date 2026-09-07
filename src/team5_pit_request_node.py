import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32


class Team5PitRequestNode(Node):

    def __init__(self):
        super().__init__('team5_pit_request_node')

        self.declare_parameter(
            'battery_threshold_percent',
            50.0
        )

        self.battery_threshold = float(
            self.get_parameter(
                'battery_threshold_percent'
            ).value
        )

        self.stop_seen = False
        self.battery_percentage = None

        self.last_stop_time = None
        self.last_battery_time = None

        # If either signal becomes stale, pit request becomes False.
        self.stop_timeout = 1.0
        self.battery_timeout = 3.0

        self.stop_subscriber = self.create_subscription(
            Bool,
            '/team5_stop_seen',
            self.stop_callback,
            10
        )

        self.battery_subscriber = self.create_subscription(
            Float32,
            '/battery_percentage',
            self.battery_callback,
            10
        )

        self.pit_request_publisher = self.create_publisher(
            Bool,
            '/team5_pit_request',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_pit_request
        )

        self.last_request = None

        self.get_logger().info(
            'Team 5 pit request gate started'
        )

        self.get_logger().info(
            'PIT = /team5_stop_seen AND '
            f'/battery_percentage < {self.battery_threshold:.1f}%'
        )


    def stop_callback(self, msg):
        self.stop_seen = bool(msg.data)
        self.last_stop_time = time.monotonic()


    def battery_callback(self, msg):
        self.battery_percentage = float(msg.data)
        self.last_battery_time = time.monotonic()


    def publish_pit_request(self):

        now = time.monotonic()

        stop_is_fresh = (
            self.last_stop_time is not None
            and
            now - self.last_stop_time
            <= self.stop_timeout
        )

        battery_is_fresh = (
            self.last_battery_time is not None
            and
            now - self.last_battery_time
            <= self.battery_timeout
        )

        battery_low = (
            battery_is_fresh
            and
            self.battery_percentage is not None
            and
            self.battery_percentage
            < self.battery_threshold
        )

        pit_request = (
            stop_is_fresh
            and
            self.stop_seen
            and
            battery_low
        )

        msg = Bool()
        msg.data = bool(pit_request)

        self.pit_request_publisher.publish(msg)

        if pit_request != self.last_request:

            battery_text = (
                'UNKNOWN'
                if self.battery_percentage is None
                else
                f'{self.battery_percentage:.1f}%'
            )

            self.get_logger().info(
                f'STOP={self.stop_seen} | '
                f'Battery={battery_text} | '
                f'PIT REQUEST={pit_request}'
            )

            self.last_request = pit_request


def main(args=None):

    rclpy.init(args=args)

    node = Team5PitRequestNode()

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
