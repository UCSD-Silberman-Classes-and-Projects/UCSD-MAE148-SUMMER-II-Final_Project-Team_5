from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32

from .battery_calibration import (
    LOW_ENTER_VOLTAGE,
    LOW_RECOVERY_VOLTAGE,
    update_low_battery_state,
    voltage_to_percentage,
)


# Compatibility aliases retained for existing launch/tests and operator notes.
LOW_THRESHOLD = LOW_ENTER_VOLTAGE
RECOVERY_THRESHOLD = LOW_RECOVERY_VOLTAGE
AVERAGE_SAMPLES = 5


class BatteryMonitorNode(Node):

    def __init__(self):
        super().__init__('battery_monitor_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('averaging_window', AVERAGE_SAMPLES),
                ('low_voltage', LOW_ENTER_VOLTAGE),
                ('recovery_voltage', LOW_RECOVERY_VOLTAGE),
            ],
        )
        self.averaging_window = int(
            self.get_parameter('averaging_window').value
        )
        self.low_voltage = float(self.get_parameter('low_voltage').value)
        self.recovery_voltage = float(
            self.get_parameter('recovery_voltage').value
        )
        if self.averaging_window < 1:
            raise ValueError('averaging_window must be at least one')
        if self.recovery_voltage <= self.low_voltage:
            raise ValueError('recovery_voltage must exceed low_voltage')

        self.battery_low_publisher = self.create_publisher(
            Bool,
            '/battery_low',
            10
        )

        self.battery_percentage_publisher = self.create_publisher(
            Float32,
            '/battery_percentage',
            10
        )

        self.battery_voltage_subscriber = self.create_subscription(
            Float32,
            '/battery_voltage',
            self.battery_callback,
            10
        )

        self.voltage_history = deque(maxlen=self.averaging_window)
        self.battery_low = False

        self.get_logger().info(
            'Battery monitor listening to /battery_voltage; '
            f'window={self.averaging_window}, '
            f'low={self.low_voltage:.2f} V, '
            f'recovery={self.recovery_voltage:.2f} V'
        )

    def battery_callback(self, msg):
        voltage = float(msg.data)

        self.voltage_history.append(voltage)

        if len(self.voltage_history) < self.averaging_window:
            self.get_logger().info(
                f'Battery voltage: {voltage:.2f} V '
                f'({len(self.voltage_history)}/'
                f'{self.averaging_window} samples)'
            )
            return

        average_voltage = (
            sum(self.voltage_history)
            / len(self.voltage_history)
        )

        battery_percentage = voltage_to_percentage(
            average_voltage
        )

        self.battery_low = update_low_battery_state(
            average_voltage,
            self.battery_low,
            self.low_voltage,
            self.recovery_voltage,
        )

        low_msg = Bool()
        low_msg.data = self.battery_low
        self.battery_low_publisher.publish(low_msg)

        percentage_msg = Float32()
        percentage_msg.data = float(battery_percentage)
        self.battery_percentage_publisher.publish(
            percentage_msg
        )

        self.get_logger().info(
            f'Raw: {voltage:.2f} V | '
            f'Average: {average_voltage:.2f} V | '
            f'Battery: {battery_percentage:.1f}% | '
            f'Battery Low: {self.battery_low}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = BatteryMonitorNode()

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
