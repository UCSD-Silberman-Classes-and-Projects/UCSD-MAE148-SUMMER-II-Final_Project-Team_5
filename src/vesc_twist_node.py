import rclpy
import traceback
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from .vesc_submodule.vesc_client import VESC_

NODE_NAME = 'vesc_twist_node'
TOPIC_NAME = '/cmd_vel'


class VescTwist(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.vesc = VESC_()
        self.rpm_subscriber = self.create_subscription(Twist, TOPIC_NAME, self.callback, 10)
        self.battery_voltage_publisher = self.create_publisher(
            Float32,
            '/battery_voltage',
            10
        )

        self.battery_timer = self.create_timer(
            1.0,
            self.publish_battery_voltage
        )



        # Default actuator values
        self.default_rpm_value = int(20000) 
        self.default_steering_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.default_throttle_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.default_max_right_steering = 0.8
        self.default_straight_steering = 0.5
        self.default_max_left_steering = 0.1
        self.default_zero_throttle = -0.032
        self.default_max_throttle = 0.382
        self.default_min_throttle = 0.322
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_rpm', self.default_rpm_value),
                ('steering_polarity', self.default_steering_polarity),
                ('throttle_polarity', self.default_throttle_polarity),
                ('max_right_steering', self.default_max_right_steering),
                ('straight_steering', self.default_straight_steering),
                ('max_left_steering', self.default_max_left_steering),
                ('zero_throttle', self.default_zero_throttle),
                ('max_throttle', self.default_max_throttle),
                ('min_throttle', self.default_min_throttle)
            ])
        self.max_rpm = int(self.get_parameter('max_rpm').value)
        self.steering_polarity = int(self.get_parameter('steering_polarity').value)
        self.throttle_polarity = int(self.get_parameter('throttle_polarity').value)
        self.max_right_steering = self.get_parameter('max_right_steering').value
        self.straight_steering = self.get_parameter('straight_steering').value
        self.max_left_steering = self.get_parameter('max_left_steering').value
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value
        
        # Remappings 
        self.max_right_steering_angle = self.remap(self.max_right_steering)
        self.steering_offset = self.remap(self.straight_steering) - self.default_straight_steering
        self.max_left_steering_angle = self.remap(self.max_left_steering)
        self.zero_rpm = int(self.zero_throttle * self.max_rpm)
        self.max_rpm = int(self.max_throttle  * self.max_rpm)
        self.min_rpm = int(self.min_throttle  * self.max_rpm)

        self.get_logger().info(
            f'\nmax_rpm: {self.max_rpm}'
            f'\nsteering_polarity: {self.steering_polarity}'
            f'\nthrottle_polarity: {self.throttle_polarity}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nstraight_steering: {self.straight_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
            f'\nsteering_offset: {self.steering_offset}'
            )


    def callback(self, msg):
        # # Steering map from [-1,1] --> [0,1]
        steering_angle = float(self.steering_offset + self.remap(msg.angular.z))
        
        # RPM map from [-1,1] --> [-max_rpm,max_rpm]
        rpm = int(self.max_rpm * msg.linear.x)

        # self.get_logger().info(f'rpm: {rpm}, steering_angle: {steering_angle}')
        self.vesc.send_rpm(int(self.throttle_polarity * rpm))
        self.vesc.send_servo_angle(float(self.steering_polarity * steering_angle))

    def publish_battery_voltage(self):
        try:
            voltage = self.vesc.get_input_voltage()

            if voltage is None:
                self.get_logger().warn(
                    'No battery measurement received.'
                )
                return

            msg = Float32()
            msg.data = voltage
            self.battery_voltage_publisher.publish(msg)

        except Exception as error:
            self.get_logger().warn(
                f'Battery voltage read failed: {error}'
            )


    def remap(self, value):
        input_start = -1
        input_end = 1
        output_start = 0
        output_end = 1
        normalized_output = float(output_start + (value - input_start) * ((output_end - output_start) / (input_end - input_start)))
        return normalized_output
    
    def clamp(self, data, upper_bound, lower_bound=None):
            if lower_bound==None:
                lower_bound = -upper_bound # making lower bound symmetric about zero
            if data < lower_bound:
                data_c = lower_bound
            elif data > upper_bound:
                data_c = upper_bound
            else:
                data_c = data
            return data_c 


def main(args=None):
    rclpy.init(args=args)
    try:
        vesc_twist = VescTwist()
        rclpy.spin(vesc_twist)
        vesc_twist.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print("VESC NODE REAL ERROR:", repr(e))
        traceback.print_exc()
        vesc_twist.get_logger().info(f'Shutting down {NODE_NAME}...')
        vesc_twist.destroy_node()
        rclpy.shutdown()
        vesc_twist.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
