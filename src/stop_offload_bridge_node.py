import json
import urllib.request
import urllib.error

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class StopOffloadBridgeNode(Node):

    def __init__(self):
        super().__init__('stop_offload_bridge_node')

        self.declare_parameter(
            'server_url',
            'http://192.168.139.143:8000/infer'
        )
        self.declare_parameter('inference_hz', 2.0)
        self.declare_parameter('jpeg_quality', 70)

        self.server_url = (
            self.get_parameter('server_url')
            .get_parameter_value()
            .string_value
        )

        self.inference_hz = (
            self.get_parameter('inference_hz')
            .get_parameter_value()
            .double_value
        )

        self.jpeg_quality = (
            self.get_parameter('jpeg_quality')
            .get_parameter_value()
            .integer_value
        )

        self.bridge = CvBridge()
        self.latest_image = None

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_0',
            self.image_callback,
            image_qos,
        )

        self.stop_publisher = self.create_publisher(
            Bool,
            '/stop_sign_detected',
            10,
        )

        period = 1.0 / max(self.inference_hz, 0.1)
        self.timer = self.create_timer(period, self.run_inference)

        self.get_logger().info(
            f'STOP-only offload bridge ready. '
            f'Server={self.server_url}, rate={self.inference_hz:.1f} Hz'
        )

        self.get_logger().info(
            'This node publishes ONLY /stop_sign_detected.'
        )

    def image_callback(self, msg):
        self.latest_image = msg

    def run_inference(self):
        if self.latest_image is None:
            return

        msg = self.latest_image
        self.latest_image = None

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            success, encoded = cv2.imencode(
                '.jpg',
                frame,
                [
                    int(cv2.IMWRITE_JPEG_QUALITY),
                    self.jpeg_quality,
                ],
            )

            if not success:
                self.get_logger().warning('JPEG encoding failed.')
                return

            request = urllib.request.Request(
                self.server_url,
                data=encoded.tobytes(),
                method='POST',
                headers={'Content-Type': 'image/jpeg'},
            )

            with urllib.request.urlopen(
                request,
                timeout=1.5
            ) as response:
                payload = json.loads(
                    response.read().decode('utf-8')
                )

            detected = bool(
                payload.get('stop_detected', False)
            )

            ros_msg = Bool()
            ros_msg.data = detected
            self.stop_publisher.publish(ros_msg)

            confidence = payload.get('confidence', 0.0)
            ratio = payload.get('box_height_ratio', 0.0)
            inference_ms = payload.get('inference_ms', 0.0)
            seen = payload.get('stop_seen', False)

            self.get_logger().info(
                f'STOP seen={seen} '
                f'triggered={detected} '
                f'conf={confidence} '
                f'height={ratio} '
                f'inference={inference_ms}ms'
            )

        except Exception as error:
            self.get_logger().warning(
                f'STOP server request failed: {error}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = StopOffloadBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
