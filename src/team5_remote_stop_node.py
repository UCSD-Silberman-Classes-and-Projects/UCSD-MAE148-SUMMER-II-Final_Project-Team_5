import time

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .mac_vision_bridge_node import MacVisionClient
from .mac_vision_protocol import (
    ConfirmationFilter,
    resize_for_transport,
)


class Team5RemoteStopNode(Node):

    def __init__(self):
        super().__init__('team5_remote_stop_node')

        self.declare_parameter(
            'host',
            '192.168.139.143'
        )

        self.declare_parameter(
            'port',
            18420
        )

        self.declare_parameter(
            'confidence_threshold',
            0.35
        )

        self.declare_parameter(
            'min_box_height_ratio',
            0.16
        )

        self.declare_parameter(
            'confirm_detections',
            2
        )

        self.host = str(
            self.get_parameter('host').value
        )

        self.port = int(
            self.get_parameter('port').value
        )

        self.confidence_threshold = float(
            self.get_parameter(
                'confidence_threshold'
            ).value
        )

        self.min_box_height_ratio = float(
            self.get_parameter(
                'min_box_height_ratio'
            ).value
        )

        required_hits = int(
            self.get_parameter(
                'confirm_detections'
            ).value
        )

        self.bridge = CvBridge()

        self.client = MacVisionClient(
            self.host,
            self.port,
            0.5
        )

        self.confirmation = ConfirmationFilter(
            required_hits
        )

        self.latest_image = None

        self.stop_publisher = self.create_publisher(
            Bool,
            '/team5_stop_seen',
            10
        )

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.camera_subscription = (
            self.create_subscription(
                Image,
                '/camera/color/image_0',
                self.camera_callback,
                camera_qos
            )
        )

        # Send at 5 Hz.
        self.timer = self.create_timer(
            0.2,
            self.run_detection
        )

        self.request_id = 0
        self.last_log_time = 0.0

        self.get_logger().info(
            'Team 5 remote YOLO STOP node started'
        )

        self.get_logger().info(
            f'PC server = {self.host}:{self.port}'
        )

        self.get_logger().info(
            'Publishing STOP result on '
            '/team5_stop_seen'
        )


    def camera_callback(self, msg):
        self.latest_image = msg


    def publish_stop(self, detected):

        msg = Bool()
        msg.data = bool(detected)

        self.stop_publisher.publish(msg)


    def run_detection(self):

        if self.latest_image is None:
            return

        image_msg = self.latest_image

        # Always use newest frame only.
        self.latest_image = None

        try:

            frame = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8'
            )

            frame = resize_for_transport(
                frame,
                640
            )

            success, jpeg = cv2.imencode(
                '.jpg',
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    80
                ]
            )

            if not success:
                raise RuntimeError(
                    'JPEG encoding failed'
                )

            self.request_id += 1

            result = self.client.analyze(
                jpeg.tobytes(),
                self.request_id,
                'NORMAL',
                self.confidence_threshold,
                self.min_box_height_ratio,
            )

            confirmed = (
                self.confirmation.update(
                    result['stop']
                )
            )

            self.publish_stop(
                confirmed
            )

            now = time.monotonic()

            if (
                now - self.last_log_time
                >= 1.0
            ):

                self.get_logger().info(
                    f'STOP raw={result["stop"]} | '
                    f'confidence='
                    f'{result["confidence"]:.3f} | '
                    f'confirmed={confirmed}'
                )

                self.last_log_time = now

        except Exception as error:

            self.confirmation.clear()

            self.publish_stop(
                False
            )

            now = time.monotonic()

            if (
                now - self.last_log_time
                >= 2.0
            ):

                self.get_logger().warning(
                    f'PC YOLO unavailable: {error}'
                )

                self.last_log_time = now


def main(args=None):

    rclpy.init(args=args)

    node = Team5RemoteStopNode()

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
