"""ROS bridge sending newest camera frames to one Mac vision service."""

import http.client
import json
import threading
import time

import cv2

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import Image

from std_msgs.msg import Bool, Float32, String

from .mac_vision_protocol import (
    ConfirmationFilter,
    normalize_lane_mode,
    PROTOCOL_VERSION,
    resize_for_transport,
    validate_response,
)


MAX_RESPONSE_BYTES = 32 * 1024
MAX_REQUEST_HZ = 30.0


class MacVisionClient:
    """Dependency-free HTTP client for one combined vision request."""

    def __init__(self, host, port, timeout):
        """Store and validate the remote endpoint configuration."""
        self.host = str(host)
        self.port = int(port)
        self.timeout = float(timeout)
        if not self.host:
            raise ValueError('host must not be empty')
        if not 1 <= self.port <= 65535:
            raise ValueError('port must be between 1 and 65535')
        if self.timeout <= 0.0:
            raise ValueError('request timeout must be positive')

    def analyze(
        self,
        jpeg_bytes,
        request_id,
        lane_mode,
        confidence_threshold,
        min_box_height_ratio,
    ):
        """Send one JPEG and validate the versioned combined response."""
        connection = http.client.HTTPConnection(
            self.host,
            self.port,
            timeout=self.timeout,
        )
        try:
            connection.request(
                'POST',
                '/v1/detect-vision',
                body=jpeg_bytes,
                headers={
                    'Content-Type': 'image/jpeg',
                    'X-Protocol-Version': str(PROTOCOL_VERSION),
                    'X-Request-Id': str(request_id),
                    'X-Lane-Mode': lane_mode,
                    'Connection': 'close',
                },
            )
            response = connection.getresponse()
            body = response.read(MAX_RESPONSE_BYTES + 1)
            if len(body) > MAX_RESPONSE_BYTES:
                raise ValueError('host response is too large')
            if response.status != 200:
                detail = body.decode('utf-8', errors='replace')
                raise RuntimeError(
                    f'host returned HTTP {response.status}: {detail}'
                )
            payload = json.loads(body.decode('utf-8'))
            return validate_response(
                payload,
                request_id,
                lane_mode,
                confidence_threshold,
                min_box_height_ratio,
            )
        finally:
            connection.close()


class MacVisionBridgeNode(Node):
    """Send one newest-frame-only combined request at a bounded rate."""

    def __init__(self):
        """Create ROS interfaces and start the bounded worker thread."""
        super().__init__('mac_vision_bridge_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', '192.168.139.155'),
                ('port', 18420),
                ('inference_hz', 5.0),
                ('confidence_threshold', 0.35),
                ('min_box_height_ratio', 0.16),
                ('confirm_detections', 2),
                ('request_timeout', 0.5),
                ('camera_timeout', 0.75),
                ('jpeg_quality', 80),
                ('max_image_dimension', 640),
            ],
        )

        def parameter(name):
            return self.get_parameter(name).value

        self.inference_hz = float(parameter('inference_hz'))
        self.confidence_threshold = float(
            parameter('confidence_threshold')
        )
        self.min_box_height_ratio = float(
            parameter('min_box_height_ratio')
        )
        self.camera_timeout = float(parameter('camera_timeout'))
        self.jpeg_quality = int(parameter('jpeg_quality'))
        self.max_image_dimension = int(parameter('max_image_dimension'))
        required_hits = int(parameter('confirm_detections'))
        self._validate_parameters(required_hits)

        self.client = MacVisionClient(
            parameter('host'),
            parameter('port'),
            parameter('request_timeout'),
        )
        self.bridge = CvBridge()
        self.confirmation = ConfirmationFilter(required_hits)
        self.stop_publisher = self.create_publisher(
            Bool,
            '/stop_sign_detected',
            10,
        )
        self.centroid_publisher = self.create_publisher(
            Float32,
            '/centroid',
            10,
        )
        self.lane_acquired_publisher = self.create_publisher(
            Bool,
            '/lane_acquired',
            10,
        )
        self.lane_status_publisher = self.create_publisher(
            String,
            '/lane_status',
            10,
        )

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_0',
            self.image_callback,
            camera_qos,
        )
        self.mode_subscription = self.create_subscription(
            String,
            '/lane_mode',
            self.lane_mode_callback,
            10,
        )

        self._frame_condition = threading.Condition()
        self._latest_frame = None
        self._last_camera_time = None
        self._lane_mode = 'NORMAL'
        self._request_id = 0
        self._stop_event = threading.Event()
        self._last_log_time = 0.0
        self._last_error_log_time = 0.0
        self._worker = threading.Thread(
            target=self._worker_loop,
            name='mac-vision-bridge-worker',
            daemon=True,
        )
        self._worker.start()

        self.get_logger().info(
            'Mac vision bridge ready: '
            f'{self.client.host}:{self.client.port}; '
            f'{self.inference_hz:.1f} Hz; '
            f'STOP threshold={self.confidence_threshold:.2f}; '
            f'min box ratio={self.min_box_height_ratio:.2f}; '
            f'confirmations={required_hits}'
        )

    def _validate_parameters(self, required_hits):
        if not 0.0 < self.inference_hz <= MAX_REQUEST_HZ:
            raise ValueError('inference_hz must be in (0, 30]')
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be in [0, 1]')
        if not 0.0 <= self.min_box_height_ratio <= 1.0:
            raise ValueError('min_box_height_ratio must be in [0, 1]')
        if self.camera_timeout <= 0.0:
            raise ValueError('camera_timeout must be positive')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')
        if self.max_image_dimension < 1:
            raise ValueError('max_image_dimension must be positive')
        if required_hits < 1:
            raise ValueError('confirm_detections must be positive')

    def image_callback(self, msg):
        """Replace the one pending frame without conversion or blocking."""
        received_at = time.monotonic()
        with self._frame_condition:
            self._latest_frame = (msg, received_at)
            self._last_camera_time = received_at
            self._frame_condition.notify()

    def lane_mode_callback(self, msg):
        """Track the requested mode and invalidate in-flight old-mode data."""
        requested_mode = normalize_lane_mode(msg.data)
        with self._frame_condition:
            if requested_mode != self._lane_mode:
                self._lane_mode = requested_mode
                self.confirmation.clear()
                self._publish_stop(False)
                self._publish_lane_lost(requested_mode)

    def _worker_loop(self):
        period = 1.0 / self.inference_hz
        next_request_time = time.monotonic()
        while not self._stop_event.is_set():
            with self._frame_condition:
                wait_time = max(
                    0.0,
                    next_request_time - time.monotonic(),
                )
                if wait_time > 0.0:
                    self._frame_condition.wait(timeout=wait_time)
                    continue
                frame_record = self._latest_frame
                self._latest_frame = None
                last_camera_time = self._last_camera_time
                lane_mode = self._lane_mode

            request_started = time.monotonic()
            next_request_time = request_started + period
            if frame_record is None:
                if (
                    last_camera_time is None
                    or request_started - last_camera_time
                    > self.camera_timeout
                ):
                    self._publish_failure(
                        'camera frame is stale or unavailable'
                    )
                continue

            msg, received_at = frame_record
            if request_started - received_at > self.camera_timeout:
                self._publish_failure('discarded stale camera frame')
                continue

            try:
                frame = self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding='bgr8',
                )
                transport_frame = resize_for_transport(
                    frame,
                    self.max_image_dimension,
                )
                encoded, jpeg = cv2.imencode(
                    '.jpg',
                    transport_frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if not encoded:
                    raise RuntimeError('OpenCV JPEG encoding failed')

                self._request_id += 1
                result = self.client.analyze(
                    jpeg.tobytes(),
                    self._request_id,
                    lane_mode,
                    self.confidence_threshold,
                    self.min_box_height_ratio,
                )
                with self._frame_condition:
                    if lane_mode == self._lane_mode:
                        self._publish_result(result)
                        continue
                    self._publish_failure(
                        'discarded response for an old lane mode'
                    )
            except Exception as exc:
                self._publish_failure(f'Mac vision request failed: {exc}')

    def _publish_result(self, result):
        confirmed = self.confirmation.update(result['stop'])
        self._publish_stop(confirmed)

        if result['centroid'] is not None:
            centroid = Float32()
            centroid.data = float(result['centroid'])
            self.centroid_publisher.publish(centroid)
        acquired = Bool()
        acquired.data = result['lane_acquired']
        self.lane_acquired_publisher.publish(acquired)
        status = String()
        status.data = result['lane_status']
        self.lane_status_publisher.publish(status)

        now = time.monotonic()
        if now - self._last_log_time >= 1.0:
            self.get_logger().info(
                f'Lane={status.data}; centroid={result["centroid"]}; '
                f'STOP confidence={result["confidence"]:.3f}; '
                f'box ratio={result["box_height_ratio"]:.3f}; '
                f'hits={self.confirmation.hits}/'
                f'{self.confirmation.required_hits}; '
                f'confirmed={self.confirmation.confirmed}'
            )
            self._last_log_time = now

    def _publish_failure(self, reason):
        with self._frame_condition:
            self.confirmation.clear()
            self._publish_stop(False)
            lane_mode = self._lane_mode
            self._publish_lane_lost(lane_mode)
        now = time.monotonic()
        if now - self._last_error_log_time >= 2.0:
            self.get_logger().warning(
                f'{reason}; publishing STOP=False and lane LOST'
            )
            self._last_error_log_time = now

    def _publish_stop(self, detected):
        message = Bool()
        message.data = bool(detected)
        self.stop_publisher.publish(message)

    def _publish_lane_lost(self, lane_mode):
        acquired = Bool()
        acquired.data = False
        self.lane_acquired_publisher.publish(acquired)
        status = String()
        status.data = f'{lane_mode}:LOST'
        self.lane_status_publisher.publish(status)

    def destroy_node(self):
        """Stop and join the worker before destroying ROS resources."""
        self._stop_event.set()
        with self._frame_condition:
            self._frame_condition.notify_all()
        if self._worker.is_alive():
            self._worker.join(timeout=self.client.timeout + 0.5)
        return super().destroy_node()


def main(args=None):
    """Run the combined Mac vision bridge."""
    rclpy.init(args=args)
    node = MacVisionBridgeNode()
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
