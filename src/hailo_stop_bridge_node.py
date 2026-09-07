"""ROS bridge from newest camera frames to the host Hailo service."""

import http.client
import json
import math
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
from std_msgs.msg import Bool


PROTOCOL_VERSION = 1
STOP_CLASS_ID = 11
STOP_LABEL = 'stop sign'
MAX_INFERENCE_HZ = 4.0
MAX_RESPONSE_BYTES = 16 * 1024


class ConfirmationFilter:
    """Match the legacy detector's consecutive-hit confirmation behavior."""

    def __init__(self, required_hits):
        if required_hits < 1:
            raise ValueError('required_hits must be positive')
        self.required_hits = int(required_hits)
        self.hits = 0
        self.confirmed = False

    def update(self, detected):
        if detected:
            self.hits += 1
            if self.hits >= self.required_hits:
                self.confirmed = True
        else:
            self.clear()
        return self.confirmed

    def clear(self):
        self.hits = 0
        self.confirmed = False
        return self.confirmed


def resize_for_transport(frame, max_dimension):
    """Downscale without upscaling while retaining the camera aspect ratio."""
    height, width = frame.shape[:2]
    if height < 1 or width < 1:
        raise ValueError('camera frame has invalid dimensions')
    scale = min(1.0, float(max_dimension) / max(height, width))
    if scale == 1.0:
        return frame
    output_width = max(1, round(width * scale))
    output_height = max(1, round(height * scale))
    return cv2.resize(
        frame,
        (output_width, output_height),
        interpolation=cv2.INTER_AREA,
    )


def validate_response(payload, expected_request_id, threshold):
    """Validate the complete response before accepting its detection."""
    if not isinstance(payload, dict):
        raise ValueError('response must be a JSON object')
    if payload.get('protocol_version') != PROTOCOL_VERSION:
        raise ValueError('response protocol version mismatch')
    if payload.get('request_id') != expected_request_id:
        raise ValueError('response request id mismatch')
    if payload.get('class_id') != STOP_CLASS_ID:
        raise ValueError('response class id mismatch')
    if payload.get('label') != STOP_LABEL:
        raise ValueError('response class label mismatch')
    if type(payload.get('stop')) is not bool:
        raise ValueError('response stop value is not boolean')

    confidence = payload.get('confidence')
    if isinstance(confidence, bool) or not isinstance(
        confidence,
        (int, float),
    ):
        raise ValueError('response confidence is not numeric')
    confidence = float(confidence)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError('response confidence is outside [0, 1]')

    # Both ends enforce the threshold. A false host result remains false even
    # if a mismatched host configuration reports a larger confidence.
    detected = payload['stop'] and confidence >= threshold
    return detected, confidence


class HailoBridgeClient:
    """Small, dependency-free HTTP client for one JPEG request."""

    def __init__(self, host, port, timeout):
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        if not self.host:
            raise ValueError('host must not be empty')
        if not 1 <= self.port <= 65535:
            raise ValueError('port must be between 1 and 65535')
        if self.timeout <= 0.0:
            raise ValueError('request timeout must be positive')

    def detect(self, jpeg_bytes, request_id, threshold):
        connection = http.client.HTTPConnection(
            self.host,
            self.port,
            timeout=self.timeout,
        )
        try:
            connection.request(
                'POST',
                '/v1/detect-stop',
                body=jpeg_bytes,
                headers={
                    'Content-Type': 'image/jpeg',
                    'X-Protocol-Version': str(PROTOCOL_VERSION),
                    'X-Request-Id': str(request_id),
                    'Connection': 'close',
                },
            )
            response = connection.getresponse()
            body = response.read(MAX_RESPONSE_BYTES + 1)
            if len(body) > MAX_RESPONSE_BYTES:
                raise ValueError('host response is too large')
            if response.status != 200:
                raise RuntimeError(
                    f'host returned HTTP {response.status}: '
                    f'{body.decode("utf-8", errors="replace")}'
                )
            payload = json.loads(body.decode('utf-8'))
            return validate_response(payload, request_id, threshold)
        finally:
            connection.close()


class HailoStopBridgeNode(Node):
    """Send at most four newest-frame requests per second to Hailo."""

    def __init__(self):
        super().__init__('hailo_stop_bridge_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', '127.0.0.1'),
                ('port', 18420),
                ('inference_hz', 4.0),
                ('confidence_threshold', 0.35),
                ('confirm_detections', 3),
                ('request_timeout', 0.75),
                ('camera_timeout', 0.75),
                ('jpeg_quality', 80),
                ('max_image_dimension', 640),
            ],
        )

        def parameter(name):
            return self.get_parameter(name).value

        self.inference_hz = float(parameter('inference_hz'))
        self.threshold = float(parameter('confidence_threshold'))
        self.camera_timeout = float(parameter('camera_timeout'))
        self.jpeg_quality = int(parameter('jpeg_quality'))
        self.max_image_dimension = int(parameter('max_image_dimension'))
        required_hits = int(parameter('confirm_detections'))

        self._validate_parameters(required_hits)
        self.client = HailoBridgeClient(
            str(parameter('host')),
            int(parameter('port')),
            float(parameter('request_timeout')),
        )
        self.bridge = CvBridge()
        self.confirmation = ConfirmationFilter(required_hits)
        self.publisher = self.create_publisher(
            Bool,
            '/stop_sign_detected',
            10,
        )

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_0',
            self.image_callback,
            camera_qos,
        )

        self._frame_condition = threading.Condition()
        self._latest_frame = None
        self._last_camera_time = None
        self._request_id = 0
        self._stop_event = threading.Event()
        self._last_log_time = 0.0
        self._last_error_log_time = 0.0
        self._worker = threading.Thread(
            target=self._worker_loop,
            name='hailo-stop-bridge-worker',
            daemon=True,
        )
        self._worker.start()

        self.get_logger().info(
            'Hailo STOP bridge ready: '
            f'{self.client.host}:{self.client.port}; '
            f'threshold={self.threshold:.2f}; '
            f'required detections={required_hits}; '
            f'inference={self.inference_hz:.1f} Hz'
        )

    def _validate_parameters(self, required_hits):
        if not 0.0 < self.inference_hz <= MAX_INFERENCE_HZ:
            raise ValueError('inference_hz must be greater than 0 and at most 4')
        if not 0.0 <= self.threshold <= 1.0:
            raise ValueError('confidence_threshold must be in [0, 1]')
        if self.camera_timeout <= 0.0:
            raise ValueError('camera_timeout must be positive')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')
        if not 1 <= self.max_image_dimension <= 640:
            raise ValueError('max_image_dimension must be in [1, 640]')
        if required_hits < 1:
            raise ValueError('confirm_detections must be positive')

    def image_callback(self, msg):
        """Replace the one pending frame without image conversion or blocking."""
        received_at = time.monotonic()
        with self._frame_condition:
            self._latest_frame = (msg, received_at)
            self._last_camera_time = received_at
            self._frame_condition.notify()

    def _worker_loop(self):
        period = 1.0 / self.inference_hz
        next_request_time = time.monotonic()

        while not self._stop_event.is_set():
            with self._frame_condition:
                wait_time = max(0.0, next_request_time - time.monotonic())
                if wait_time > 0.0:
                    self._frame_condition.wait(timeout=wait_time)
                    continue
                frame_record = self._latest_frame
                self._latest_frame = None
                last_camera_time = self._last_camera_time

            request_started = time.monotonic()
            next_request_time = request_started + period

            if frame_record is None:
                if (
                    last_camera_time is None
                    or request_started - last_camera_time
                    > self.camera_timeout
                ):
                    self._publish_clear('camera frame is stale or unavailable')
                continue

            msg, received_at = frame_record
            if request_started - received_at > self.camera_timeout:
                self._publish_clear('discarded stale camera frame')
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
                request_id = self._request_id
                detected, confidence = self.client.detect(
                    jpeg.tobytes(),
                    request_id,
                    self.threshold,
                )
                confirmed = self.confirmation.update(detected)
                self._publish(confirmed)
                self._log_status(confidence)
            except Exception as exc:
                self.confirmation.clear()
                self._publish(False)
                self._log_error(exc)

    def _publish_clear(self, reason):
        self.confirmation.clear()
        self._publish(False)
        now = time.monotonic()
        if now - self._last_error_log_time >= 2.0:
            self.get_logger().warning(reason)
            self._last_error_log_time = now

    def _publish(self, detected):
        message = Bool()
        message.data = bool(detected)
        self.publisher.publish(message)

    def _log_status(self, confidence):
        now = time.monotonic()
        if now - self._last_log_time < 1.0:
            return
        self.get_logger().info(
            f'Confidence: {confidence:.3f} | '
            f'Hits: {self.confirmation.hits}/'
            f'{self.confirmation.required_hits} | '
            f'STOP: {self.confirmation.confirmed}'
        )
        self._last_log_time = now

    def _log_error(self, error):
        now = time.monotonic()
        if now - self._last_error_log_time < 2.0:
            return
        self.get_logger().error(
            f'Hailo STOP bridge request failed; publishing False: {error}'
        )
        self._last_error_log_time = now

    def destroy_node(self):
        self._stop_event.set()
        with self._frame_condition:
            self._frame_condition.notify_all()
        if self._worker.is_alive():
            self._worker.join(timeout=self.client.timeout + 0.5)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HailoStopBridgeNode()
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
