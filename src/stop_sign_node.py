import os
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


MODEL_PATH = os.path.join(
    get_package_share_directory('team5_pitstop_pkg'),
    'models',
    'best.onnx',
)

# -----------------------------
# YOLO SETTINGS
# -----------------------------

CONFIDENCE_THRESHOLD = 0.35


# Require multiple successful detections before saying STOP.
CONFIRM_DETECTIONS = 3

# Do not run YOLO on every 15-18 Hz camera frame.
# Keeps CPU available for lane following.
INFERENCE_HZ = 4.0


class StopSignNode(Node):

    def __init__(self):
        super().__init__("stop_sign_node")

        self.bridge = CvBridge()

        # Limit ONNX CPU use so lane following still has resources.
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1

        self.session = ort.InferenceSession(
            MODEL_PATH,
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )

        self.input_info = self.session.get_inputs()[0]
        self.input_name = self.input_info.name

        shape = self.input_info.shape

        self.input_height = (
            int(shape[2])
            if isinstance(shape[2], int)
            else 640
        )

        self.input_width = (
            int(shape[3])
            if isinstance(shape[3], int)
            else 640
        )

        self.publisher = self.create_publisher(
            Bool,
            "/stop_sign_detected",
            10,
        )

        self.subscription = self.create_subscription(
            Image,
            "/camera/color/image_0",
            self.image_callback,
            10,
        )

        self.last_inference_time = 0.0

        self.consecutive_detections = 0
        self.confirmed_detection = False

        self.last_log_time = 0.0

        self.get_logger().info(
            f"STOP-sign detector ready. "
            f"Threshold={CONFIDENCE_THRESHOLD:.2f}, "
            f"required detections={CONFIRM_DETECTIONS}, "
            f"inference={INFERENCE_HZ:.1f} Hz"
        )

        self.get_logger().info(
            f"Model input: "
            f"{self.input_width}x{self.input_height}"
        )


    def image_callback(self, msg):

        now = time.monotonic()

        # Throttle inference.
        if now - self.last_inference_time < (1.0 / INFERENCE_HZ):
            return

        self.last_inference_time = now

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            # Resize for YOLO.
            resized = cv2.resize(
                frame,
                (self.input_width, self.input_height),
            )

            # YOLO expects RGB.
            rgb = cv2.cvtColor(
                resized,
                cv2.COLOR_BGR2RGB,
            )

            # HWC -> CHW
            input_tensor = np.transpose(
                rgb,
                (2, 0, 1),
            )

            input_tensor = (
                input_tensor.astype(np.float32) / 255.0
            )

            # Add batch dimension.
            input_tensor = np.expand_dims(
                input_tensor,
                axis=0,
            )

            outputs = self.session.run(
                None,
                {self.input_name: input_tensor},
            )

            predictions = np.squeeze(outputs[0])

            # Standard Ultralytics YOLO ONNX output
            # is commonly [channels, detections].
            if (
                predictions.ndim == 2
                and predictions.shape[0] < predictions.shape[1]
                and predictions.shape[0] <= 100
            ):
                predictions = predictions.T

            max_confidence = 0.0

            if (
                predictions.ndim == 2
                and predictions.shape[1] >= 5
                and len(predictions) > 0
            ):

                # best.onnx is a one-class STOP-sign model.
                class_scores = predictions[:, 4:]

                max_confidence = float(
                    np.max(class_scores)
                )

            raw_detection = (
                max_confidence >= CONFIDENCE_THRESHOLD
            )

            # ------------------------------------
            # Consecutive detection protection
            # ------------------------------------

            if raw_detection:

                self.consecutive_detections += 1

            else:

                self.consecutive_detections = 0
                self.confirmed_detection = False

            if (
                self.consecutive_detections
                >= CONFIRM_DETECTIONS
            ):

                self.confirmed_detection = True

            detection_msg = Bool()
            detection_msg.data = self.confirmed_detection

            self.publisher.publish(detection_msg)

            # Print status roughly once per second.
            if now - self.last_log_time >= 1.0:

                self.get_logger().info(
                    f"Confidence: {max_confidence:.3f} | "
                    f"Hits: "
                    f"{self.consecutive_detections}/"
                    f"{CONFIRM_DETECTIONS} | "
                    f"STOP: {self.confirmed_detection}"
                )

                self.last_log_time = now

        except Exception as error:

            self.get_logger().error(
                f"STOP-sign inference error: {error}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = StopSignNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
