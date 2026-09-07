import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist


class WhitePathDebug(Node):

    def __init__(self):
        super().__init__('white_path_debug')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_0',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.last_print = 0.0

        self.get_logger().info(
            'EDGE-GUARD DRIVE TEST running - MOTOR OUTPUT ENABLED'
        )

    def drive_watchdog(self):
        if hasattr(self, 'cmd_pub'):
            if time.time() - self.last_good_drive > 0.30:
                self.cmd_pub.publish(Twist())

    def image_callback(self, msg):
        if not hasattr(self, 'cmd_pub'):
            self.cmd_pub = self.create_publisher(
                Twist,
                '/team5_lane_cmd_vel',
                10
            )
            self.last_good_drive = 0.0
            self.drive_start_time = time.time()
            self.good_drive_frames = 0
            self.watchdog_timer = self.create_timer(
                0.10,
                self.drive_watchdog
            )

        drive_cmd = Twist()
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        frame_h, frame_w = frame.shape[:2]

        # Same idea as our working detector:
        # ignore only the very top 5 percent.
        top = int(frame_h * 0.05)
        roi = frame[top:, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Normalize brightness so sunny/cloudy changes
        # do not require a different fixed V threshold.
        h, s_channel, v = cv2.split(hsv)

        clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )

        v_normalized = clahe.apply(v)

        hsv_normalized = cv2.merge(
            (h, s_channel, v_normalized)
        )

        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 70, 255])

        mask = cv2.inRange(
            hsv_normalized,
            white_lower,
            white_upper
        )

        # SAME morphology idea as working detector.
        kernel = np.ones((3, 3), np.uint8)

        mask = cv2.blur(mask, (3, 3))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=4)

        _, mask = cv2.threshold(
            mask,
            61,
            255,
            cv2.THRESH_BINARY
        )

        roi_h = roi.shape[0]

        # We start near the car and walk UP the image.
        band_height = 35

        # Don't require the line to reach the horizon.
        stop_y = int(roi_h * 0.55)

        points = []
        previous_x = None

        y_bottom = roi_h

        while y_bottom > stop_y:

            y_top = max(
                stop_y,
                y_bottom - band_height
            )

            band = mask[y_top:y_bottom, :]

            if previous_x is None:
                # First piece must originate on the RIGHT side.
                # Ignore the extreme-right image edge during
                # initial acquisition.
                search_left = int(frame_w * 0.62)
                search_right = int(frame_w * 0.97)
            else:
                # Once locked, only look near where the
                # SAME boundary was in the previous band.
                search_radius = 120

                search_left = max(
                    0,
                    previous_x - search_radius
                )

                search_right = min(
                    frame_w,
                    previous_x + search_radius
                )

            search = band[
                :,
                search_left:search_right
            ]

            if search.size == 0:
                y_bottom = y_top
                continue

            histogram = np.count_nonzero(
                search,
                axis=0
            )

            if histogram.size == 0:
                y_bottom = y_top
                continue

            peak_value = int(histogram.max())

            # Require actual white support in this horizontal band.
            min_peak = 30 if previous_x is None else 3
            if peak_value < min_peak:
                y_bottom = y_top
                continue

            if previous_x is None:
                # On acquisition, prefer the RIGHTMOST strong peak.
                threshold = max(
                    3,
                    int(peak_value * 0.60)
                )

                strong = np.where(
                    histogram >= threshold
                )[0]

                peak_index = int(strong[-1])

            else:
                # After acquisition, strongest nearby continuation wins.
                peak_index = int(
                    np.argmax(histogram)
                )

            x_guess = search_left + peak_index

            # Find white pixels around that peak so we get
            # a real centroid rather than one histogram column.
            x1 = max(0, x_guess - 12)
            x2 = min(frame_w, x_guess + 13)

            local = band[:, x1:x2]

            ys, xs = np.nonzero(local)

            if len(xs) >= 5:

                point_x = int(
                    np.median(xs) + x1
                )

                point_y_roi = int(
                    np.median(ys) + y_top
                )

                point_y_frame = (
                    point_y_roi + top
                )

                # Stop if we lost the boundary for too large a vertical gap.
                if points:
                    y_gap = points[-1][1] - point_y_frame
                    if y_gap > 80:
                        break

                # Predict where the SAME boundary should continue.
                # This rejects sudden sideways jumps onto unrelated objects.
                if len(points) >= 2:
                    predicted_x = (
                        points[-1][0]
                        + (
                            points[-1][0]
                            - points[-2][0]
                        )
                    )

                    lateral_error = abs(
                        point_x - predicted_x
                    )

                    if lateral_error > 90:
                        y_bottom = y_top
                        continue

                points.append(
                    (point_x, point_y_frame)
                )

                previous_x = point_x

            y_bottom = y_top

        debug = frame.copy()

        # Draw tracked samples.
        for x, y in points:
            cv2.circle(
                debug,
                (x, y),
                7,
                (0, 255, 0),
                -1
            )

        fit_text = 'NO FIT'

        # Fit x as a function of y.
        if len(points) >= 3:

            ys = np.array(
                [p[1] for p in points],
                dtype=np.float32
            )

            xs = np.array(
                [p[0] for p in points],
                dtype=np.float32
            )

            degree = 2 if len(points) >= 5 else 1

            coeff = np.polyfit(
                ys,
                xs,
                degree
            )

            curve_ys = np.linspace(
                min(ys),
                max(ys),
                80
            )

            curve_xs = np.polyval(
                coeff,
                curve_ys
            )

            previous = None
            virtual_points = []

            y_min = float(min(ys))
            y_max = float(max(ys))
            y_span = max(1.0, y_max - y_min)

            for curve_x, curve_y in zip(
                curve_xs,
                curve_ys
            ):

                px = int(curve_x)
                py = int(curve_y)

                if (
                    0 <= px < frame_w
                    and 0 <= py < frame_h
                ):
                    # BLUE = detected right white boundary.
                    if previous is not None:
                        cv2.line(
                            debug,
                            previous,
                            (px, py),
                            (255, 0, 0),
                            3
                        )

                    previous = (px, py)

                    # Move desired driving path LEFT,
                    # farther toward the middle of the lane.
                    perspective = (
                        (float(curve_y) - y_min)
                        / y_span
                    )

                    # Far ahead: about 120 px left.
                    # Near car: about 340 px left.
                    offset_px = (
                        180.0
                        + 330.0 * perspective
                    )

                    virtual_x = int(
                        curve_x - offset_px
                    )

                    virtual_y = py

                    if (
                        0 <= virtual_x < frame_w
                        and 0 <= virtual_y < frame_h
                    ):
                        virtual_points.append(
                            (virtual_x, virtual_y)
                        )

            # MAGENTA = desired driving path.
            virtual_previous = None

            for virtual_point in virtual_points:

                if virtual_previous is not None:
                    cv2.line(
                        debug,
                        virtual_previous,
                        virtual_point,
                        (255, 0, 255),
                        4
                    )

                virtual_previous = virtual_point

            # RED = lookahead steering target.
            if virtual_points:

                lookahead_index = int(
                    len(virtual_points) * 0.45
                )

                lookahead_index = max(
                    0,
                    min(
                        len(virtual_points) - 1,
                        lookahead_index
                    )
                )

                lookahead = virtual_points[
                    lookahead_index
                ]

                # STEERING DEBUG ONLY - NO MOTOR OUTPUT
                frame_center_x = int(
                    frame_w * 0.50
                )

                steering_error = float(
                    (
                        lookahead[0]
                        - frame_center_x
                    )
                    / (frame_w * 0.50)
                )

                steering_cmd = float(
                    np.clip(
                        steering_error,
                        -0.60,
                        0.60
                    )
                )

                if (
                    len(points) >= 7
                    and abs(steering_error) <= 0.45
                    and time.time() - self.drive_start_time >= 1.0
                ):
                    self.good_drive_frames += 1
                else:
                    self.good_drive_frames = 0

                if self.good_drive_frames >= 3:
                    drive_cmd.linear.x = 0.40
                    drive_cmd.angular.z = float(
                        np.clip(
                            steering_cmd,
                            -0.35,
                            0.35
                        )
                    )
                    self.last_good_drive = time.time()

                if (
                    time.time()
                    - self.last_print
                    >= 1.0
                ):
                    print(
                        f'[STEER DEBUG] '
                        f'lookahead={lookahead} '
                        f'center_x={frame_center_x} '
                        f'error={steering_error:+.3f} '
                        f'steering={steering_cmd:+.3f}',
                        flush=True
                    )

                cv2.circle(
                    debug,
                    lookahead,
                    12,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    debug,
                    'MAGENTA=center path  RED=lookahead',
                    (15, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 255),
                    2
                )

            fit_text = (
                f'degree={degree}'
            )

        cv2.putText(
            debug,
            f'points={len(points)} {fit_text}',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        self.cmd_pub.publish(drive_cmd)

        # Save latest frame so we can inspect it.
        cv2.imwrite(
            '/tmp/team5_white_path.jpg',
            debug
        )

        cv2.imwrite(
            '/tmp/team5_white_path_mask.jpg',
            mask
        )

        now = time.time()

        if now - self.last_print >= 1.0:
            print(
                f'[PATH DEBUG] points={points}',
                flush=True
            )

            self.last_print = now


def main():
    rclpy.init()
    node = WhitePathDebug()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
