import time
import math
import traceback
from pyvesc import VESC
from pyvesc.protocol.interface import decode

VESC_NODE_NAME = 'vesc_client'
VESC_TOPIC_NAME = 'vesc'


class FramedResponseVESC(VESC):
    """Read complete VESC frames instead of guessing response lengths."""

    def _read_exact(self, size):
        data = bytearray()
        while len(data) < size:
            chunk = self.serial_port.read(size - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    def write(self, data, num_read_bytes=None):
        self.serial_port.write(data)

        if num_read_bytes is None:
            return None

        start = self._read_exact(1)
        if start is None:
            return None

        if start == b'\x02':
            length_bytes = self._read_exact(1)
        elif start == b'\x03':
            length_bytes = self._read_exact(2)
        else:
            raise ValueError(
                f'Invalid VESC packet start byte: 0x{start[0]:02x}'
            )

        if length_bytes is None:
            return None

        payload_length = int.from_bytes(length_bytes, byteorder='big')
        remainder = self._read_exact(payload_length + 3)
        if remainder is None:
            return None

        response, _ = decode(start + length_bytes + remainder)
        return response


class VESC_:
    def __init__(self):
        self.serial_port = "/dev/ttyACM0"
        self.baudrate = 115200
        self.is_inverted = False
        self.has_sensor = False
        self.start_heartbeat = False
        print("Connecting to VESC")
        for attempt in range(10):
            try:
                self.v = FramedResponseVESC(
                    serial_port=self.serial_port,
                    has_sensor=self.has_sensor,
                    start_heartbeat=self.start_heartbeat,
                    baudrate=self.baudrate,
                    timeout=1.0
                )
                print("VESC Connected")
                self.send_rpm(0)
                self.inverted = -1 if self.is_inverted else 1
                break
            except Exception as e:
                print(f"VESC attempt {attempt + 1}/10 failed:", repr(e))
                time.sleep(1)
        else:
            raise RuntimeError("Could not connect to VESC after 10 attempts")

    def print_firmware_version(self):
        print("VESC Firmware Version: ", self.v.get_firmware_version())

    def send_servo_angle(self, angle):
        self.v.set_servo(angle)

    def send_rpm(self, rpm):
        self.v.set_rpm(rpm)

    def send_duty_cycle(self, dc):
        # HACK. not used.
        self.v.set_duty_cycle(dc)

    def send_current(self, curr):
        self.v.set_current(curr)

    def get_rpm(self):
        return self.v.get_rpm() * self.inverted

    def get_motor_position(self):
        return self.v.get_motor_position()

    def get_input_voltage(self):
        measurements = self.v.get_measurements()

        if measurements is None:
            return None

        return float(measurements.v_in)


if __name__ == "__main__":
    print('MAKE SURE YOUR CAR IS ON A STAND AND WHEELS CAN SPIN FREELY')
    input('Hit ENTER to continue...')
    v = VESC_()
    v.print_firmware_version()
    backward_rpm = -10000
    steering_angle_left = 0.0  # in the range of [0, 1]
    steering_angle_right = 1.0
    steering_angle_straight = 0.5

    v.send_servo_angle(steering_angle_straight)

    print('right turn')
    time.sleep(2)
    v.send_servo_angle(steering_angle_right)

    print('turn straight')
    time.sleep(2)
    v.send_servo_angle(steering_angle_straight)
    
    print('go forward')
    time.sleep(2)
    v.send_rpm(15000)
    
    print('stop')
    time.sleep(2)
    v.send_rpm(0)
    #
    # n = 23.8 # gear ratio
    # forward_rpm = 3000
    # rpm_to_rps = 1/60
    # # forward_rps = forward_rpm * rpm_to_rps
    # wheel_radius = 5 #cm
    # time_to_run = 1
    # # num_wheel_rotations = (forward_rps * time_to_run / n) * 2 * math.pi # radians
    # # dist_trav = num_wheel_rotations * wheel_radius
    # # num_wheel_rotations = dist_trav / wheel_radius
    # # (forward_rps * time_to_run / n) * 2 * math.pi = dist_trav / wheel_radius
    # # forward_rps = (dist_trav / wheel_radius) * (n / (time_to_run*2 * math.pi))
    # # forward_rpm * rpm_to_rps = (dist_trav / wheel_radius) * (n / (time_to_run*2 * math.pi))
    #
    # dist_trav = 200
    # # forward_rpm = int((dist_trav / wheel_radius) * (n / time_to_run * 2 * math.pi) / rpm_to_rps)
    # forward_rpm = ((dist_trav / wheel_radius) * (n / (time_to_run * 2 * math.pi))) / rpm_to_rps
    # forward_rpm_int = int(forward_rpm)
    #
    # #
    # rev_estimate = forward_rpm_int * time_to_run / rpm_to_rps
    # v.send_rpm(forward_rpm)
    # time.sleep(1)
    # time.sleep(time_to_run)
    # vesc_rpm = v.get_rpm()
    #
    # print("Encoder Count (Calc): ", rev_estimate)
    # print("RPM sent (Calc): ", forward_rpm)
    # print("RPM sent (VESC): ", vesc_rpm)
    # print("Distance Traveled (Calc): ", dist_trav)
    # #
    # v.send_rpm(0)
    
    # v.send_servo_angle(steering_angle_straight)
    # print("Encoder Count (VESC): ", v.get_motor_position())
    # time.sleep(2)

    # v.send_rpm(backward_rpm)
    # time.sleep(2)
    # print("Encoder Count: ", v.get_motor_position())
