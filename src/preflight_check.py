#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import re
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


WATCH_TOPICS = [
    '/pit_state',
    '/lane_mode',
    '/battery_voltage',
    '/battery_percentage',
    '/battery_low',
    '/stop_sign_detected',
    '/lane_status',
    '/lane_cmd_vel',
    '/cmd_vel',
]


class PreflightChecker(Node):

    def __init__(self):
        super().__init__('team5_preflight_check')
        self.received = {}
        self.counts = {}
        self.subscriptions_keepalive = []

    def discover_and_subscribe(self):
        topic_map = dict(self.get_topic_names_and_types())

        for topic in WATCH_TOPICS:
            msg_types = topic_map.get(topic, [])
            if not msg_types:
                continue

            try:
                msg_type = get_message(msg_types[0])
            except Exception:
                continue

            self.counts[topic] = 0

            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, name=topic: self.record_message(name, msg),
                10,
            )

            self.subscriptions_keepalive.append(sub)

    def record_message(self, topic, msg):
        self.counts[topic] = self.counts.get(topic, 0) + 1

        if hasattr(msg, 'data'):
            self.received[topic] = msg.data
        else:
            self.received[topic] = '<message received>'


def command_output(command):
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None, '', ''


def check_onnx_model(results):
    try:
        import onnxruntime as ort

        share = Path(get_package_share_directory('team5_pitstop_pkg'))
        model = share / 'models' / 'best.onnx'

        if not model.exists():
            results.append(('FAIL', 'STOP model', f'missing: {model}'))
            return

        session = ort.InferenceSession(
            str(model),
            providers=['CPUExecutionProvider'],
        )

        shape = session.get_inputs()[0].shape

        if list(shape) != [1, 3, 640, 640]:
            results.append(
                ('WARN', 'STOP model', f'loads, unexpected input {shape}')
            )
            return

        results.append(
            ('PASS', 'STOP model', 'ONNX Runtime + best.onnx OK')
        )

    except Exception as exc:
        results.append(('FAIL', 'STOP model', str(exc)))


def check_pi_power(results):
    code, stdout, _ = command_output(['vcgencmd', 'get_throttled'])

    if code is None or code != 0:
        results.append(
            ('WARN', 'Pi power', 'vcgencmd unavailable in this environment')
        )
        return

    match = re.search(r'0x([0-9a-fA-F]+)', stdout)
    if not match:
        results.append(('WARN', 'Pi power', stdout or 'unreadable result'))
        return

    value = int(match.group(1), 16)

    current_undervoltage = bool(value & (1 << 0))
    current_throttled = bool(value & (1 << 2))
    past_undervoltage = bool(value & (1 << 16))
    past_throttled = bool(value & (1 << 18))

    if current_undervoltage or current_throttled:
        results.append(
            ('FAIL', 'Pi power', f'CURRENT power problem: 0x{value:x}')
        )
    elif past_undervoltage or past_throttled:
        results.append(
            ('WARN', 'Pi power',
             f'no current fault, historical event recorded: 0x{value:x}')
        )
    else:
        results.append(('PASS', 'Pi power', f'get_throttled=0x{value:x}'))


def check_pit_default(results):
    try:
        share = Path(get_package_share_directory('team5_pitstop_pkg'))
        launch_file = share / 'launch' / 'pitstop_integration.launch.py'
        text = launch_file.read_text()

        if (
            'pit_lane_enabled' in text
            and re.search(
                r"pit_lane_enabled.*?default_value\s*=\s*['\"]false['\"]",
                text,
                re.DOTALL | re.IGNORECASE,
            )
        ):
            results.append(
                ('PASS', 'Pit default', 'pit_lane_enabled defaults FALSE')
            )
        else:
            results.append(
                ('WARN', 'Pit default',
                 'could not prove disabled default from installed launch')
            )

    except Exception as exc:
        results.append(('WARN', 'Pit default', str(exc)))


def print_result(level, name, detail):
    print(f'{level:<5}  {name:<24} {detail}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--mode',
        choices=['stationary', 'motion'],
        default='stationary',
    )
    parser.add_argument(
        '--sample-seconds',
        type=float,
        default=2.0,
    )
    args = parser.parse_args()

    rclpy.init()
    node = PreflightChecker()
    results = []

    try:
        print()
        print('TEAM 5 ROBOCAR PREFLIGHT')
        print('=' * 68)
        print(f'Mode: {args.mode}')
        print()

        check_onnx_model(results)
        check_pi_power(results)
        check_pit_default(results)

        # Graph discovery.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)

        cmd_publishers = len(
            node.get_publishers_info_by_topic('/cmd_vel')
        )

        if cmd_publishers > 1:
            results.append(
                ('FAIL', '/cmd_vel publishers',
                 f'{cmd_publishers} publishers — duplicate control risk')
            )
        elif cmd_publishers == 1:
            results.append(
                ('PASS', '/cmd_vel publishers', 'exactly 1')
            )
        elif args.mode == 'motion':
            results.append(
                ('FAIL', '/cmd_vel publishers', 'none found')
            )
        else:
            results.append(
                ('WARN', '/cmd_vel publishers',
                 'none found — control stack may be off')
            )

        # Device presence only. Never open the serial device.
        if os.path.exists('/dev/ttyACM0'):
            results.append(
                ('PASS', 'VESC device', '/dev/ttyACM0 exists (not opened)')
            )
        elif args.mode == 'motion':
            results.append(
                ('FAIL', 'VESC device', '/dev/ttyACM0 missing')
            )
        else:
            results.append(
                ('WARN', 'VESC device',
                 '/dev/ttyACM0 missing — acceptable while hardware is off')
            )

        node.discover_and_subscribe()

        start = time.monotonic()
        end = start + max(0.5, args.sample_seconds)

        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        elapsed = max(time.monotonic() - start, 0.001)

        # Report telemetry.
        for topic in [
            '/pit_state',
            '/lane_mode',
            '/battery_voltage',
            '/battery_percentage',
            '/battery_low',
            '/stop_sign_detected',
            '/lane_status',
        ]:
            if topic in node.received:
                results.append(
                    ('PASS', topic, str(node.received[topic]))
                )
            elif args.mode == 'motion' and topic in [
                '/pit_state',
                '/lane_mode',
                '/battery_voltage',
                '/battery_low',
                '/lane_status',
            ]:
                results.append(
                    ('FAIL', topic, 'no fresh message received')
                )
            else:
                results.append(
                    ('WARN', topic, 'no message received')
                )

        # Command frequencies.
        for topic in ['/lane_cmd_vel', '/cmd_vel']:
            count = node.counts.get(topic, 0)
            rate = count / elapsed

            if count == 0:
                level = 'FAIL' if args.mode == 'motion' else 'WARN'
                results.append((level, topic + ' rate', '0 Hz'))
            else:
                if rate >= 10.0:
                    level = 'PASS'
                elif args.mode == 'motion':
                    level = 'FAIL'
                else:
                    level = 'WARN'

                results.append(
                    (level, topic + ' rate', f'{rate:.1f} Hz')
                )

        print(f'{"STATUS":<7}{"CHECK":<25}DETAIL')
        print('-' * 68)

        for level, name, detail in results:
            print_result(level, name, detail)

        failures = sum(level == 'FAIL' for level, _, _ in results)
        warnings = sum(level == 'WARN' for level, _, _ in results)

        print('-' * 68)

        if failures:
            print(f'RESULT: DO NOT DRIVE — {failures} failure(s), '
                  f'{warnings} warning(s)')
            exit_code = 2
        elif warnings:
            print(f'RESULT: PASS WITH WARNINGS — {warnings} warning(s)')
            exit_code = 0
        else:
            print('RESULT: PASS')
            exit_code = 0

        print()
        print('This checker is READ-ONLY and publishes no control topics.')
        print()

    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
