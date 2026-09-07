#!/usr/bin/env python3

import os
import time

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


TOPICS = [
    '/pit_state',
    '/lane_mode',
    '/lane_status',
    '/lane_acquired',
    '/battery_voltage',
    '/battery_percentage',
    '/battery_low',
    '/stop_sign_detected',
    '/lane_cmd_vel',
    '/cmd_vel',
]


class Team5Status(Node):

    def __init__(self):
        super().__init__('team5_status')

        self.values = {}
        self.last_seen = {}
        self.counts = {}
        self.last_rate_time = time.monotonic()
        self.rates = {}
        self.subscriptions_keepalive = []

        self.create_timer(1.0, self.discover_topics)
        self.create_timer(0.25, self.render)

    def discover_topics(self):
        topic_map = dict(self.get_topic_names_and_types())

        subscribed = {
            sub.topic_name
            for sub in self.subscriptions_keepalive
        }

        for topic in TOPICS:
            if topic in subscribed:
                continue

            types = topic_map.get(topic, [])
            if not types:
                continue

            try:
                msg_type = get_message(types[0])
            except Exception:
                continue

            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, name=topic: self.callback(name, msg),
                10,
            )

            self.subscriptions_keepalive.append(sub)
            self.counts.setdefault(topic, 0)

    def callback(self, topic, msg):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.last_seen[topic] = time.monotonic()

        if hasattr(msg, 'linear') and hasattr(msg, 'angular'):
            self.values[topic] = {
                'linear': float(msg.linear.x),
                'angular': float(msg.angular.z),
            }
        elif hasattr(msg, 'data'):
            self.values[topic] = msg.data
        else:
            self.values[topic] = str(msg)

    def update_rates(self):
        now = time.monotonic()
        elapsed = now - self.last_rate_time

        if elapsed < 1.0:
            return

        for topic in ['/lane_cmd_vel', '/cmd_vel']:
            count = self.counts.get(topic, 0)
            self.rates[topic] = count / elapsed
            self.counts[topic] = 0

        self.last_rate_time = now

    def age(self, topic):
        last_seen = self.last_seen.get(topic)
        if last_seen is None:
            return None
        return max(0.0, time.monotonic() - last_seen)

    def value(self, topic, default='---', stale_after=None):
        value = self.values.get(topic, default)

        if value == default:
            return default

        if stale_after is not None:
            age = self.age(topic)
            if age is None or age > stale_after:
                return f'STALE ({age:.1f}s)' if age is not None else 'STALE'

        return value

    def format_cmd(self, topic):
        value = self.values.get(topic)

        if not isinstance(value, dict):
            return '---'

        return (
            f"x={value['linear']:+.2f}   "
            f"steer={value['angular']:+.2f}"
        )

    def format_float(self, topic, suffix='', stale_after=None):
        value = self.values.get(topic)

        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return '---'

        if stale_after is not None:
            age = self.age(topic)
            if age is None or age > stale_after:
                return f'STALE ({age:.1f}s)' if age is not None else 'STALE'

        return f'{value:.2f}{suffix}'

    def render(self):
        self.update_rates()

        # ANSI clear-screen + cursor-home.
        print('\033[2J\033[H', end='')

        print('TEAM 5 ROBOCAR STATUS')
        print('=' * 58)
        print('READ ONLY — NO CONTROL TOPICS ARE PUBLISHED')
        print('-' * 58)

        print(f'PIT STATE       {self.value("/pit_state")}')
        print(f'LANE MODE       {self.value("/lane_mode")}')
        print(
            f'LANE STATUS     '
            f'{self.value("/lane_status", stale_after=1.0)}'
        )
        print(
            f'LANE ACQUIRED   '
            f'{self.value("/lane_acquired", stale_after=1.0)}'
        )

        print('-' * 58)

        print(
            f'BATTERY         '
            f'{self.format_float("/battery_voltage", " V", 2.5)}'
        )
        print(
            f'PERCENT         '
            f'{self.format_float("/battery_percentage", " %", 2.5)}'
        )
        print(
            f'BATTERY LOW     '
            f'{self.value("/battery_low", stale_after=2.5)}'
        )
        print(
            f'STOP SIGN       '
            f'{self.value("/stop_sign_detected", stale_after=1.0)}'
        )

        print('-' * 58)

        print(f'LANE CMD        {self.format_cmd("/lane_cmd_vel")}')
        print(f'FINAL CMD       {self.format_cmd("/cmd_vel")}')

        lane_rate = self.rates.get('/lane_cmd_vel')
        cmd_rate = self.rates.get('/cmd_vel')

        lane_rate_text = (
            f'{lane_rate:.1f} Hz' if lane_rate is not None else '---'
        )
        cmd_rate_text = (
            f'{cmd_rate:.1f} Hz' if cmd_rate is not None else '---'
        )

        print(f'LANE RATE       {lane_rate_text}')
        print(f'CMD RATE        {cmd_rate_text}')

        print('-' * 58)

        cmd_publishers = len(
            self.get_publishers_info_by_topic('/cmd_vel')
        )

        if cmd_publishers == 1:
            pub_status = 'OK — exactly 1'
        elif cmd_publishers == 0:
            pub_status = 'STACK OFF / NONE'
        else:
            pub_status = f'WARNING — {cmd_publishers} publishers'

        print(f'/cmd_vel PUBS   {pub_status}')

        vesc_present = os.path.exists('/dev/ttyACM0')
        print(
            f'VESC DEVICE     '
            f'{"/dev/ttyACM0 PRESENT" if vesc_present else "NOT PRESENT"}'
        )

        print('=' * 58)
        print('Ctrl+C to exit.')


def main():
    rclpy.init()
    node = Team5Status()

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
