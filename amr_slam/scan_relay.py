#!/usr/bin/env python3
"""
Scan QoS Relay Node
Subscribes to /scan with Best Effort QoS (matching YDLidar)
Re-publishes on /scan_reliable with Reliable QoS (matching SLAM Toolbox)
Optionally masks blocked angular sectors before republishing.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.declare_parameter('enable_sector_mask', True)
        # Pairs of [start_deg, end_deg, start_deg, end_deg, ...]
        # Default masks left/right side sectors.
        self.declare_parameter(
            'blocked_sectors_deg',
            [-120.0, -60.0, 60.0, 120.0],
        )

        self.enable_sector_mask = bool(
            self.get_parameter('enable_sector_mask').value
        )
        blocked_degrees = list(
            self.get_parameter('blocked_sectors_deg').value
        )
        self.blocked_sectors_rad = self._parse_sector_pairs(blocked_degrees)
        self._last_size_warn_time_ns = 0

        # Subscribe with Best Effort (matches YDLidar)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Publish with Reliable (matches SLAM Toolbox)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(LaserScan, '/scan_reliable', pub_qos)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sub_qos
        )

        self.get_logger().info(
            f'Scan relay: /scan (BestEffort) -> /scan_reliable (Reliable), '
            f'sector_mask={"enabled" if self.enable_sector_mask else "disabled"}, '
            f'sectors_deg={blocked_degrees}'
        )

    def _parse_sector_pairs(self, values):
        if len(values) % 2 != 0:
            self.get_logger().warn(
                'blocked_sectors_deg must contain start/end pairs; ignoring last value.'
            )
            values = values[:-1]

        sectors = []
        for i in range(0, len(values), 2):
            start = math.radians(float(values[i]))
            end = math.radians(float(values[i + 1]))
            sectors.append((self._normalize_angle(start), self._normalize_angle(end)))
        return sectors

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _angle_is_blocked(self, angle):
        norm_angle = self._normalize_angle(angle)
        for start, end in self.blocked_sectors_rad:
            if start <= end:
                if start <= norm_angle <= end:
                    return True
            else:
                if norm_angle >= start or norm_angle <= end:
                    return True
        return False

    @staticmethod
    def _expected_scan_size(msg: LaserScan):
        if msg.angle_increment == 0.0:
            return len(msg.ranges)
        return int(round((msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1

    def _normalize_scan_sizes(self, msg: LaserScan, ranges, intensities):
        expected = self._expected_scan_size(msg)
        current = len(ranges)
        if current == expected:
            return ranges, intensities

        if current < expected:
            pad = expected - current
            ranges.extend([float('inf')] * pad)
            intensities.extend([0.0] * pad)
        else:
            del ranges[expected:]
            del intensities[expected:]

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_size_warn_time_ns > 5_000_000_000:
            self.get_logger().warn(
                f'Normalized LaserScan size from {current} to {expected} samples.'
            )
            self._last_size_warn_time_ns = now_ns

        return ranges, intensities

    def scan_callback(self, msg: LaserScan):
        if not self.enable_sector_mask or not self.blocked_sectors_rad:
            self.pub.publish(msg)
            return

        masked_scan = LaserScan()
        masked_scan.header = msg.header
        masked_scan.angle_min = msg.angle_min
        masked_scan.angle_max = msg.angle_max
        masked_scan.angle_increment = msg.angle_increment
        masked_scan.time_increment = msg.time_increment
        masked_scan.scan_time = msg.scan_time
        masked_scan.range_min = msg.range_min
        masked_scan.range_max = msg.range_max

        ranges = list(msg.ranges)
        intensities = list(msg.intensities)

        for i in range(len(ranges)):
            angle = msg.angle_min + (i * msg.angle_increment)
            if self._angle_is_blocked(angle):
                ranges[i] = float('inf')
                if i < len(intensities):
                    intensities[i] = 0.0

        ranges, intensities = self._normalize_scan_sizes(msg, ranges, intensities)
        masked_scan.ranges = ranges
        masked_scan.intensities = intensities
        self.pub.publish(masked_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
