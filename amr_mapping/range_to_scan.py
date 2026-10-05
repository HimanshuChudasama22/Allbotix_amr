#!/usr/bin/env python3
"""Convert ultrasonic Range messages into LaserScan arcs."""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, Range


SENSOR_NAMES = ("us_center", "us_right", "us_left")

SENSOR_DEFAULTS = {
    "us_center": {
        "range_topic": "/range_center_filtered",
        "scan_topic": "/scan_us_center",
        "max_dist": 1.5,
        "min_dist": 0.12,
        "max_scan_density": 21,
        "dist_density_ratio": 1.0,
        "inflation": 1.0,
        "enable_variable_range": True,
        "min_linear_speed": 0.15,
        "max_linear_speed": 3.0,
        "min_angular_speed": 0.1,
        "max_angular_speed": 0.7,
        "dist_to_angular_weight_ratio": 0.0,
        "stopped_min_dist": 0.25,
        "enable_clear_scan": True,
        "clear_range": 2.8,
    },
    "us_right": {
        "range_topic": "/range_right_filtered",
        "scan_topic": "/scan_us_right",
        "max_dist": 0.8,
        "min_dist": 0.12,
        "max_scan_density": 11,
        "dist_density_ratio": 1.0,
        "inflation": 1.0,
        "enable_variable_range": True,
        "min_linear_speed": 0.15,
        "max_linear_speed": 3.0,
        "min_angular_speed": 0.1,
        "max_angular_speed": 0.7,
        "dist_to_angular_weight_ratio": 0.0,
        "stopped_min_dist": 0.25,
        "enable_clear_scan": True,
        "clear_range": 2.8,
    },
    "us_left": {
        "range_topic": "/range_left_filtered",
        "scan_topic": "/scan_us_left",
        "max_dist": 0.8,
        "min_dist": 0.12,
        "max_scan_density": 11,
        "dist_density_ratio": 1.0,
        "inflation": 1.0,
        "enable_variable_range": True,
        "min_linear_speed": 0.15,
        "max_linear_speed": 3.0,
        "min_angular_speed": 0.1,
        "max_angular_speed": 0.7,
        "dist_to_angular_weight_ratio": 0.0,
        "stopped_min_dist": 0.25,
        "enable_clear_scan": True,
        "clear_range": 2.8,
    },
}

NODE_DEFAULTS = {
    "odom_topic": "/diff_drive_controller/odom",
    "cmd_vel_topic": "/cmd_vel",
}

# Match the A21 Range publisher so subscriptions connect.
RANGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ros2 topic echo / RViz default subscriptions are RELIABLE.
SCAN_RELIABLE_QOS = RANGE_QOS
# Nav2 ObstacleLayer uses sensor-data (BEST_EFFORT).
SCAN_SENSOR_QOS = qos_profile_sensor_data


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def normalize_speed(value: float, minimum: float, maximum: float) -> float:
    """Map |speed| into [0, 1] between the configured min and max."""
    speed = abs(value)
    if maximum <= minimum:
        return 1.0 if speed >= maximum else 0.0
    return clamp01((speed - minimum) / (maximum - minimum))


def speed_blend_factor(
    linear_speed: float,
    angular_speed: float,
    min_linear_speed: float,
    max_linear_speed: float,
    min_angular_speed: float,
    max_angular_speed: float,
    dist_to_angular_weight_ratio: float,
) -> float:
    """Blend normalized linear and angular speed.

    dist_to_angular_weight_ratio:
      0.0 -> linear speed only
      1.0 -> angular speed only
    """
    weight = clamp01(dist_to_angular_weight_ratio)
    linear = normalize_speed(linear_speed, min_linear_speed, max_linear_speed)
    angular = normalize_speed(angular_speed, min_angular_speed, max_angular_speed)
    return (1.0 - weight) * linear + weight * angular


@dataclass
class SensorConfig:
    name: str
    range_topic: str
    scan_topic: str
    max_dist: float
    min_dist: float
    max_scan_density: int
    dist_density_ratio: float
    inflation: float
    enable_variable_range: bool
    min_linear_speed: float
    max_linear_speed: float
    min_angular_speed: float
    max_angular_speed: float
    dist_to_angular_weight_ratio: float
    stopped_min_dist: float
    enable_clear_scan: bool
    clear_range: float

    def point_count(self, range_m: float) -> int:
        """Fewer points at longer range when dist_density_ratio > 0."""
        if self.max_scan_density <= 1:
            return 1
        if self.dist_density_ratio <= 0.0 or range_m <= self.min_dist:
            return self.max_scan_density

        scale = (self.min_dist / range_m) ** self.dist_density_ratio
        return max(1, int(round(self.max_scan_density * scale)))

    def effective_max_dist(self, linear_speed: float, angular_speed: float) -> float:
        if not self.enable_variable_range:
            return self.max_dist

        factor = speed_blend_factor(
            linear_speed,
            angular_speed,
            self.min_linear_speed,
            self.max_linear_speed,
            self.min_angular_speed,
            self.max_angular_speed,
            self.dist_to_angular_weight_ratio,
        )
        low = min(max(self.stopped_min_dist, self.min_dist), self.max_dist)
        return low + (self.max_dist - low) * factor

    def clear_scan_range_max(self) -> float:
        """LaserScan.range_max for an empty-cone clear ray."""
        return max(self.max_dist, self.clear_range)

    def clear_scan_hit(self) -> float:
        """Finite range strictly below range_max so ObstacleLayer keeps the beam.

        Must also sit beyond local obstacle_max_range so the endpoint is not marked.
        """
        range_max = self.clear_scan_range_max()
        hit = min(self.clear_range, range_max - 0.001)
        return max(hit, self.min_dist + 0.001)


class RangeToScanNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "range_to_scan",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.sensors: dict[str, SensorConfig] = {}
        self.reliable_publishers = {}
        self.sensor_publishers = {}
        self._got_range = {name: False for name in SENSOR_NAMES}

        self.odom_topic = str(self._node_param("odom_topic"))
        self.cmd_vel_topic = str(self._node_param("cmd_vel_topic"))

        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._odom_linear = 0.0
        self._odom_angular = 0.0
        self._got_speed = False

        for name in SENSOR_NAMES:
            config = self._load_sensor(name)
            self.sensors[name] = config
            self.reliable_publishers[name] = self.create_publisher(
                LaserScan, config.scan_topic, SCAN_RELIABLE_QOS
            )
            self.sensor_publishers[name] = self.create_publisher(
                LaserScan, config.scan_topic, SCAN_SENSOR_QOS
            )
            callback = self._make_callback(name)
            # Ultrasound publishes RELIABLE; keep a BEST_EFFORT listener too.
            self.create_subscription(Range, config.range_topic, callback, RANGE_QOS)
            self.create_subscription(
                Range, config.range_topic, callback, qos_profile_sensor_data
            )
            self.get_logger().info(
                f"{name}: {config.range_topic} -> {config.scan_topic} "
                f"[{config.min_dist:.2f}, {config.max_dist:.2f}] m, "
                f"density={config.max_scan_density}, "
                f"ratio={config.dist_density_ratio:.2f}, "
                f"inflation={config.inflation:.2f}, "
                f"variable_range="
                f"{'on' if config.enable_variable_range else 'off'} "
                f"lin=[{config.min_linear_speed:.2f}, {config.max_linear_speed:.2f}] "
                f"ang=[{config.min_angular_speed:.2f}, {config.max_angular_speed:.2f}] "
                f"dist_to_angular={config.dist_to_angular_weight_ratio:.2f} "
                f"stopped_min_dist={config.stopped_min_dist:.2f} m "
                f"clear_scan={'on' if config.enable_clear_scan else 'off'} "
                f"clear_range={config.clear_range:.2f} m"
            )

        if any(cfg.enable_variable_range for cfg in self.sensors.values()):
            if self.odom_topic:
                self.create_subscription(
                    Odometry, self.odom_topic, self._on_odom, 10
                )
            if self.cmd_vel_topic:
                self.create_subscription(
                    Twist, self.cmd_vel_topic, self._on_cmd_vel, 10
                )
            self.get_logger().info(
                f"speed sources: odom={self.odom_topic} cmd_vel={self.cmd_vel_topic}"
            )

    def _node_param(self, key: str):
        default = NODE_DEFAULTS[key]
        if not self.has_parameter(key):
            self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def _param(self, name: str, key: str):
        dotted = f"{name}.{key}"
        default = SENSOR_DEFAULTS[name][key]
        if not self.has_parameter(dotted):
            self.declare_parameter(dotted, default)
        return self.get_parameter(dotted).value

    def _load_sensor(self, name: str) -> SensorConfig:
        max_dist = float(self._param(name, "max_dist"))
        min_dist = float(self._param(name, "min_dist"))
        if max_dist <= min_dist:
            raise ValueError(f"{name}.max_dist must be greater than min_dist")

        inflation = float(self._param(name, "inflation"))
        if inflation < 0.0:
            raise ValueError(f"{name}.inflation must be >= 0")

        dist_density_ratio = float(self._param(name, "dist_density_ratio"))
        if dist_density_ratio < 0.0:
            raise ValueError(f"{name}.dist_density_ratio must be >= 0")

        max_scan_density = int(self._param(name, "max_scan_density"))
        if max_scan_density < 1:
            raise ValueError(f"{name}.max_scan_density must be >= 1")

        min_linear_speed = float(self._param(name, "min_linear_speed"))
        max_linear_speed = float(self._param(name, "max_linear_speed"))
        if min_linear_speed < 0.0 or max_linear_speed < 0.0:
            raise ValueError(f"{name}: linear speeds must be >= 0")
        if max_linear_speed < min_linear_speed:
            raise ValueError(f"{name}.max_linear_speed must be >= min_linear_speed")

        min_angular_speed = float(self._param(name, "min_angular_speed"))
        max_angular_speed = float(self._param(name, "max_angular_speed"))
        if min_angular_speed < 0.0 or max_angular_speed < 0.0:
            raise ValueError(f"{name}: angular speeds must be >= 0")
        if max_angular_speed < min_angular_speed:
            raise ValueError(f"{name}.max_angular_speed must be >= min_angular_speed")

        dist_to_angular_weight_ratio = float(
            self._param(name, "dist_to_angular_weight_ratio")
        )
        if not 0.0 <= dist_to_angular_weight_ratio <= 1.0:
            raise ValueError(
                f"{name}.dist_to_angular_weight_ratio must be in [0, 1]"
            )

        stopped_min_dist = float(self._param(name, "stopped_min_dist"))
        if stopped_min_dist < 0.0:
            raise ValueError(f"{name}.stopped_min_dist must be >= 0")

        enable_clear_scan = bool(self._param(name, "enable_clear_scan"))
        clear_range = float(self._param(name, "clear_range"))
        if clear_range <= min_dist:
            raise ValueError(f"{name}.clear_range must be greater than min_dist")

        return SensorConfig(
            name=name,
            range_topic=str(self._param(name, "range_topic")),
            scan_topic=str(self._param(name, "scan_topic")),
            max_dist=max_dist,
            min_dist=min_dist,
            max_scan_density=max_scan_density,
            dist_density_ratio=dist_density_ratio,
            inflation=inflation,
            enable_variable_range=bool(self._param(name, "enable_variable_range")),
            min_linear_speed=min_linear_speed,
            max_linear_speed=max_linear_speed,
            min_angular_speed=min_angular_speed,
            max_angular_speed=max_angular_speed,
            dist_to_angular_weight_ratio=dist_to_angular_weight_ratio,
            stopped_min_dist=stopped_min_dist,
            enable_clear_scan=enable_clear_scan,
            clear_range=clear_range,
        )

    def _on_cmd_vel(self, message: Twist) -> None:
        self._cmd_linear = abs(float(message.linear.x))
        self._cmd_angular = abs(float(message.angular.z))
        self._got_speed = True

    def _on_odom(self, message: Odometry) -> None:
        self._odom_linear = abs(float(message.twist.twist.linear.x))
        self._odom_angular = abs(float(message.twist.twist.angular.z))
        self._got_speed = True

    def _current_speeds(self) -> tuple[float, float]:
        return (
            max(self._cmd_linear, self._odom_linear),
            max(self._cmd_angular, self._odom_angular),
        )

    def _effective_max_dist(self, config: SensorConfig) -> float:
        if not config.enable_variable_range or not self._got_speed:
            return config.max_dist
        linear_speed, angular_speed = self._current_speeds()
        return config.effective_max_dist(linear_speed, angular_speed)

    def _make_callback(self, name: str):
        def callback(message: Range) -> None:
            self._convert(name, message)

        return callback

    def _convert(self, name: str, message: Range) -> None:
        config = self.sensors[name]
        if not self._got_range[name]:
            self._got_range[name] = True
            self.get_logger().info(
                f"{name}: first Range {message.range:.3f} m, "
                f"fov={math.degrees(message.field_of_view):.1f} deg -> "
                f"{config.scan_topic}"
            )

        range_m = float(message.range)
        mark_max = self._effective_max_dist(config)
        in_window = (
            math.isfinite(range_m)
            and config.min_dist <= range_m <= mark_max
        )

        fov = max(float(message.field_of_view), 0.0) * config.inflation
        if fov <= 0.0:
            fov = math.radians(1.0)

        if in_window:
            point_count = config.point_count(range_m)
            hit = range_m
            range_max = mark_max
        else:
            point_count = 0
            hit = mark_max
            range_max = mark_max

        # No echo in the speed window, or a scan that would have 0 points:
        # publish a long finite cone so local raytrace can clear leftover marks.
        # Inf + inf_is_valid painted a phantom wall at obstacle_max_range.
        empty_scan = (not in_window) or point_count < 1
        if empty_scan and config.enable_clear_scan:
            range_max = config.clear_scan_range_max()
            hit = config.clear_scan_hit()
            point_count = max(config.max_scan_density, 2)
        elif empty_scan:
            point_count = max(config.max_scan_density, 2)
            hit = mark_max
            range_max = mark_max
        elif config.enable_clear_scan:
            range_max = max(mark_max, config.max_dist, config.clear_range)

        point_count = max(point_count, 2)
        half = 0.5 * fov
        scan = LaserScan()
        scan.header = message.header
        scan.angle_min = -half
        scan.angle_max = half
        scan.angle_increment = fov / (point_count - 1)
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = config.min_dist
        scan.range_max = range_max
        scan.ranges = [hit] * point_count
        scan.intensities = []

        self.reliable_publishers[name].publish(scan)
        self.sensor_publishers[name].publish(scan)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RangeToScanNode | None = None
    try:
        node = RangeToScanNode()
        rclpy.spin(node)
    except ValueError as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"Failed to start range_to_scan: {error}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
