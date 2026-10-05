#!/usr/bin/env python3
"""Median + jump-confirm filter for ultrasonic Range topics."""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Range


SENSOR_NAMES = ("us_center", "us_right", "us_left")

SENSOR_DEFAULTS = {
    "us_center": {
        "range_topic": "/range_center",
        "filtered_topic": "/range_center_filtered",
        "window_size": 5,
        "spike_threshold": 0.12,
        "confirm_count": 3,
        "confirm_tolerance": 0.08,
        "ema_alpha": 0.5,
        "min_range": 0.03,
        "max_range": 3.0,
    },
    "us_right": {
        "range_topic": "/range_right",
        "filtered_topic": "/range_right_filtered",
        "window_size": 5,
        "spike_threshold": 0.12,
        "confirm_count": 3,
        "confirm_tolerance": 0.08,
        "ema_alpha": 0.5,
        "min_range": 0.03,
        "max_range": 1.5,
    },
    "us_left": {
        "range_topic": "/range_left",
        "filtered_topic": "/range_left_filtered",
        "window_size": 5,
        "spike_threshold": 0.12,
        "confirm_count": 3,
        "confirm_tolerance": 0.08,
        "ema_alpha": 0.5,
        "min_range": 0.03,
        "max_range": 1.5,
    },
}

RANGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


@dataclass
class SensorConfig:
    name: str
    range_topic: str
    filtered_topic: str
    window_size: int
    spike_threshold: float
    confirm_count: int
    confirm_tolerance: float
    ema_alpha: float
    min_range: float
    max_range: float


class SensorFilter:
    """Reject one-off spikes; smooth the readings that remain."""

    def __init__(self, config: SensorConfig) -> None:
        self.config = config
        self.window: deque[float] = deque(maxlen=config.window_size)
        self.candidates: deque[float] = deque(maxlen=config.confirm_count)
        self.output: float | None = None

    def update(self, range_m: float) -> float:
        if not math.isfinite(range_m) or range_m <= 0.0:
            range_m = self.config.max_range
        range_m = min(max(range_m, self.config.min_range), self.config.max_range)

        self.window.append(range_m)
        median = statistics.median(self.window)

        if self.output is None:
            self.output = median
            return self.output

        if abs(median - self.output) <= self.config.spike_threshold:
            self.candidates.clear()
            self.output = self._ema(median)
            return self.output

        if (
            self.candidates
            and abs(median - self.candidates[-1]) > self.config.confirm_tolerance
        ):
            self.candidates.clear()
        self.candidates.append(median)

        if len(self.candidates) >= self.config.confirm_count:
            accepted = statistics.median(self.candidates)
            self.candidates.clear()
            self.output = accepted
            return self.output

        return self.output

    def _ema(self, sample: float) -> float:
        alpha = self.config.ema_alpha
        return alpha * sample + (1.0 - alpha) * self.output


class RangeFilterNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "range_filter",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.filters: dict[str, SensorFilter] = {}
        self.filtered_publishers = {}
        self.range_subscriptions = []
        self._got_range = {name: False for name in SENSOR_NAMES}

        for name in SENSOR_NAMES:
            config = self._load_sensor(name)
            self.filters[name] = SensorFilter(config)
            self.filtered_publishers[name] = self.create_publisher(
                Range, config.filtered_topic, RANGE_QOS
            )
            callback = self._make_callback(name)
            self.range_subscriptions.append(
                self.create_subscription(
                    Range, config.range_topic, callback, RANGE_QOS
                )
            )
            self.range_subscriptions.append(
                self.create_subscription(
                    Range, config.range_topic, callback, qos_profile_sensor_data
                )
            )
            self.get_logger().info(
                f"{name}: {config.range_topic} -> {config.filtered_topic} "
                f"window={config.window_size}, "
                f"spike={config.spike_threshold:.2f} m, "
                f"confirm={config.confirm_count}"
            )

    def _param(self, name: str, key: str):
        dotted = f"{name}.{key}"
        default = SENSOR_DEFAULTS[name][key]
        if not self.has_parameter(dotted):
            self.declare_parameter(dotted, default)
        return self.get_parameter(dotted).value

    def _load_sensor(self, name: str) -> SensorConfig:
        window_size = int(self._param(name, "window_size"))
        if window_size < 1:
            raise ValueError(f"{name}.window_size must be >= 1")

        confirm_count = int(self._param(name, "confirm_count"))
        if confirm_count < 1:
            raise ValueError(f"{name}.confirm_count must be >= 1")

        ema_alpha = float(self._param(name, "ema_alpha"))
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError(f"{name}.ema_alpha must be in (0, 1]")

        min_range = float(self._param(name, "min_range"))
        max_range = float(self._param(name, "max_range"))
        if max_range <= min_range:
            raise ValueError(f"{name}.max_range must be greater than min_range")

        spike_threshold = float(self._param(name, "spike_threshold"))
        if spike_threshold <= 0.0:
            raise ValueError(f"{name}.spike_threshold must be > 0")

        confirm_tolerance = float(self._param(name, "confirm_tolerance"))
        if confirm_tolerance <= 0.0:
            raise ValueError(f"{name}.confirm_tolerance must be > 0")

        return SensorConfig(
            name=name,
            range_topic=str(self._param(name, "range_topic")),
            filtered_topic=str(self._param(name, "filtered_topic")),
            window_size=window_size,
            spike_threshold=spike_threshold,
            confirm_count=confirm_count,
            confirm_tolerance=confirm_tolerance,
            ema_alpha=ema_alpha,
            min_range=min_range,
            max_range=max_range,
        )

    def _make_callback(self, name: str):
        def callback(message: Range) -> None:
            self._filter(name, message)

        return callback

    def _filter(self, name: str, message: Range) -> None:
        if not self._got_range[name]:
            self._got_range[name] = True
            self.get_logger().info(
                f"{name}: first Range {float(message.range):.3f} m -> "
                f"{self.filters[name].config.filtered_topic}"
            )
        filtered = Range()
        filtered.header = message.header
        filtered.radiation_type = message.radiation_type
        filtered.field_of_view = message.field_of_view
        filtered.min_range = self.filters[name].config.min_range
        filtered.max_range = self.filters[name].config.max_range
        filtered.range = self.filters[name].update(float(message.range))
        self.filtered_publishers[name].publish(filtered)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RangeFilterNode | None = None
    try:
        node = RangeFilterNode()
        rclpy.spin(node)
    except (ValueError, AttributeError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"Failed to start range_filter: {error}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
