#!/usr/bin/env python3
"""ROS 2 node for the A21 Modbus RTU ultrasonic range sensor."""

from __future__ import annotations

import math
import struct
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField, Range
from std_msgs.msg import Header
import serial


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def make_read_request(slave: int, register: int) -> bytes:
    frame = bytes(
        [slave, 0x03, register >> 8, register & 0xFF, 0x00, 0x01]
    )
    return frame + modbus_crc(frame).to_bytes(2, "little")


def make_pointcloud(header: Header, points: list[tuple[float, float, float]]) -> PointCloud2:
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * max(len(points), 1)
    message.is_dense = True
    message.data = b"".join(struct.pack("<fff", *point) for point in points)
    return message


class A21RangeNode(Node):
    def __init__(self) -> None:
        super().__init__("a21_ultrasound")

        self.declare_parameter("port", "/dev/ultrasound_1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("slave", 1)
        self.declare_parameter("topic", "range")
        self.declare_parameter("cloud_topic", "")
        self.declare_parameter("clear_cloud_topic", "")
        self.declare_parameter("publish_cloud", False)
        self.declare_parameter("frame_id", "ultrasound_link")
        self.declare_parameter("min_range", 0.03)
        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("field_of_view", math.radians(20.0))
        self.declare_parameter("clear_ratio", 0.92)
        # Hits closer than this are treated as floor/crosstalk, not obstacles.
        self.declare_parameter("mark_min_range", 0.12)
        self.declare_parameter("hit_confirm_count", 2)
        self.declare_parameter("hit_confirm_window", 5)
        self.declare_parameter("hit_confirm_tolerance", 0.20)
        # Lateral width of the lethal bar written into the costmap, meters.
        self.declare_parameter("mark_width", 0.12)
        self.declare_parameter("retries", 2)
        self.declare_parameter("retry_delay", 0.01)
        self.declare_parameter("timeout", 0.25)

        port_name = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self.slave = self.get_parameter("slave").value
        topic = self.get_parameter("topic").value
        cloud_topic = self.get_parameter("cloud_topic").value
        clear_cloud_topic = self.get_parameter("clear_cloud_topic").value
        self.publish_cloud = bool(self.get_parameter("publish_cloud").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.field_of_view = float(self.get_parameter("field_of_view").value)
        self.clear_ratio = float(self.get_parameter("clear_ratio").value)
        self.mark_min_range = float(self.get_parameter("mark_min_range").value)
        self.hit_confirm_count = int(self.get_parameter("hit_confirm_count").value)
        self.hit_confirm_window = int(self.get_parameter("hit_confirm_window").value)
        self.hit_confirm_tolerance = float(
            self.get_parameter("hit_confirm_tolerance").value
        )
        self.mark_width = float(self.get_parameter("mark_width").value)
        self.retries = self.get_parameter("retries").value
        self.retry_delay = self.get_parameter("retry_delay").value
        timeout = self.get_parameter("timeout").value

        if not 1 <= self.slave <= 254:
            raise ValueError("slave must be between 1 and 254")
        if not 0.5 <= self.clear_ratio <= 1.0:
            raise ValueError("clear_ratio must be between 0.5 and 1.0")
        if self.max_range <= self.min_range:
            raise ValueError("max_range must be greater than min_range")

        if not cloud_topic:
            cloud_topic = f"{topic}/cloud"
        if not clear_cloud_topic:
            clear_cloud_topic = f"{topic}/clear"

        self.recent_hits: deque[float] = deque(maxlen=self.hit_confirm_window)

        self.request = make_read_request(self.slave, 0x0101)
        self.serial_port = serial.Serial(
            port=port_name,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )
        range_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(Range, topic, range_qos)
        self.cloud_publisher = None
        self.clear_cloud_publisher = None
        if self.publish_cloud:
            self.cloud_publisher = self.create_publisher(
                PointCloud2, cloud_topic, qos_profile_sensor_data
            )
            self.clear_cloud_publisher = self.create_publisher(
                PointCloud2, clear_cloud_topic, qos_profile_sensor_data
            )
        self.consecutive_failures = 0

        self.timer = self.create_timer(0.001, self.poll_and_publish)
        if self.publish_cloud:
            self.get_logger().info(
                f"Publishing {topic}, {cloud_topic}, {clear_cloud_topic} from "
                f"{port_name} at {baud} baud"
            )
        else:
            self.get_logger().info(
                f"Publishing {topic} from {port_name} at {baud} baud "
                "(point clouds disabled)"
            )

    def read_distance_mm(self) -> int:
        self.serial_port.reset_input_buffer()
        self.serial_port.write(self.request)
        self.serial_port.flush()
        response = self.serial_port.read(7)

        if len(response) != 7:
            raise TimeoutError(f"received {len(response)} of 7 bytes")
        expected_crc = int.from_bytes(response[-2:], "little")
        if modbus_crc(response[:-2]) != expected_crc:
            raise ValueError("bad CRC")
        if response[0] != self.slave:
            raise ValueError(f"unexpected slave 0x{response[0]:02X}")
        if response[1] & 0x80:
            raise ValueError(f"Modbus exception 0x{response[2]:02X}")
        if response[1:3] != b"\x03\x02":
            raise ValueError(f"unexpected response {response.hex(' ')}")
        return int.from_bytes(response[3:5], "big")

    def classify_range(self, range_m: float) -> tuple[float, bool]:
        """Return (range_m, is_clear). Clear means no obstacle to mark."""
        if range_m <= 0.0:
            raise ValueError(f"non-positive range {range_m}")

        if range_m >= self.max_range * self.clear_ratio or range_m > self.max_range:
            return self.max_range, True

        # Floor bounce / crosstalk at these mounts sits well below a real
        # bumper-height obstacle. Treat it as empty so the costmap does not
        # grow a phantom wall in front of each transducer.
        if range_m < max(self.min_range, self.mark_min_range):
            return self.max_range, True

        return range_m, False

    def confirmed_hit(self, range_m: float) -> bool:
        # Range shrinks while driving at a static object, so do not require
        # repeated copies of the same distance. Consecutive detections are
        # enough; a sudden jump is treated as crosstalk and the window resets.
        if (
            self.recent_hits
            and abs(range_m - self.recent_hits[-1]) > self.hit_confirm_tolerance
        ):
            self.recent_hits.clear()
        self.recent_hits.append(range_m)
        return len(self.recent_hits) >= self.hit_confirm_count

    def hit_bar(self, range_m: float) -> list[tuple[float, float, float]]:
        """A short lateral bar so the mark is more than one costmap cell."""
        half = max(self.mark_width, 0.0) / 2.0
        if half <= 0.0:
            return [(range_m, 0.0, 0.0)]
        return [
            (range_m, -half, 0.0),
            (range_m, 0.0, 0.0),
            (range_m, half, 0.0),
        ]

    def poll_and_publish(self) -> None:
        last_error: Exception | None = None
        for attempt in range(self.retries + 1):
            try:
                distance_mm = self.read_distance_mm()
                last_error = None
                break
            except (TimeoutError, ValueError, serial.SerialException) as error:
                last_error = error
                if attempt < self.retries:
                    time.sleep(self.retry_delay)

        if last_error is not None:
            self.consecutive_failures += 1
            if self.consecutive_failures == 1 or self.consecutive_failures % 10 == 0:
                self.get_logger().warning(
                    f"Read failed after {self.retries + 1} attempts: {last_error}"
                )
            return

        self.consecutive_failures = 0
        if distance_mm == 0xFFFF:
            return

        try:
            range_m, is_clear = self.classify_range(distance_mm / 1000.0)
        except ValueError:
            return

        if is_clear:
            self.recent_hits.clear()
            mark_hit = False
            publish_clear = True
        else:
            mark_hit = self.confirmed_hit(range_m)
            # Do not raytrace-clear while a hit is still unconfirmed. An empty
            # mark plus a max-range clear would wipe a real world-frame obstacle
            # just because the robot is approaching and range is changing.
            publish_clear = mark_hit

        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id

        message = Range()
        message.header = header
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = self.field_of_view
        message.min_range = self.min_range
        message.max_range = self.max_range
        message.range = range_m
        self.publisher.publish(message)

        if not self.publish_cloud:
            return

        if publish_clear:
            self.clear_cloud_publisher.publish(
                make_pointcloud(header, [(self.max_range, 0.0, 0.0)])
            )
        hit_points = self.hit_bar(range_m) if mark_hit else []
        self.cloud_publisher.publish(make_pointcloud(header, hit_points))

    def destroy_node(self) -> bool:
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: A21RangeNode | None = None
    try:
        node = A21RangeNode()
        rclpy.spin(node)
    except (serial.SerialException, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"Failed to start A21 ultrasound node: {error}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
