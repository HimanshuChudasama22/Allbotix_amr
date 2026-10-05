#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan


class PointCloudToLaserScan(Node):

    def __init__(self):
        super().__init__("pointcloud_to_laserscan")

        # ============================================================
        # Parameters
        # Change the default values directly here
        # ============================================================

        self.declare_parameter("input_topic", "/point_cloud")
        self.declare_parameter("output_topic", "/scan_reliable")

        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))

        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("scan_time", 0.10)

        # Enable or disable artificial-point interpolation
        self.declare_parameter("enable_interpolation", True)

        # 1 = no artificial points
        # 2 = one artificial point between adjacent points
        # 3 = two artificial points between adjacent points
        self.declare_parameter("interpolation_factor", 10)

        # Given:
        #   a = distance(p1, p2)
        #   b = distance(p2, p3)
        #
        # Do not interpolate between p2 and p3 when:
        #   b > gap_ratio_threshold * a
        self.declare_parameter("gap_ratio_threshold", 1.2)

        # Absolute distance limit in metres.
        #
        # Do not interpolate between p2 and p3 when:
        #   distance(p2, p3) > max_interpolation_gap
        #
        # Example:
        #   0.20 = maximum allowed gap of 20 cm
        self.declare_parameter("max_interpolation_gap", 0.15)

        # ============================================================
        # Read parameters
        # ============================================================

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.angle_min = float(
            self.get_parameter("angle_min").value
        )

        self.angle_max = float(
            self.get_parameter("angle_max").value
        )

        self.base_angle_increment = float(
            self.get_parameter("angle_increment").value
        )

        self.range_min = float(
            self.get_parameter("range_min").value
        )

        self.range_max = float(
            self.get_parameter("range_max").value
        )

        self.scan_time = float(
            self.get_parameter("scan_time").value
        )

        self.enable_interpolation = bool(
            self.get_parameter("enable_interpolation").value
        )

        self.interpolation_factor = int(
            self.get_parameter("interpolation_factor").value
        )

        self.gap_ratio_threshold = float(
            self.get_parameter("gap_ratio_threshold").value
        )

        self.max_interpolation_gap = float(
            self.get_parameter("max_interpolation_gap").value
        )

        # ============================================================
        # Validate parameters
        # ============================================================

        if self.angle_max <= self.angle_min:
            raise ValueError(
                "angle_max must be greater than angle_min"
            )

        if self.base_angle_increment <= 0.0:
            raise ValueError(
                "angle_increment must be greater than zero"
            )

        if self.range_max <= self.range_min:
            raise ValueError(
                "range_max must be greater than range_min"
            )

        if self.scan_time <= 0.0:
            raise ValueError(
                "scan_time must be greater than zero"
            )

        if self.interpolation_factor < 1:
            self.get_logger().warning(
                "interpolation_factor cannot be below 1. "
                "Using interpolation_factor=1."
            )
            self.interpolation_factor = 1

        if self.gap_ratio_threshold <= 0.0:
            self.get_logger().warning(
                "gap_ratio_threshold must be greater than zero. "
                "Using gap_ratio_threshold=1.0."
            )
            self.gap_ratio_threshold = 1.0

        if self.max_interpolation_gap <= 0.0:
            self.get_logger().warning(
                "max_interpolation_gap must be greater than zero. "
                "Using max_interpolation_gap=0.20 metres."
            )
            self.max_interpolation_gap = 0.20

        # Increase output angular resolution when interpolation is enabled.
        if (
            self.enable_interpolation
            and self.interpolation_factor > 1
        ):
            self.output_angle_increment = (
                self.base_angle_increment
                / self.interpolation_factor
            )
        else:
            self.output_angle_increment = (
                self.base_angle_increment
            )

        self.num_beams = int(
            math.ceil(
                (self.angle_max - self.angle_min)
                / self.output_angle_increment
            )
        )

        if self.num_beams < 1:
            raise ValueError(
                "Calculated LaserScan beam count is invalid"
            )

        # ============================================================
        # ROS interfaces
        # ============================================================

        self.subscription = self.create_subscription(
            PointCloud,
            self.input_topic,
            self.callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            "PointCloud -> LaserScan node started"
        )

        self.get_logger().info(
            f"Input topic: {self.input_topic}"
        )

        self.get_logger().info(
            f"Output topic: {self.output_topic}"
        )

        self.get_logger().info(
            f"Interpolation enabled: {self.enable_interpolation}"
        )

        self.get_logger().info(
            f"Interpolation factor: {self.interpolation_factor}"
        )

        self.get_logger().info(
            f"Gap ratio threshold: {self.gap_ratio_threshold:.2f}"
        )

        self.get_logger().info(
            "Maximum interpolation gap: "
            f"{self.max_interpolation_gap:.3f} metres"
        )

        self.get_logger().info(
            "Output angle increment: "
            f"{math.degrees(self.output_angle_increment):.4f} degrees"
        )

        self.get_logger().info(
            f"Output beam count: {self.num_beams}"
        )

    @staticmethod
    def point_distance(point_a, point_b):
        """Calculate planar Euclidean distance between two points."""

        dx = point_b["x"] - point_a["x"]
        dy = point_b["y"] - point_a["y"]

        return math.hypot(dx, dy)

    def add_sample_to_scan(
        self,
        scan,
        x,
        y,
        intensity
    ):
        """Insert one real or interpolated point into the LaserScan."""

        if not math.isfinite(x) or not math.isfinite(y):
            return

        distance = math.hypot(x, y)

        if distance < self.range_min:
            return

        if distance > self.range_max:
            return

        angle = math.atan2(y, x)

        if angle < self.angle_min:
            return

        if angle > self.angle_max:
            return

        index = int(
            (angle - self.angle_min)
            / self.output_angle_increment
        )

        if index < 0 or index >= self.num_beams:
            return

        # Keep the closest point if multiple points enter the same bin.
        if distance < scan.ranges[index]:
            scan.ranges[index] = distance
            scan.intensities[index] = float(intensity)

    def callback(self, cloud: PointCloud):

        scan = LaserScan()

        scan.header = cloud.header

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.output_angle_increment

        scan.scan_time = self.scan_time
        scan.time_increment = (
            self.scan_time / self.num_beams
        )

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        scan.ranges = [
            float("inf")
        ] * self.num_beams

        scan.intensities = [
            0.0
        ] * self.num_beams

        # ============================================================
        # Find intensity channel
        # ============================================================

        intensity_channel = None

        for channel in cloud.channels:
            if channel.name in ("intensities", "intensity"):
                intensity_channel = channel.values
                break

        # ============================================================
        # Extract valid original points
        # ============================================================

        points = []

        for index, point in enumerate(cloud.points):

            if (
                not math.isfinite(point.x)
                or not math.isfinite(point.y)
            ):
                continue

            distance = math.hypot(point.x, point.y)

            if distance < self.range_min:
                continue

            if distance > self.range_max:
                continue

            angle = math.atan2(point.y, point.x)

            if angle < self.angle_min:
                continue

            if angle > self.angle_max:
                continue

            intensity = 0.0

            if (
                intensity_channel is not None
                and index < len(intensity_channel)
            ):
                intensity = float(
                    intensity_channel[index]
                )

            points.append(
                {
                    "x": float(point.x),
                    "y": float(point.y),
                    "angle": angle,
                    "intensity": intensity,
                }
            )

        # PointCloud ordering is not guaranteed.
        points.sort(
            key=lambda stored_point: stored_point["angle"]
        )

        # ============================================================
        # Add original measured points
        # ============================================================

        for point in points:
            self.add_sample_to_scan(
                scan,
                point["x"],
                point["y"],
                point["intensity"]
            )

        # ============================================================
        # Add interpolated artificial points
        # ============================================================

        if (
            self.enable_interpolation
            and self.interpolation_factor > 1
            and len(points) >= 3
        ):
            for index in range(1, len(points) - 1):

                p1 = points[index - 1]
                p2 = points[index]
                p3 = points[index + 1]

                # a = distance(p1, p2)
                previous_gap = self.point_distance(
                    p1,
                    p2
                )

                # b = distance(p2, p3)
                candidate_gap = self.point_distance(
                    p2,
                    p3
                )

                # Duplicate p1 and p2 cannot provide a useful ratio.
                if previous_gap <= 1e-9:
                    continue

                # Check 1:
                # Reject the candidate if the gap suddenly increases
                # compared with the preceding gap.
                maximum_ratio_gap = (
                    self.gap_ratio_threshold
                    * previous_gap
                )

                if candidate_gap > maximum_ratio_gap:
                    continue

                # Check 2:
                # Reject the candidate if its absolute distance is
                # greater than the configured metre limit.
                if candidate_gap > self.max_interpolation_gap:
                    continue

                # Factor 2 adds one point at 1/2.
                # Factor 3 adds points at 1/3 and 2/3.
                for step in range(
                    1,
                    self.interpolation_factor
                ):
                    interpolation_ratio = (
                        step / self.interpolation_factor
                    )

                    interpolated_x = (
                        p2["x"]
                        + interpolation_ratio
                        * (p3["x"] - p2["x"])
                    )

                    interpolated_y = (
                        p2["y"]
                        + interpolation_ratio
                        * (p3["y"] - p2["y"])
                    )

                    interpolated_intensity = (
                        p2["intensity"]
                        + interpolation_ratio
                        * (
                            p3["intensity"]
                            - p2["intensity"]
                        )
                    )

                    self.add_sample_to_scan(
                        scan,
                        interpolated_x,
                        interpolated_y,
                        interpolated_intensity
                    )

        self.publisher.publish(scan)


def main(args=None):

    rclpy.init(args=args)

    node = PointCloudToLaserScan()

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