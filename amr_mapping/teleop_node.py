#!/usr/bin/env python3
"""Convert game-controller joystick axes into differential-drive cmd_vel.

Left stick  Y → linear.x  (forward / backward)
Right stick X → angular.z (left / right turn)
L2 trigger    → scales speed from base → max
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Twist


class EvofoxTeleop(Node):

    def __init__(self):
        super().__init__("teleop_node")

        # ------------------------
        # Parameters
        # ------------------------
        self.base_linear = 0.2
        self.base_angular = 0.5

        self.max_linear = 1.0
        self.max_angular = 1.5

        self.deadzone = 0.05

        # joystick mapping (SDL game controller)
        self.LEFT_Y = 1
        self.RIGHT_X = 2
        self.L2 = 5

        self.publish_rate = 50.0
        self.frame_id = "base_link"

        # latest joystick state, updated by joy_callback and
        # published on a fixed-rate timer independent of /joy's rate
        self.left_y = 0.0
        self.right_x = 0.0
        self.l2 = -1.0  # rest position

        self.sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10,
        )

        self.diff_drive_pub = self.create_publisher(
            TwistStamped,
            "diff_drive_controller/cmd_vel",
            10,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_cmd_vel,
        )

        self.get_logger().info("teleop node started")

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        if len(msg.axes) > self.LEFT_Y:
            self.left_y = msg.axes[self.LEFT_Y]
        if len(msg.axes) > self.RIGHT_X:
            self.right_x = msg.axes[self.RIGHT_X]
        if len(msg.axes) > self.L2:
            self.l2 = msg.axes[self.L2]

    def publish_cmd_vel(self):
        left_y = self.apply_deadzone(self.left_y)
        right_x = self.apply_deadzone(self.right_x)

        # convert trigger: -1 rest → +1 pressed
        throttle = (1.0 - self.l2) / 2.0

        current_linear_limit = (
            self.base_linear
            + throttle * (self.max_linear - self.base_linear)
        )
        current_angular_limit = (
            self.base_angular
            + throttle * (self.max_angular - self.base_angular)
        )

        linear_vel = left_y * current_linear_limit
        angular_vel = right_x * current_angular_limit

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.frame_id
        cmd.twist.linear.x = linear_vel
        cmd.twist.angular.z = angular_vel

        self.diff_drive_pub.publish(cmd)

        # twt = Twist()
        # twt.linear.x = linear_vel
        # twt.angular.z = angular_vel

        # self.cmd_vel_pub.publish(twt)





def main(args=None):
    rclpy.init(args=args)
    node = EvofoxTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


