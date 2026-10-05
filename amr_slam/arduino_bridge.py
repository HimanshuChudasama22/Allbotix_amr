#!/usr/bin/env python3
"""
Arduino Bridge Node for AMR
- Serial communication with Arduino Mega (RMCS-2305 driver)
- Subscribes to /cmd_vel and sends C,v,w commands to Arduino
- Reads E,left_ticks,right_ticks from Arduino
- Computes differential-drive odometry
- Publishes nav_msgs/Odometry on /odom
- Broadcasts odom -> base_link TF
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import serial


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a Quaternion message from a yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('ticks_per_rev', 2350)
        self.declare_parameter('wheel_diameter_m', 0.0735)
        self.declare_parameter('wheelbase_m', 0.330)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.wheel_diameter = self.get_parameter('wheel_diameter_m').value
        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Derived constants
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.dist_per_tick = self.wheel_circumference / self.ticks_per_rev

        self.get_logger().info(
            f'Robot params: ticks/rev={self.ticks_per_rev}, '
            f'wheel_diam={self.wheel_diameter:.4f}m, '
            f'wheelbase={self.wheelbase:.3f}m, '
            f'dist/tick={self.dist_per_tick:.6f}m'
        )

        # ── Odometry state ──────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vtheta = 0.0

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_odom_time = None

        # ── ROS interfaces ──────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        # ── Serial connection ───────────────────────────────────
        self.ser = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        # ── Serial read thread ──────────────────────────────────
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

        # ── Watchdog: resend zero if no cmd_vel for a while ─────
        self.last_cmd_time = time.time()
        self.create_timer(0.5, self.watchdog_callback)

        self.get_logger().info('Arduino bridge node started')

    def connect_serial(self):
        """Open serial port to Arduino."""
        max_retries = 10
        for attempt in range(max_retries):
            try:
                self.ser = serial.Serial(
                    self.serial_port,
                    self.baud_rate,
                    timeout=0.1
                )
                time.sleep(2.0)  # wait for Arduino reset
                # Flush startup messages
                while self.ser.in_waiting:
                    self.ser.readline()
                self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
                return
            except serial.SerialException as e:
                self.get_logger().warn(
                    f'Serial connect attempt {attempt+1}/{max_retries} failed: {e}'
                )
                time.sleep(1.0)
        self.get_logger().error('Could not connect to Arduino!')

    def cmd_vel_callback(self, msg: Twist):
        """Send cmd_vel to Arduino as C,v,w."""
        v = msg.linear.x   # m/s
        w = msg.angular.z   # rad/s
        cmd = f'C,{v:.4f},{w:.4f}\n'
        self.serial_write(cmd)
        self.last_cmd_time = time.time()

    def watchdog_callback(self):
        """If no cmd_vel for 1 second, send stop."""
        if time.time() - self.last_cmd_time > 1.0:
            self.serial_write('C,0.0000,0.0000\n')

    def serial_write(self, data: str):
        """Thread-safe serial write."""
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(data.encode('ascii'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')

    def serial_read_loop(self):
        """Background thread: read lines from Arduino."""
        while self.running:
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                with self.serial_lock:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    else:
                        line = None
                if line:
                    self.process_serial_line(line)
                else:
                    time.sleep(0.005)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.5)

    def process_serial_line(self, line: str):
        """Parse Arduino messages."""
        if line.startswith('E,'):
            parts = line.split(',')
            if len(parts) == 3:
                try:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])
                    self.update_odometry(left_ticks, right_ticks)
                except ValueError:
                    pass
        elif line == 'AMR_READY_CMDVEL':
            self.get_logger().info('Arduino reports READY')
        elif line == 'PONG':
            self.get_logger().debug('Arduino PONG received')

    def update_odometry(self, left_ticks: int, right_ticks: int):
        """Compute odometry from encoder ticks and publish."""
        now = self.get_clock().now()

        if self.prev_left_ticks is None:
            # First reading — initialize and return
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_odom_time = now
            return

        # Tick deltas
        dl = left_ticks - self.prev_left_ticks
        dr = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Time delta
        dt_ns = (now - self.last_odom_time).nanoseconds
        dt = dt_ns / 1e9
        self.last_odom_time = now

        if dt < 1e-6:
            return

        # Distance traveled by each wheel
        dist_left = dl * self.dist_per_tick
        dist_right = dr * self.dist_per_tick

        # Differential drive kinematics
        delta_s = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheelbase

        # Update pose (mid-point integration)
        # Calculate position using mid-point angle without modifying theta yet
        theta_mid = self.theta + delta_theta / 2.0
        self.x += delta_s * math.cos(theta_mid)
        self.y += delta_s * math.sin(theta_mid)
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocities
        self.vx = delta_s / dt
        self.vtheta = delta_theta / dt

        # ── Publish Odometry message ────────────────────────────
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self.theta)

        # Covariance (rough estimates — tune for your robot)
        odom_msg.pose.covariance[0]  = 0.01   # x
        odom_msg.pose.covariance[7]  = 0.01   # y
        odom_msg.pose.covariance[35] = 0.03   # yaw

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.z = self.vtheta

        odom_msg.twist.covariance[0]  = 0.01
        odom_msg.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom_msg)

        # ── Broadcast TF: odom -> base_link ─────────────────────
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            q = quaternion_from_yaw(self.theta)
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.serial_write('STOP\n')
            time.sleep(0.1)
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
