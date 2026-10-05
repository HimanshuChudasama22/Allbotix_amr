
from zlac8015d import ZLAC8015D
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class WheelControlNode(Node):

	def __init__(self):
		super().__init__('wheel_control_node')

		self.declare_parameter('port', '/dev/motorcontroller')
		self.declare_parameter('wheel_radius', 0.088)
		self.declare_parameter('wheel_separation', 0.445)
		self.declare_parameter('control_rate', 15.0)
		self.declare_parameter('cmd_vel_timeout', 0.5)
		self.declare_parameter('invert_right_wheel', True)
		self.declare_parameter('odom_frame_id', 'odom')
		self.declare_parameter('base_frame_id', 'base_link')

		self.wheel_radius = self.get_parameter('wheel_radius').value
		self.wheel_separation = self.get_parameter('wheel_separation').value
		self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
		self.right_sign = -1.0 if self.get_parameter('invert_right_wheel').value else 1.0
		self.odom_frame_id = self.get_parameter('odom_frame_id').value
		self.base_frame_id = self.get_parameter('base_frame_id').value

		port = self.get_parameter('port').value
		self.motors = ZLAC8015D.Controller(port=port)
		self.motors.disable_motor()
		self.motors.set_accel_time(100, 100)
		self.motors.set_decel_time(100, 100)
		self.motors.set_mode(3)
		self.motors.enable_motor()

		self.target_v = 0.0
		self.target_w = 0.0
		self.last_cmd_time = self.get_clock().now()

		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.last_odom_time = self.get_clock().now()

		self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_safe', self.cmd_vel_callback, 10)
		self.odom_pub = self.create_publisher(Odometry, '/diff_drive_controller/odom', 10)
		self.tf_broadcaster = TransformBroadcaster(self)

		control_rate = self.get_parameter('control_rate').value
		self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

		self.get_logger().info('wheel_control_node started on port {}'.format(port))

	def cmd_vel_callback(self, msg):
		self.target_v = msg.linear.x
		self.target_w = msg.angular.z
		self.last_cmd_time = self.get_clock().now()

	def rpm_from_linear(self, v):
		return (v / (2.0 * math.pi * self.wheel_radius)) * 60.0

	def linear_from_rpm(self, rpm):
		return (rpm / 60.0) * 2.0 * math.pi * self.wheel_radius

	def control_loop(self):
		now = self.get_clock().now()

		v = self.target_v
		w = self.target_w
		if (now - self.last_cmd_time) > Duration(seconds=self.cmd_vel_timeout):
			v = 0.0
			w = 0.0

		v_left = v - w * self.wheel_separation / 2.0
		v_right = v + w * self.wheel_separation / 2.0

		rpm_left_cmd = self.rpm_from_linear(v_left)
		rpm_right_cmd = self.right_sign * self.rpm_from_linear(v_right)

		self.motors.set_rpm(int(rpm_left_cmd), int(rpm_right_cmd))

		rpm_left_fb, rpm_right_fb = self.motors.get_rpm()
		v_left_fb = self.linear_from_rpm(rpm_left_fb)
		v_right_fb = self.right_sign * self.linear_from_rpm(rpm_right_fb)

		v_fb = (v_left_fb + v_right_fb) / 2.0
		w_fb = (v_right_fb - v_left_fb) / self.wheel_separation

		dt = (now - self.last_odom_time).nanoseconds / 1e9
		self.last_odom_time = now

		self.x += v_fb * math.cos(self.theta) * dt
		self.y += v_fb * math.sin(self.theta) * dt
		self.theta += w_fb * dt

		self.publish_odom(now, v_fb, w_fb)

		# self.get_logger().info(
		# 	'cmd v: {:+.2f} w: {:+.2f} | rpm cmd L: {:+6.1f} R: {:+6.1f} | rpm fb L: {:+6.1f} R: {:+6.1f}'.format(
		# 		v, w, rpm_left_cmd, rpm_right_cmd, rpm_left_fb, rpm_right_fb),
		# 	throttle_duration_sec=1.0)

	def publish_odom(self, stamp, v, w):
		odom = Odometry()
		odom.header.stamp = stamp.to_msg()
		odom.header.frame_id = self.odom_frame_id
		odom.child_frame_id = self.base_frame_id

		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0

		odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
		odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

		odom.twist.twist.linear.x = v
		odom.twist.twist.angular.z = w

		self.odom_pub.publish(odom)

		tf = TransformStamped()
		tf.header.stamp = odom.header.stamp
		tf.header.frame_id = self.odom_frame_id
		tf.child_frame_id = self.base_frame_id
		tf.transform.translation.x = odom.pose.pose.position.x
		tf.transform.translation.y = odom.pose.pose.position.y
		tf.transform.translation.z = odom.pose.pose.position.z
		tf.transform.rotation = odom.pose.pose.orientation

		self.tf_broadcaster.sendTransform(tf)

	def shutdown(self):
		self.motors.disable_motor()
		self.get_logger().info('Motors disabled.')


def main(args=None):
	rclpy.init(args=args)
	node = WheelControlNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.shutdown()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
