#!/usr/bin/env python3
"""
Camera Publisher Node
Publishes monocular camera feed as sensor_msgs/Image and CompressedImage.
"""

import cv2
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from builtin_interfaces.msg import Time


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('device', '/dev/amr_camera')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('publish_compressed', True)

        self.device = self.get_parameter('device').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.publish_compressed = self.get_parameter('publish_compressed').value

        # ── Publishers ──────────────────────────────────────────
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 5)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 5)

        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage, '/camera/image_raw/compressed', 5
            )

        # ── Camera setup ────────────────────────────────────────
        self.cap = None
        self.running = True
        self.connect_camera()

        # ── Capture thread ──────────────────────────────────────
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info(
            f'Camera publisher started: {self.device} @ {self.width}x{self.height} {self.fps}fps'
        )

    def connect_camera(self):
        """Open camera device with retries."""
        max_retries = 10
        for attempt in range(max_retries):
            # Try symlink first, fall back to device path
            for dev in [self.device, '/dev/video0', '/dev/video1', 0]:
                try:
                    self.cap = cv2.VideoCapture(dev)
                    if self.cap.isOpened():
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                        # Read actual values
                        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        self.get_logger().info(
                            f'Camera opened: {dev} ({actual_w}x{actual_h})'
                        )
                        return
                    else:
                        self.cap.release()
                except Exception:
                    pass

            self.get_logger().warn(
                f'Camera connect attempt {attempt+1}/{max_retries} failed, retrying...'
            )
            time.sleep(2.0)

        self.get_logger().error('Could not open camera!')

    def capture_loop(self):
        """Background thread: capture frames and publish."""
        period = 1.0 / self.fps

        while self.running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(1.0)
                continue

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Frame capture failed', throttle_duration_sec=5.0)
                time.sleep(0.1)
                continue

            now = self.get_clock().now()
            stamp = now.to_msg()

            # Publish raw Image
            img_msg = self.cv2_to_imgmsg(frame, stamp)
            self.image_pub.publish(img_msg)

            # Publish CameraInfo (uncalibrated placeholder)
            info_msg = self.make_camera_info(stamp, frame.shape[1], frame.shape[0])
            self.info_pub.publish(info_msg)

            # Publish compressed
            if self.publish_compressed:
                compressed_msg = self.cv2_to_compressed(frame, stamp)
                self.compressed_pub.publish(compressed_msg)

            # Rate limit
            time.sleep(period)

    def cv2_to_imgmsg(self, frame, stamp) -> Image:
        """Convert OpenCV BGR frame to sensor_msgs/Image."""
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    def cv2_to_compressed(self, frame, stamp) -> CompressedImage:
        """Convert OpenCV BGR frame to CompressedImage (JPEG)."""
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        msg.data = buf.tobytes()
        return msg

    def make_camera_info(self, stamp, width: int, height: int) -> CameraInfo:
        """Create a basic CameraInfo (uncalibrated)."""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = width
        msg.height = height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # Default identity-like intrinsics (user should calibrate)
        fx = float(width)
        fy = float(width)
        cx = float(width) / 2.0
        cy = float(height) / 2.0
        msg.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return msg

    def destroy_node(self):
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
