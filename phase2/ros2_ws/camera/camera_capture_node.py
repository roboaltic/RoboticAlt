#!/usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__("camera_capture_node")

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("image_topic", "/image_raw")

        # Вхідний потік з IMX708 краще брати 1280x720 NV12,
        # а вже потім масштабувати до 640x480 BGR для OpenCV/ROS.
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.image_topic = self.get_parameter("image_topic").value

        self.capture_width = int(self.get_parameter("capture_width").value)
        self.capture_height = int(self.get_parameter("capture_height").value)

        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(
            Image,
            self.image_topic,
            10
        )

        fps_num = int(round(self.fps))

        self.pipeline = (
            "libcamerasrc ! "
            f"video/x-raw,colorimetry=bt709,format=NV12,"
            f"width={self.capture_width},height={self.capture_height},"
            f"framerate={fps_num}/1 ! "
            "queue ! videoscale ! videoconvert ! "
            f"video/x-raw,format=BGR,width={self.width},height={self.height} ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        self.get_logger().info("📷 Відкриваю камеру через GStreamer/libcamera")
        self.get_logger().info(f"Pipeline: {self.pipeline}")

        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("❌ Не вдалося відкрити камеру через GStreamer pipeline")
        else:
            self.get_logger().info("✅ Камера відкрита через libcamera/GStreamer")
            self.get_logger().info(f"📷 Publish raw image: {self.image_topic}")
            self.get_logger().info(f"📐 Output frame: {self.width}x{self.height} @ {self.fps} FPS")

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_frame)

    def capture_frame(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn("⚠️ Камера не повернула кадр")
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"

            self.image_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"❌ Помилка публікації кадру: {e}")

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None and self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main():
    rclpy.init()

    node = CameraCaptureNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
