import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import time


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # ===== Publishers =====
        self.pub_raw = self.create_publisher(Image, '/image_raw', 10)
        self.pub_compressed = self.create_publisher(
            CompressedImage,
            '/image_raw/compressed',
            10
        )

        self.bridge = CvBridge()

        # ===== Camera config (Logitech C270) =====
        self.device = '/dev/video0'   # Твой USB-порт
        self.width = 640
        self.height = 480
        self.fps = 15.0

        # Открываем камеру через V4L2
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().fatal('❌ Не удалось открыть камеру на /dev/video0')
            raise RuntimeError('Camera open failed')

        # Настройка MJPEG → минимальная нагрузка на CPU
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.period = 1.0 / self.fps
        self.last_time = time.time()

        # Таймер ROS для публикации кадров
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info(
            f'✅ Camera started: {self.width}x{self.height} @ {self.fps} FPS (MJPEG)'
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('⚠️ Кадр не получен')
            return

        now = self.get_clock().now().to_msg()

        # ===== RAW image =====
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.stamp = now
        raw_msg.header.frame_id = 'camera_frame'
        self.pub_raw.publish(raw_msg)

        # ===== COMPRESSED image (JPEG) =====
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = now
        compressed_msg.header.frame_id = 'camera_frame'
        compressed_msg.format = 'jpeg'

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        success, encoded = cv2.imencode('.jpg', frame, encode_param)

        if success:
            compressed_msg.data = encoded.tobytes()
            self.pub_compressed.publish(compressed_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
