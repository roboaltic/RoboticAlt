import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Параметры
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('fps', 15)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        device = self.get_parameter('device').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Не удалось открыть камеру на {device}")
            raise RuntimeError('Camera open failed')

        # Publisher с QoS buffer
        from rclpy.qos import QoSProfile
        qos_profile = QoSProfile(depth=10)
        self.pub_raw = self.create_publisher(Image, '/image_raw', qos_profile)
        self.pub_compressed = self.create_publisher(Image, '/image_raw/compressed', qos_profile)

        self.bridge = CvBridge()
        self.frame_count = 0

        # Таймер для регулярного захвата
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("✅ CameraNode initialized")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("❌ Не удалось получить кадр")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub_raw.publish(msg)

        # Сжатое изображение
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        msg_compressed = self.bridge.cv2_to_imgmsg(buffer.tobytes(), encoding='jpeg')
        self.pub_compressed.publish(msg_compressed)

        self.frame_count += 1
        if self.frame_count % 15 == 0:  # лог раз в 15 кадров
            self.get_logger().info(f"Publishing frame {self.frame_count}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
