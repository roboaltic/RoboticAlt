import rclpy                         # Основна бібліотека ROS2 для Python
from rclpy.node import Node          # Базовий клас для створення ноди
from sensor_msgs.msg import Image   # Тип повідомлення для зображень
from cv_bridge import CvBridge       # Міст між OpenCV та ROS Image
import cv2                           # Бібліотека OpenCV для роботи з камерою


class CameraNode(Node):

    def __init__(self):
        # Ініціалізація ноди з іменем "camera_node"
        super().__init__('camera_node')

        # === Оголошення параметрів ноди ===

        # Шлях до пристрою камери
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('fps', 15)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        # === Зчитування параметрів ===

        device = self.get_parameter('device').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        # === Підключення до камери через OpenCV ===

        # Відкриваємо відеопотік
        self.cap = cv2.VideoCapture(device)

        # Встановлюємо роздільну здатність
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Встановлюємо FPS
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Перевірка, чи камера відкрилась
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Не удалось открыть камеру на {device}")
            raise RuntimeError('Camera open failed')

        # === Налаштування Publisher з QoS ===

        from rclpy.qos import QoSProfile

        # Буфер на 10 повідомлень
        qos_profile = QoSProfile(depth=10)

        # Publisher для сирого зображення
        self.pub_raw = self.create_publisher(
            Image,
            '/image_raw',
            qos_profile
        )

        # Publisher для стисненого зображення
        self.pub_compressed = self.create_publisher(
            Image,
            '/image_raw/compressed',
            qos_profile
        )

        # Ініціалізація CV Bridge
        self.bridge = CvBridge()

        # Лічильник кадрів
        self.frame_count = 0

        # === Таймер для регулярного зчитування кадрів ===

        # Період виклику callback
        timer_period = 1.0 / fps

        # Створення таймера
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        self.get_logger().info("✅ CameraNode initialized")

    # Функція, яка викликається таймером
    def timer_callback(self):

        # Зчитуємо кадр з камери
        ret, frame = self.cap.read()

        # Якщо не вдалося зчитати кадр
        if not ret:
            self.get_logger().warn("❌ Не удалось получить кадр")
            return

        # === Публікація сирого зображення ===

        # Конвертація OpenCV → ROS Image
        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8"
        )

        # Публікація в топік /image_raw
        self.pub_raw.publish(msg)

        # === Створення JPEG-зображення ===

        # Кодуємо кадр у JPEG (якість 80%)
        _, buffer = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        )

        # Конвертація байтів у ROS-повідомлення
        msg_compressed = self.bridge.cv2_to_imgmsg(
            buffer.tobytes(),
            encoding='jpeg'
        )

        # Публікація стисненого кадру
        self.pub_compressed.publish(msg_compressed)

        # === Логування ===

        self.frame_count += 1

        # Виводимо повідомлення кожні 15 кадрів
        if self.frame_count % 15 == 0:
            self.get_logger().info(
                f"Publishing frame {self.frame_count}"
            )

    # Коректне завершення роботи ноди
    def destroy_node(self):

        # Звільняємо камеру
        self.cap.release()

        # Викликаємо стандартний destroy
        super().destroy_node()


# Головна функція
def main():

    # Ініціалізація ROS2
    rclpy.init()

    # Створення ноди
    node = CameraNode()

    try:
        # Запуск обробки callback-функцій
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Завершення через Ctrl+C
        pass

    finally:
        # Коректне завершення
        node.destroy_node()
        rclpy.shutdown()
