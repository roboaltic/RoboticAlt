#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.bridge = CvBridge()

        self.device_id = 0

        self.cap = None
        self.open_camera()

        # Счётчик кадров
        self.frame_count = 0
        self.clear_interval = 500

        # 20 FPS
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Camera node started with auto buffer clear')


    # =============================
    # Открытие камеры
    # =============================
    def open_camera(self):

        if self.cap is not None:
            self.cap.release()

        self.cap = cv2.VideoCapture(self.device_id)

        # Минимизируем внутренний буфер
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error('Camera not opened!')
        else:
            self.get_logger().info('Camera opened')


    # =============================
    # Очистка буфера
    # =============================
    def clear_buffer(self):

        self.get_logger().info('Clearing camera buffer...')

        # Пересоздаём поток
        self.open_camera()

        self.frame_count = 0


    # =============================
    # Основной цикл
    # =============================
    def timer_callback(self):

        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('Frame not received')
            return

        # Счётчик кадров
        self.frame_count += 1

        # Очистка каждые 500 кадров
        if self.frame_count >= self.clear_interval:
            self.clear_buffer()
            return


        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        self.publisher.publish(msg)


    # =============================
    # Завершение
    # =============================
    def destroy_node(self):

        if self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
