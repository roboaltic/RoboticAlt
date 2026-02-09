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

        # Открываем камеру (обычно 0 или /dev/video0)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error('Camera not opened!')
            return

        # Частота кадров
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Camera node started')


    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('Frame not received')
            return

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
