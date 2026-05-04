#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node

from crack_detector import CrackDetector, ObjectEdgeDetector


class RobotVisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Инициализация модулей
        self.obj_dev = ObjectEdgeDetector()
        self.crack_dev = CrackDetector()
        self.dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        self.create_timer(0.06, self.step)  # ~15 FPS
        self.get_logger().info("Vision System Online")

    def step(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # --- СЛОЙ АНАЛИТИКИ ---
        obj_mask, obj_contours = self.obj_dev.get_object_mask(frame)
        cracks = self.crack_dev.detect(frame, obj_mask)

        # --- СЛОЙ ARUCO ---
        corners, ids, _ = aruco.detectMarkers(frame, self.dict)

        # --- ВИЗУАЛИЗАЦИЯ (Минималистично) ---
        # Рисуем объекты (синим) и трещины (красным)
        cv2.drawContours(frame, obj_contours, -1, (255, 0, 0), 2)
        cv2.drawContours(frame, cracks, -1, (0, 0, 255), 3)

        cmd = Twist()
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            # Логика движения по X-центру первого маркера
            err = corners[0][0][:, 0].mean() - (frame.shape[1] / 2)
            cmd.angular.z = -0.005 * err
            cmd.linear.x = 0.1

            if len(cracks) > 0:
                self.get_logger().warn(f"DEFECTS FOUND: {len(cracks)}")

        self.pub.publish(cmd)
        cv2.imshow("Stream", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    rclpy.spin(RobotVisionNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
