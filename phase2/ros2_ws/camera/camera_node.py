#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# --- АНАЛІТИЧНІ МОДУЛІ ---


class ObjectEdgeDetector:
    def __init__(self):
        self.params = {"min_area": 5000, "kernel": np.ones((15, 15), np.uint8)}

    def get_object_mask(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blk = cv2.GaussianBlur(gray, (7, 7), 0)
        edge = cv2.Canny(blk, 50, 150)

        mask = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, self.params["kernel"])

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obj_mask = np.zeros_like(mask)

        valid_cnts = [
            c for c in contours if cv2.contourArea(c) > self.params["min_area"]
        ]

        cv2.drawContours(obj_mask, valid_cnts, -1, 255, -1)

        return obj_mask, valid_cnts


class CrackDetector:
    def __init__(self):
        self.k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

    def detect(self, frame, obj_mask):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, self.k)

        _, thresh = cv2.threshold(hat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        refined_mask = cv2.bitwise_and(thresh, obj_mask)

        contours, _ = cv2.findContours(
            refined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        cracks = []

        for c in contours:
            area = cv2.contourArea(c)

            if 100 < area < 5000:
                rect = cv2.minAreaRect(c)
                w, h = rect[1]

                if max(w, h) / (min(w, h) + 0.1) > 2.0:
                    cracks.append(c)

        return cracks


# --- ОСНОВНА ROS2 НОДА ---


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        # --- ПАРАМЕТРИ КАМЕРИ ---
        self.declare_parameter("device", "/dev/video0")

        device_path = self.get_parameter("device").value

        self.cap = cv2.VideoCapture(device_path)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Не вдалося відкрити камеру: {device_path}")
        else:
            self.get_logger().info(f"✅ Камера відкрита: {device_path}")

        # Поріг центральної рамки
        self.log_threshold = 150

        # Аналітика
        self.obj_dev = ObjectEdgeDetector()
        self.crack_dev = CrackDetector()

        # ArUco
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        # ROS publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.buzzer_pub = self.create_publisher(Int32, "/buzzer_signal", 10)

        # Ось цей publisher потрібен для ROS streamer-а
        self.image_pub = self.create_publisher(Image, "/image_raw", 10)

        self.bridge = CvBridge()

        self.passed_markers = []
        self.mission_stage = 0

        # 15 FPS
        self.timer = self.create_timer(1.0 / 15.0, self.process_frame)

        self.get_logger().info(
            "✅ ArUco camera node запущена. Кадри йдуть у /image_raw"
        )

    def process_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("⚠️ Камера не повернула кадр")
            return

        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2

        # --- АНАЛІТИКА: об'єкти і тріщини ---
        obj_mask, obj_contours = self.obj_dev.get_object_mask(frame)
        cracks = self.crack_dev.detect(frame, obj_mask)

        # --- ВІЗУАЛІЗАЦІЯ ЦЕНТРУ ---
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 1)

        cv2.rectangle(
            frame,
            (cx - self.log_threshold, cy - self.log_threshold),
            (cx + self.log_threshold, cy + self.log_threshold),
            (200, 200, 200),
            1,
        )

        # --- ДЕТЕКЦІЯ ARUCO ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        cmd = Twist()

        if ids is not None:
            current_id = int(ids[0][0])

            if current_id not in self.passed_markers:
                c = corners[0][0]

                m_x = int(c[:, 0].mean())
                m_y = int(c[:, 1].mean())

                marker_width = abs(c[0][0] - c[1][0])

                offset_x = m_x - cx
                offset_y = cy - m_y

                aruco.drawDetectedMarkers(frame, corners, ids)

                # Підпис на кадрі
                cv2.putText(
                    frame,
                    f"ID: {current_id} W: {int(marker_width)}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    frame,
                    f"offset_x: {offset_x}",
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                # --- ЦІЛЬ ДОСЯГНУТА ---
                if marker_width > 230:
                    self.get_logger().warn(f"🎯 ЦІЛЬ ДОСЯГНУТА: ID {current_id}")

                    self.passed_markers.append(current_id)
                    self.mission_stage += 1

                    self.buzzer_pub.publish(Int32(data=1))

                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                    self.get_logger().info(
                        f"🔎 Перехід до пошуку наступного маркера. Етап: {self.mission_stage}"
                    )

                else:
                    if (
                        abs(offset_x) < self.log_threshold
                        and abs(offset_y) < self.log_threshold
                    ):
                        self.get_logger().info(
                            f"📡 Супровід: ID {current_id} | Ширина: {int(marker_width)}"
                        )

                    # --- КЕРУВАННЯ РУХОМ ДО МАРКЕРА ---
                    if abs(offset_x) > 30:
                        cmd.angular.z = -0.004 * float(offset_x)
                        cmd.linear.x = 0.05
                    else:
                        cmd.linear.x = 0.15
                        cmd.angular.z = 0.0

            else:
                # Маркер вже пройдений — ігноруємо
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        else:
            # Якщо маркера немає — стоїмо
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # --- МАЛЮЄМО КОНТУРИ ОБ'ЄКТІВ І ТРІЩИН ---
        cv2.drawContours(frame, obj_contours, -1, (255, 0, 0), 2)
        cv2.drawContours(frame, cracks, -1, (0, 0, 255), 3)

        # --- ПУБЛІКУЄМО КОМАНДУ НА МОТОРИ ---
        self.cmd_pub.publish(cmd)

        # --- ПУБЛІКУЄМО ОБРОБЛЕНИЙ КАДР У ROS STREAMER ---
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera"

            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"❌ Помилка публікації image_raw: {e}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()

    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
