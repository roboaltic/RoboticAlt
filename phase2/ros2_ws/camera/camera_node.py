#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import rclpy

# from camera.crack_detector import CrackDetector, ObjectEdgeDetector
from camera.crack_detector import CrackDetector, ObjectEdgeDetector
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DEV = "/dev/video2"  # в цю змінну вписуємо шлях до камери


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("device", DEV)
        self.declare_parameter("fps", 15)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)

        device = self.get_parameter("device").get_parameter_value().string_value
        fps = self.get_parameter("fps").get_parameter_value().integer_value
        width = self.get_parameter("width").get_parameter_value().integer_value
        height = self.get_parameter("height").get_parameter_value().integer_value

        self.aruco_dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_parameters = aruco.DetectorParameters_create()
        self.aruco_parameters.adaptiveThreshConstant = 7
        self.aruco_parameters.minMarkerPerimeterRate = 0.03

        # Ініціалізація наших модулів комп'ютерного зору
        self.crack_detector = CrackDetector()
        # Шукаємо об'єкти від 2000 до 80000 пікселів
        self.edge_detector = ObjectEdgeDetector(min_area=2000, max_area=80000)

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Не вдалося відкрити камеру на {device}")
            raise RuntimeError("Camera open failed")

        from rclpy.qos import QoSProfile

        self.pub_raw = self.create_publisher(Image, "/image_raw", QoSProfile(depth=10))
        self.bridge = CvBridge()

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(
            "✅ CameraNode запущена (ArUco + Тріщини в ROI + Грані об'єктів)"
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        height, width, _ = frame.shape
        center_f_x = width // 2
        center_f_y = height // 2

        # ==========================================
        # 1. ПОШУК ГРАНЕЙ ОБ'ЄКТІВ (НА ВСЬОМУ КАДРІ)
        frame = self.edge_detector.detect(frame)
        # ==========================================

        # === НАЛАШТУВАННЯ ЗОНИ СКАНУВАННЯ ARUCO ===
        roi_w = width // 2
        roi_h = height // 2

        x1 = center_f_x - (roi_w // 2)
        y1 = center_f_y - (roi_h // 2)
        x2 = center_f_x + (roi_w // 2)
        y2 = center_f_y + (roi_h // 2)

        # Малюємо оранжеву рамку зони пошуку ArUco та центр
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
        cv2.line(
            frame,
            (center_f_x - 10, center_f_y),
            (center_f_x + 10, center_f_y),
            (0, 0, 255),
            2,
        )
        cv2.line(
            frame,
            (center_f_x, center_f_y - 10),
            (center_f_x, center_f_y + 10),
            (0, 0, 255),
            2,
        )

        # Скануємо ТІЛЬКИ вирізану зону на наявність ArUco
        roi_frame = frame[y1:y2, x1:x2]
        gray_aruco = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(
            gray_aruco, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        if ids is not None:
            for i in range(len(corners)):
                corners[i][0][:, 0] += x1
                corners[i][0][:, 1] += y1

            aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                c = corners[i][0]
                m_x = int(c[:, 0].mean())
                m_y = int(c[:, 1].mean())

                cv2.circle(frame, (m_x, m_y), 5, (0, 255, 0), -1)

                offset_x = m_x - center_f_x
                offset_y = center_f_y - m_y

                text = f"ID: {ids[i][0]} DX: {offset_x} DY: {offset_y}"
                cv2.putText(
                    frame,
                    text,
                    (m_x + 10, m_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2,
                )

                tolerance = 25
                if abs(offset_x) <= tolerance and abs(offset_y) <= tolerance:
                    self.get_logger().info(
                        f"🎯 ЦІЛЬ ЗАХОПЛЕНО! Marker {ids[i][0]} в центрі (X: {offset_x}, Y: {offset_y})",
                        throttle_duration_sec=1.0,
                    )

        # ==========================================
        # 2. ПОШУК ТРІЩИН У СТРОГО ВИДІЛЕНІЙ ЗОНІ (ROI)
        scan_w = 400
        scan_h = 300

        c_x1 = center_f_x - (scan_w // 2)
        c_y1 = center_f_y - (scan_h // 2)
        c_x2 = center_f_x + (scan_w // 2)
        c_y2 = center_f_y + (scan_h // 2)

        cv2.rectangle(frame, (c_x1, c_y1), (c_x2, c_y2), (255, 0, 255), 2)
        cv2.putText(
            frame,
            "CRACK SCAN ZONE",
            (c_x1, c_y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 255),
            2,
        )

        crack_roi = frame[c_y1:c_y2, c_x1:c_x2]
        processed_roi, is_crack_detected, crack_count = self.crack_detector.detect(
            crack_roi
        )

        frame[c_y1:c_y2, c_x1:c_x2] = processed_roi

        if is_crack_detected:
            self.get_logger().warn(
                f"⚠️ Увага: Виявлено {crack_count} тріщин(у) в зоні інспекції!",
                throttle_duration_sec=2.0,
            )
        # ==========================================

        cv2.imshow("ArUco Debug", frame)
        cv2.waitKey(1)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub_raw.publish(msg)

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
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
