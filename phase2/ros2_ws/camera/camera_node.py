#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import rclpy

# from camera.crack_detector import CrackDetector, ObjectEdgeDetector
from crack_detector import CrackDetector, ObjectEdgeDetector
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DEV = "/dev/video0"  # в цю змінну вписуємо шлях до камери


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

        # Получаем размеры кадра
        height, width, _ = frame.shape
        center_f_x = width // 2
        center_f_y = height // 2

        # ==========================================================
        # 1. ПОШУК ОБ'ЄКТІВ (На всьому кадрі)
        # Получаем обновленный кадр и список контуров найденных деталей
        frame, found_objects = self.edge_detector.detect(frame)
        # ==========================================================

        # === НАЛАШТУВАННЯ ЗОНИ СКАНУВАННЯ ARUCO ===
        roi_w, roi_h = width // 2, height // 2
        x1, y1 = center_f_x - (roi_w // 2), center_f_y - (roi_h // 2)
        x2, y2 = center_f_x + (roi_w // 2), center_f_y + (roi_h // 2)

        # Отрисовка прицела ArUco
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

        # Работа с ArUco (внутри ROI)
        roi_aruco = frame[y1:y2, x1:x2]
        gray_aruco = cv2.cvtColor(roi_aruco, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray_aruco, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        if ids is not None:
            # Коррекция координат ArUco под полный кадр
            for i in range(len(corners)):
                corners[i][0][:, 0] += x1
                corners[i][0][:, 1] += y1

            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                c = corners[i][0]
                m_x, m_y = int(c[:, 0].mean()), int(c[:, 1].mean())

                offset_x = m_x - center_f_x
                offset_y = center_f_y - m_y

                cv2.circle(frame, (m_x, m_y), 5, (0, 255, 0), -1)
                cv2.putText(
                    frame,
                    f"ID: {ids[i][0]} DX: {offset_x} DY: {offset_y}",
                    (m_x + 10, m_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2,
                )

                if abs(offset_x) <= 25 and abs(offset_y) <= 25:
                    self.get_logger().info(
                        f"🎯 TARGET LOCKED: ID {ids[i][0]}", throttle_duration_sec=1.0
                    )

        # ==========================================================
        # 2. ПОШУК ТРІЩИН (Тільки всередині знайдених об'єктів)
        # ==========================================================
        scan_w, scan_h = 400, 300
        c_x1 = center_f_x - (scan_w // 2)
        c_y1 = center_f_y - (scan_h // 2)
        c_x2 = center_f_x + (scan_w // 2)
        c_y2 = center_f_y + (scan_h // 2)

        # Малюємо рамку зони інспекції трещин
        cv2.rectangle(frame, (c_x1, c_y1), (c_x2, c_y2), (255, 0, 255), 2)

        # Вирізаємо область для детектора трещин
        crack_roi = frame[c_y1:c_y2, c_x1:c_x2]

        # Передаємо: саму картинку, контури об'єктів та зміщення (offset) координат
        processed_roi, is_crack_detected, crack_count = self.crack_detector.detect(
            crack_roi, parent_contours=found_objects, offset=(c_x1, c_y1)
        )

        # Повертаємо оброблений ROI назад на основний кадр
        frame[c_y1:c_y2, c_x1:c_x2] = processed_roi

        if is_crack_detected:
            self.get_logger().warn(
                f"⚠️ CRITICAL: Found {crack_count} cracks inside objects!",
                throttle_duration_sec=2.0,
            )
        # ==========================================================

        # Показ результату та публікація
        cv2.imshow("Detection Debug", frame)
        cv2.waitKey(1)

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub_raw.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

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
