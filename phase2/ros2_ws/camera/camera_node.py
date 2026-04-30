#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Оголошення параметрів ROS 2
        self.declare_parameter('device', '/dev/video2')
        self.declare_parameter('fps', 15)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        device = self.get_parameter('device').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        
        # Налаштування ArUco
        self.aruco_dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_parameters = aruco.DetectorParameters_create()
        self.aruco_parameters.adaptiveThreshConstant = 7
        self.aruco_parameters.minMarkerPerimeterRate = 0.03

        # Відкриття камери
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Не вдалося відкрити камеру на {device}")
            raise RuntimeError('Camera open failed')

        from rclpy.qos import QoSProfile
        self.pub_raw = self.create_publisher(Image, '/image_raw', QoSProfile(depth=10))
        self.bridge = CvBridge()

        # Таймер оновлення кадрів
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info("✅ CameraNode запущена (Режим обмеженої зони сканування + Логи по центру)")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        # Отримуємо розміри кадру
        height, width, _ = frame.shape
        center_f_x = width // 2
        center_f_y = height // 2

        # === НАЛАШТУВАННЯ ЗОНИ СКАНУВАННЯ (ROI) ===
        # Беремо 50% від ширини та висоти екрану по центру
        roi_w = width // 2
        roi_h = height // 2
        
        x1 = center_f_x - (roi_w // 2)
        y1 = center_f_y - (roi_h // 2)
        x2 = center_f_x + (roi_w // 2)
        y2 = center_f_y + (roi_h // 2)

        # Малюємо оранжеву рамку зони пошуку (маркер буде видно тільки в ній)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)

        # Малюємо червоне перехрестя центру кадру для візуалізації
        cv2.line(frame, (center_f_x - 10, center_f_y), (center_f_x + 10, center_f_y), (0, 0, 255), 2)
        cv2.line(frame, (center_f_x, center_f_y - 10), (center_f_x, center_f_y + 10), (0, 0, 255), 2)

        # Вирізаємо частину кадру для обробки та переводимо в Ч/Б
        roi_frame = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        
        # Скануємо ТІЛЬКИ вирізану зону
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dictionary, parameters=self.aruco_parameters)

        if ids is not None:
            # Повертаємо координати кутів маркера назад до масштабів повного кадру
            for i in range(len(corners)):
                corners[i][0][:, 0] += x1
                corners[i][0][:, 1] += y1

            aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i in range(len(ids)):
                # Обчислюємо центр маркера
                c = corners[i][0]
                m_x = int(c[:, 0].mean())
                m_y = int(c[:, 1].mean())

                # Малюємо зелену точку в центрі маркера
                cv2.circle(frame, (m_x, m_y), 5, (0, 255, 0), -1)

                # === ОБЧИСЛЕННЯ ЗМІЩЕННЯ ===
                offset_x = m_x - center_f_x
                offset_y = center_f_y - m_y  # Інвертуємо Y (вгору = +)

                # Виводимо дані на екран
                text = f"ID: {ids[i][0]} DX: {offset_x} DY: {offset_y}"
                cv2.putText(frame, text, (m_x + 10, m_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                # === ЛОГУВАННЯ ТІЛЬКИ ПРИ ЦЕНТРУВАННІ ===
                tolerance = 25  # Допуск у пікселях (зона "прицілу")

                if abs(offset_x) <= tolerance and abs(offset_y) <= tolerance:
                    self.get_logger().info(
                        f"🎯 ЦІЛЬ ЗАХОПЛЕНО! Marker {ids[i][0]} в центрі (X: {offset_x}, Y: {offset_y})",
                        throttle_duration_sec=1.0
                    )

        # Показуємо вікно відлагодження
        cv2.imshow("ArUco Debug", frame)
        cv2.waitKey(1)

        # Публікуємо кадр у ROS-топік
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

if __name__ == '__main__':
    main()