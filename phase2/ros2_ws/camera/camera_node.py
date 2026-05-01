#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import rclpy

# from camera.crack_detector import CrackDetector, ObjectEdgeDetector
from camera.crack_detector import CrackDetector, ObjectEdgeDetector
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # --- ПАРАМЕТРИ КАМЕРИ ---
        self.declare_parameter('device', '/dev/video0')
        self.cap = cv2.VideoCapture(self.get_parameter('device').value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Поріг для виведення логів (50 пікселів від центру)
        self.log_threshold = 50 

        # Налаштування ArUco (Original 5x5 згідно з Етапом 2)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        # Публікація команд руху та сигналів зумера
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buzzer_pub = self.create_publisher(Int32, '/buzzer_signal', 10)

        self.bridge = CvBridge()
        self.passed_markers = []  # Список пройдених ID (Ризик 2)
        self.mission_stage = 0    # Лічильник етапів (Етап 6)
        
        self.timer = self.create_timer(1.0 / 15, self.process_frame)
        self.get_logger().info("✅ Контролер камери запущено. Очікування мітки...")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2 # Центр кадру

        # --- ВІЗУАЛІЗАЦІЯ ЦЕНТРУ ---
        # Малюємо хрест по центру кадру
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 1)
        # Малюємо рамку зони логування
        cv2.rectangle(frame, (cx - self.log_threshold, cy - self.log_threshold), 
                      (cx + self.log_threshold, cy + self.log_threshold), (200, 200, 200), 1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        cmd = Twist()

        if ids is not None:
            current_id = ids[0][0]
            
            # Обробляємо тільки нові мітки (Етап 5)
            if current_id not in self.passed_markers:
                c = corners[0][0]
                m_x = int(c[:, 0].mean())
                m_y = int(c[:, 1].mean())
                marker_width = abs(c[0][0] - c[1][0]) # Оцінка близькості
                
                offset_x = m_x - cx
                offset_y = cy - m_y

                # Візуалізація маркера та його координат
                aruco.drawDetectedMarkers(frame, corners, ids)
                coord_text = f"DX: {offset_x} DY: {offset_y}"
                cv2.putText(frame, coord_text, (m_x + 10, m_y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                # --- ЛОГІКА РУХУ (Етап 3) ---
                if marker_width > 220: # Зупинка за ~20 см
                    self.passed_markers.append(current_id)
                    self.mission_stage += 1
                    self.buzzer_pub.publish(Int32(data=1)) # Сигнал зумера
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                elif abs(offset_x) > 30: # Автокорекція напрямку
                    cmd.angular.z = -0.003 * offset_x
                    cmd.linear.x = 0.05
                else: # Рух вперед
                    cmd.linear.x = 0.15
                    cmd.angular.z = 0.0

                # --- РОЗУМНІ ЛОГИ ---
                # Виводимо в термінал тільки якщо маркер у зоні центру
                if abs(offset_x) < self.log_threshold and abs(offset_y) < self.log_threshold:
                    self.get_logger().info(f"🎯 ЦІЛЬ: ID {current_id} | DX: {offset_x} DY: {offset_y} | Stage: {self.mission_stage}")

        self.cmd_pub.publish(cmd)
        cv2.imshow("Robot Vision", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
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
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
