#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from cv_bridge import CvBridge

# --- УЗЕЛ НАВИГАЦИИ И УПРАВЛЕНИЯ ---
class ArucoNavigationNode(Node):
    def __init__(self):
        super().__init__("aruco_navigation_node")
        self.bridge = CvBridge()
        self.log_threshold = 150
        
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        # Публикации команд
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.buzzer_pub = self.create_publisher(Int32, "/buzzer_signal", 10)

        # Подписка на кадры от Vision Node
        self.image_sub = self.create_subscription(
            Image, 
            "/vision/processed_frame", 
            self.image_callback, 
            10
        )

        self.passed_markers = []
        self.mission_stage = 0
        self.get_logger().info("✅ Aruco Navigation Node запущен. Ожидание кадров от Vision Node...")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2

        # Визуализация центра
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 1)
        cv2.rectangle(frame, (cx - self.log_threshold, cy - self.log_threshold),
                      (cx + self.log_threshold, cy + self.log_threshold), (200, 200, 200), 1)

        # Детекция ArUco
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        cmd = Twist()

        if ids is not None:
            current_id = ids[0][0]

            if current_id not in self.passed_markers:
                c = corners[0][0]
                m_x = int(c[:, 0].mean())
                m_y = int(c[:, 1].mean())
                marker_width = abs(c[0][0] - c[1][0])
                offset_x = m_x - cx
                offset_y = cy - m_y

                # Логика достижения цели
                if marker_width > 230:
                    self.get_logger().warn(f"🎯 ЦІЛЬ ДОСЯГНУТА: ID {current_id}")
                    self.passed_markers.append(current_id)
                    self.mission_stage += 1

                    self.buzzer_pub.publish(Int32(data=1))
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info(f"🔎 Перехід до пошуку наступного маркера (Етап {self.mission_stage})")
                else:
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    if abs(offset_x) < self.log_threshold and abs(offset_y) < self.log_threshold:
                        self.get_logger().info(f"📡 Супровід: ID {current_id} | Ширина: {int(marker_width)}")

                    if abs(offset_x) > 30:
                        cmd.angular.z = -0.004 * offset_x
                        cmd.linear.x = 0.05
                    else:
                        cmd.linear.x = 0.15

        self.cmd_pub.publish(cmd)
        
        # Вывод на экран
        cv2.imshow("Integrated Robot Vision", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = ArucoNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()