import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Параметри камери
        self.declare_parameter('device', '/dev/video0')
        self.cap = cv2.VideoCapture(self.get_parameter('device').value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Publisher для мотора
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Налаштування ArUco (Original 5x5)
        self.aruco_dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_parameters = aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)
        self.get_logger().info("✅ Контролер ArUco зі зміщенням запущено")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        # Визначаємо центр кадру
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2

        # Малюємо статичне перехрестя центру
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dictionary, parameters=self.aruco_parameters)

        cmd = Twist()

        if ids is not None:
            # Працюємо з першим знайденим маркером
            c = corners[0][0]
            m_x = int(c[:, 0].mean())
            m_y = int(c[:, 1].mean())

            # === РОЗРАХУНОК ЗМІЩЕННЯ ===
            offset_x = m_x - cx
            offset_y = cy - m_y  # Вгору — плюс, вниз — мінус

            # Візуалізація зміщення
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (m_x, m_y), 5, (0, 255, 0), -1)
            cv2.line(frame, (cx, cy), (m_x, m_y), (255, 255, 0), 2) # Лінія від центру до маркера

            # Текст на екрані
            info_text = f"DX: {offset_x} DY: {offset_y}"
            cv2.putText(frame, info_text, (m_x + 10, m_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # --- ЛОГІКА РУХУ ---
            # Поворот залежно від DX
            if abs(offset_x) > 30:
                cmd.angular.z = -0.003 * offset_x
                cmd.linear.x = 0.05
            else:
                # Якщо маркер по центру — їдемо вперед
                cmd.linear.x = 0.15
                cmd.angular.z = 0.0
            
            self.get_logger().info(f"ID: {ids[0][0]} | DX: {offset_x} | DY: {offset_y}")
        else:
            # Якщо ціль втрачено — стоп
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        cv2.imshow("ArUco Navigation", frame)
        cv2.waitKey(1)

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
