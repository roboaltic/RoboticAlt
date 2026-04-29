import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # Додано для керування моторами
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Параметри
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('fps', 15)
        
        # Створюємо Publisher для команд моторам
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Налаштування ArUco
        self.aruco_dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_parameters = aruco.DetectorParameters_create()
        
        self.cap = cv2.VideoCapture(self.get_parameter('device').value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)
        
        self.get_logger().info("✅ Нода камери-контролера запущена")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        height, width, _ = frame.shape
        center_f_x = width // 2
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dictionary, parameters=self.aruco_parameters)

        # Створюємо порожнє повідомлення швидкості
        cmd = Twist()

        if ids is not None:
            # Беремо перший знайдений маркер
            c = corners[0][0]
            m_x = int(c[:, 0].mean())
            offset_x = m_x - center_f_x

            # --- ЛОГІКА КЕРУВАННЯ ---
            
            # 1. Центрування (поворот)
            # Чим далі маркер від центру, тим швидше повертаємо
            if abs(offset_x) > 40:  # "Мертва зона" 40 пікселів
                cmd.angular.z = -0.002 * offset_x  # Коефіцієнт швидкості повороту
                cmd.linear.x = 0.05               # Трохи під'їжджаємо при повороті
            else:
                # 2. Рух вперед, якщо маркер по центру
                cmd.linear.x = 0.15 
                cmd.angular.z = 0.0
                
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.putText(frame, f"Driving: L:{cmd.linear.x} A:{cmd.angular.z}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # Якщо маркера не видно — зупиняємося (безпека)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Надсилаємо команду на ноду мотора
        self.cmd_pub.publish(cmd)

        cv2.imshow("Robot View", frame)
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
