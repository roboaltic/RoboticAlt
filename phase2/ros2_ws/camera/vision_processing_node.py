#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# --- АНАЛИТИЧЕСКИЕ МОДУЛИ ---
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
        valid_cnts = [c for c in contours if cv2.contourArea(c) > self.params["min_area"]]
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
        contours, _ = cv2.findContours(refined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cracks = []
        for c in contours:
            if 100 < cv2.contourArea(c) < 5000:
                rect = cv2.minAreaRect(c)
                w, h = rect[1]
                if max(w, h) / (min(w, h) + 0.1) > 2.0:
                    cracks.append(c)
        return cracks

# --- ОСНОВНОЙ УЗЕЛ ЗРЕНИЯ ---
class VisionProcessingNode(Node):
    def __init__(self):
        super().__init__("vision_processing_node")
        self.bridge = CvBridge()
        
        self.obj_dev = ObjectEdgeDetector()
        self.crack_dev = CrackDetector()

        # ВАЖНО: Укажи здесь топик, в который публикует твоя стандартная нода камеры
        self.image_sub = self.create_subscription(
            Image, 
            "/camera/image_raw", 
            self.image_callback, 
            10
        )
        
        # Публикуем обработанный кадр для следующей ноды
        self.image_pub = self.create_publisher(Image, "/vision/processed_frame", 10)
        self.get_logger().info("✅ Vision Node запущен. Ожидание кадров с камеры...")

    def image_callback(self, msg):
        # Конвертируем ROS-сообщение в OpenCV формат
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        obj_mask, obj_contours = self.obj_dev.get_object_mask(frame)
        cracks = self.crack_dev.detect(frame, obj_mask)

        # Отрисовка контуров объектов и трещин
        cv2.drawContours(frame, obj_contours, -1, (255, 0, 0), 2)
        cv2.drawContours(frame, cracks, -1, (0, 0, 255), 3)

        # Конвертируем обратно и публикуем
        processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(processed_msg)

def main():
    rclpy.init()
    node = VisionProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()