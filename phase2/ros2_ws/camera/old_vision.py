#!/usr/bin/env python3

# import cv2
# import cv2.aruco as aruco
# import numpy as np

# import rclpy
# from rclpy.node import Node

# from cv_bridge import CvBridge
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Int32, Float32
# from sensor_msgs.msg import Image


# # --- АНАЛІТИЧНІ МОДУЛІ ---


# class ObjectEdgeDetector:
#     def __init__(self):
#         self.params = {
#             "min_area": 5000,
#             "kernel": np.ones((15, 15), np.uint8)
#         }

#     def get_object_mask(self, frame):
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (7, 7), 0)
#         edge = cv2.Canny(blur, 50, 150)

#         mask = cv2.morphologyEx(
#             edge,
#             cv2.MORPH_CLOSE,
#             self.params["kernel"]
#         )

#         contours, _ = cv2.findContours(
#             mask,
#             cv2.RETR_EXTERNAL,
#             cv2.CHAIN_APPROX_SIMPLE
#         )

#         obj_mask = np.zeros_like(mask)

#         valid_contours = [
#             c for c in contours
#             if cv2.contourArea(c) > self.params["min_area"]
#         ]

#         cv2.drawContours(obj_mask, valid_contours, -1, 255, -1)

#         return obj_mask, valid_contours


# class CrackDetector:
#     def __init__(self):
#         self.kernel = cv2.getStructuringElement(
#             cv2.MORPH_ELLIPSE,
#             (9, 9)
#         )

#     def detect(self, frame, obj_mask):
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         blackhat = cv2.morphologyEx(
#             gray,
#             cv2.MORPH_BLACKHAT,
#             self.kernel
#         )

#         _, thresh = cv2.threshold(
#             blackhat,
#             0,
#             255,
#             cv2.THRESH_BINARY + cv2.THRESH_OTSU
#         )

#         refined_mask = cv2.bitwise_and(thresh, obj_mask)

#         contours, _ = cv2.findContours(
#             refined_mask,
#             cv2.RETR_EXTERNAL,
#             cv2.CHAIN_APPROX_SIMPLE
#         )

#         cracks = []

#         for c in contours:
#             area = cv2.contourArea(c)

#             if 100 < area < 5000:
#                 rect = cv2.minAreaRect(c)
#                 w, h = rect[1]

#                 if min(w, h) <= 0:
#                     continue

#                 ratio = max(w, h) / (min(w, h) + 0.1)

#                 if ratio > 2.0:
#                     cracks.append(c)

#         return cracks


# # --- ОСНОВНА НОДА ОБРОБКИ ---


# class VisionProcessorNode(Node):
#     def __init__(self):
#         super().__init__("vision_processor_node")

#         self.declare_parameter("input_image_topic", "/image_raw")
#         self.declare_parameter("processed_image_topic", "/image_processed")

#         self.declare_parameter("cmd_topic", "/cmd_vel")
#         self.declare_parameter("buzzer_topic", "/buzzer_signal")

#         self.declare_parameter("log_threshold", 150)
#         self.declare_parameter("marker_stop_width", 230.0)

#         self.declare_parameter("turn_kp", 0.004)
#         self.declare_parameter("center_deadzone_px", 30)
#         self.declare_parameter("search_linear_speed", 0.05)
#         self.declare_parameter("forward_speed", 0.15)

#         self.input_image_topic = self.get_parameter("input_image_topic").value
#         self.processed_image_topic = self.get_parameter("processed_image_topic").value

#         self.cmd_topic = self.get_parameter("cmd_topic").value
#         self.buzzer_topic = self.get_parameter("buzzer_topic").value

#         self.log_threshold = int(self.get_parameter("log_threshold").value)
#         self.marker_stop_width = float(self.get_parameter("marker_stop_width").value)

#         self.turn_kp = float(self.get_parameter("turn_kp").value)
#         self.center_deadzone_px = int(self.get_parameter("center_deadzone_px").value)
#         self.search_linear_speed = float(self.get_parameter("search_linear_speed").value)
#         self.forward_speed = float(self.get_parameter("forward_speed").value)

#         self.bridge = CvBridge()

#         self.obj_detector = ObjectEdgeDetector()
#         self.crack_detector = CrackDetector()

#         self.passed_markers = []
#         self.mission_stage = 0

#         self.aruco_dict = self.create_aruco_dictionary()
#         self.aruco_params = self.create_aruco_parameters()
#         self.aruco_detector = self.create_aruco_detector()

#         self.image_sub = self.create_subscription(
#             Image,
#             self.input_image_topic,
#             self.image_callback,
#             10
#         )

#         self.processed_image_pub = self.create_publisher(
#             Image,
#             self.processed_image_topic,
#             10
#         )

#         self.cmd_pub = self.create_publisher(
#             Twist,
#             self.cmd_topic,
#             10
#         )

#         self.buzzer_pub = self.create_publisher(
#             Int32,
#             self.buzzer_topic,
#             10
#         )

#         self.get_logger().info("✅ Vision processor node запущена")
#         self.get_logger().info(f"📥 Input image: {self.input_image_topic}")
#         self.get_logger().info(f"📤 Processed image: {self.processed_image_topic}")
#         self.get_logger().info(f"🎮 CMD topic: {self.cmd_topic}")

#     def create_aruco_dictionary(self):
#         if hasattr(aruco, "getPredefinedDictionary"):
#             return aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

#         return aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

#     def create_aruco_parameters(self):
#         if hasattr(aruco, "DetectorParameters"):
#             return aruco.DetectorParameters()

#         return aruco.DetectorParameters_create()

#     def create_aruco_detector(self):
#         if hasattr(aruco, "ArucoDetector"):
#             return aruco.ArucoDetector(
#                 self.aruco_dict,
#                 self.aruco_params
#             )

#         return None

#     def detect_markers(self, gray):
#         if self.aruco_detector is not None:
#             corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
#             return corners, ids, rejected

#         corners, ids, rejected = aruco.detectMarkers(
#             gray,
#             self.aruco_dict,
#             parameters=self.aruco_params
#         )

#         return corners, ids, rejected

#     def image_callback(self, msg):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(
#                 msg,
#                 desired_encoding="bgr8"
#             )

#         except Exception as e:
#             self.get_logger().error(f"❌ Помилка конвертації image_raw: {e}")
#             return

#         processed_frame, cmd = self.process_frame(frame)

#         self.cmd_pub.publish(cmd)
#         self.publish_processed_image(processed_frame, msg)

#     def process_frame(self, frame):
#         h, w, _ = frame.shape
#         cx, cy = w // 2, h // 2

#         cmd = Twist()

#         # --- АНАЛІТИКА: об'єкти і тріщини ---
#         obj_mask, obj_contours = self.obj_detector.get_object_mask(frame)
#         cracks = self.crack_detector.detect(frame, obj_mask)

#         # --- ЦЕНТРАЛЬНИЙ ПРИЦІЛ ---
#         cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 1)
#         cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 1)

#         cv2.rectangle(
#             frame,
#             (cx - self.log_threshold, cy - self.log_threshold),
#             (cx + self.log_threshold, cy + self.log_threshold),
#             (200, 200, 200),
#             1,
#         )

#         # --- ARUCO ---
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         corners, ids, _ = self.detect_markers(gray)

#         if ids is not None and len(ids) > 0:
#             current_id = int(ids[0][0])

#             if current_id not in self.passed_markers:
#                 c = corners[0][0]

#                 marker_x = int(c[:, 0].mean())
#                 marker_y = int(c[:, 1].mean())

#                 marker_width = abs(c[0][0] - c[1][0])

#                 offset_x = marker_x - cx
#                 offset_y = cy - marker_y

#                 aruco.drawDetectedMarkers(frame, corners, ids)

#                 cv2.putText(
#                     frame,
#                     f"ARUCO ID: {current_id}",
#                     (20, 35),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.8,
#                     (0, 255, 0),
#                     2
#                 )

#                 cv2.putText(
#                     frame,
#                     f"width: {int(marker_width)} offset_x: {offset_x}",
#                     (20, 70),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7,
#                     (0, 255, 0),
#                     2
#                 )

#                 # --- ЦІЛЬ ДОСЯГНУТА ---
#                 if marker_width > self.marker_stop_width:
#                     self.get_logger().warn(
#                         f"🎯 ЦІЛЬ ДОСЯГНУТА: ID {current_id}"
#                     )

#                     self.passed_markers.append(current_id)
#                     self.mission_stage += 1

#                     self.buzzer_pub.publish(Int32(data=1))

#                     cmd.linear.x = 0.0
#                     cmd.angular.z = 0.0

#                     self.get_logger().info(
#                         f"🔎 Пошук наступного маркера. Етап: {self.mission_stage}"
#                     )

#                 else:
#                     if (
#                         abs(offset_x) < self.log_threshold
#                         and abs(offset_y) < self.log_threshold
#                     ):
#                         self.get_logger().info(
#                             f"📡 Супровід: ID {current_id} | Ширина: {int(marker_width)}"
#                         )

#                     # --- РУХ ДО МАРКЕРА ---
#                     if abs(offset_x) > self.center_deadzone_px:
#                         cmd.angular.z = -self.turn_kp * float(offset_x)
#                         cmd.linear.x = self.search_linear_speed
#                     else:
#                         cmd.linear.x = self.forward_speed
#                         cmd.angular.z = 0.0

#             else:
#                 # Уже пройдений маркер ігноруємо
#                 cmd.linear.x = 0.0
#                 cmd.angular.z = 0.0

#         else:
#             # Якщо ArUco немає — стоїмо
#             cmd.linear.x = 0.0
#             cmd.angular.z = 0.0

#             cv2.putText(
#                 frame,
#                 "NO ARUCO",
#                 (20, 35),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 0.8,
#                 (0, 0, 255),
#                 2
#             )

#         # --- МАЛЮЄМО ОБ'ЄКТИ І ТРІЩИНИ ---
#         cv2.drawContours(frame, obj_contours, -1, (255, 0, 0), 2)
#         cv2.drawContours(frame, cracks, -1, (0, 0, 255), 3)

#         cv2.putText(
#             frame,
#             f"cracks: {len(cracks)}",
#             (20, 105),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             0.7,
#             (0, 0, 255),
#             2
#         )

#         return frame, cmd

#     def publish_processed_image(self, frame, source_msg):
#         try:
#             out_msg = self.bridge.cv2_to_imgmsg(
#                 frame,
#                 encoding="bgr8"
#             )

#             out_msg.header.stamp = self.get_clock().now().to_msg()
#             out_msg.header.frame_id = source_msg.header.frame_id

#             self.processed_image_pub.publish(out_msg)

#         except Exception as e:
#             self.get_logger().error(f"❌ Помилка публікації image_processed: {e}")


# def main():
#     rclpy.init()

#     node = VisionProcessorNode()

#     try:
#         rclpy.spin(node)

#     except KeyboardInterrupt:
#         pass

#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
