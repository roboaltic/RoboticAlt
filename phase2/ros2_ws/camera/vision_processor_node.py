#!/usr/bin/env python3

import math

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32


# ---------------- OBJECT DETECTOR ----------------


class ObjectEdgeDetector:
    def __init__(self):
        self.params = {
            "min_area": 5000,
            "kernel": np.ones((15, 15), np.uint8),
        }

    def get_object_mask(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        edge = cv2.Canny(blur, 50, 150)

        mask = cv2.morphologyEx(
            edge,
            cv2.MORPH_CLOSE,
            self.params["kernel"]
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        obj_mask = np.zeros_like(mask)

        valid_contours = [
            c for c in contours
            if cv2.contourArea(c) > self.params["min_area"]
        ]

        cv2.drawContours(obj_mask, valid_contours, -1, 255, -1)

        return obj_mask, valid_contours


# ---------------- CRACK DETECTOR ----------------


class CrackDetector:
    def __init__(self):
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (9, 9)
        )

    def detect(self, frame, obj_mask):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        blackhat = cv2.morphologyEx(
            gray,
            cv2.MORPH_BLACKHAT,
            self.kernel
        )

        _, thresh = cv2.threshold(
            blackhat,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        refined_mask = cv2.bitwise_and(thresh, obj_mask)

        contours, _ = cv2.findContours(
            refined_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        cracks = []

        for c in contours:
            area = cv2.contourArea(c)

            if 100 < area < 5000:
                rect = cv2.minAreaRect(c)
                w, h = rect[1]

                if min(w, h) <= 0:
                    continue

                ratio = max(w, h) / (min(w, h) + 0.1)

                if ratio > 2.0:
                    cracks.append(c)

        return cracks


# ---------------- VISION NODE ----------------


class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__("vision_processor_node")

        # ---------------- TOPICS ----------------
        self.declare_parameter("input_image_topic", "/image_raw")
        self.declare_parameter("processed_image_topic", "/image_processed")

        self.declare_parameter("aruco_id_topic", "/aruco/id")
        self.declare_parameter("aruco_offset_topic", "/aruco/offset_px")
        self.declare_parameter("aruco_width_topic", "/aruco/width")

        # Залишено для сумісності зі старим кодом, але за замовчуванням
        # vision-нода НЕ керує моторами, щоб не конфліктувати з mission_master.
        self.declare_parameter("cmd_topic", "/vision_cmd_unused")
        self.declare_parameter("publish_cmd_enabled", False)
        self.declare_parameter("buzzer_topic", "/buzzer_signal")

        # ---------------- CV / ARUCO SETTINGS ----------------
        self.declare_parameter("aruco_dictionary", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("log_threshold", 300)
        self.declare_parameter("marker_stop_width", 230.0)

        self.declare_parameter("turn_kp", 0.004)

        # ВАЖЛИВО: float, щоб launch з 30.0 не валив ноду.
        self.declare_parameter("center_deadzone_px", 30.0)
        self.declare_parameter("search_linear_speed", 0.05)
        self.declare_parameter("forward_speed", 0.15)

        # Якщо ArUco мигнув на 1 кадр і пропав, ще aruco_hold_sec секунд
        # публікуємо останні валідні дані, а не id=-1.
        self.declare_parameter("aruco_hold_sec", 0.8)
        self.declare_parameter("aruco_tx_log_period_sec", 0.35)

        # ---------------- READ PARAMETERS ----------------
        self.input_image_topic = self.get_parameter("input_image_topic").value
        self.processed_image_topic = self.get_parameter("processed_image_topic").value

        self.aruco_id_topic = self.get_parameter("aruco_id_topic").value
        self.aruco_offset_topic = self.get_parameter("aruco_offset_topic").value
        self.aruco_width_topic = self.get_parameter("aruco_width_topic").value

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.publish_cmd_enabled = bool(
            self.get_parameter("publish_cmd_enabled").value
        )
        self.buzzer_topic = self.get_parameter("buzzer_topic").value

        self.aruco_dictionary_name = self.get_parameter("aruco_dictionary").value
        self.log_threshold = int(self.get_parameter("log_threshold").value)
        self.marker_stop_width = float(self.get_parameter("marker_stop_width").value)

        self.turn_kp = float(self.get_parameter("turn_kp").value)
        self.center_deadzone_px = float(
            self.get_parameter("center_deadzone_px").value
        )
        self.search_linear_speed = float(
            self.get_parameter("search_linear_speed").value
        )
        self.forward_speed = float(self.get_parameter("forward_speed").value)

        self.aruco_hold_sec = float(self.get_parameter("aruco_hold_sec").value)
        self.aruco_tx_log_period_sec = float(
            self.get_parameter("aruco_tx_log_period_sec").value
        )

        # ---------------- INTERNAL STATE ----------------
        self.bridge = CvBridge()

        self.obj_detector = ObjectEdgeDetector()
        self.crack_detector = CrackDetector()

        self.mission_stage = 0

        self.last_valid_aruco_id = -1
        self.last_valid_offset_x = 0.0
        self.last_valid_width = 0.0
        self.last_valid_aruco_time = -999.0
        self.last_tx_log_time = 0.0

        self.aruco_dict = self.create_aruco_dictionary()
        self.aruco_params = self.create_aruco_parameters()

        # ---------------- ROS SUB/PUB ----------------
        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            10
        )

        self.processed_image_pub = self.create_publisher(
            Image,
            self.processed_image_topic,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_topic,
            10
        )

        self.buzzer_pub = self.create_publisher(
            Int32,
            self.buzzer_topic,
            10
        )

        self.aruco_id_pub = self.create_publisher(
            Int32,
            self.aruco_id_topic,
            10
        )

        self.aruco_offset_pub = self.create_publisher(
            Float32,
            self.aruco_offset_topic,
            10
        )

        self.aruco_width_pub = self.create_publisher(
            Float32,
            self.aruco_width_topic,
            10
        )

        self.get_logger().info("✅ Vision processor node запущена")
        self.get_logger().info(f"📥 Input image: {self.input_image_topic}")
        self.get_logger().info(f"📤 Processed image: {self.processed_image_topic}")
        self.get_logger().info(
            f"📤 ArUco publishers: "
            f"id={self.aruco_id_topic}, "
            f"offset={self.aruco_offset_topic}, "
            f"width={self.aruco_width_topic}"
        )
        self.get_logger().info(
            f"🎮 CMD topic: {self.cmd_topic} | "
            f"publish_cmd_enabled={self.publish_cmd_enabled}"
        )
        self.get_logger().info(
            f"🧠 ArUco dictionary: {self.aruco_dictionary_name} | "
            f"hold={self.aruco_hold_sec:.2f}s"
        )

    # ============================================================
    # UTILS
    # ============================================================

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_aruco_data(self, marker_id, offset_px=0.0, width=0.0):
        id_msg = Int32()
        id_msg.data = int(marker_id)
        self.aruco_id_pub.publish(id_msg)

        offset_msg = Float32()
        offset_msg.data = float(offset_px)
        self.aruco_offset_pub.publish(offset_msg)

        width_msg = Float32()
        width_msg.data = float(width)
        self.aruco_width_pub.publish(width_msg)

        now = self.now_sec()
        if (now - self.last_tx_log_time) >= self.aruco_tx_log_period_sec:
            self.get_logger().info(
                f"📤 ArUco TX: id={int(marker_id)}, "
                f"offset={float(offset_px):.1f}, "
                f"width={float(width):.1f}"
            )
            self.last_tx_log_time = now

    def marker_side_width_px(self, marker_corners):
        """
        marker_corners має форму (4, 2).
        Рахуємо середню довжину 4 сторін, а не тільки різницю X,
        бо при нахилі маркера стара формула може давати width=1.
        """
        c = np.asarray(marker_corners, dtype=np.float32)

        if c.shape != (4, 2):
            return 0.0

        side_lengths = []
        for i in range(4):
            p1 = c[i]
            p2 = c[(i + 1) % 4]
            side_lengths.append(float(np.linalg.norm(p2 - p1)))

        return float(sum(side_lengths) / len(side_lengths))

    def select_best_marker(self, corners, ids):
        """
        Якщо в кадрі кілька маркерів — беремо найбільший,
        бо він найімовірніше є основною ціллю.
        """
        best_index = 0
        best_width = -1.0

        for i, c in enumerate(corners):
            width = self.marker_side_width_px(c[0])
            if width > best_width:
                best_width = width
                best_index = i

        marker_id = int(ids[best_index][0])
        marker_corners = corners[best_index][0]
        marker_width = max(0.0, best_width)

        return marker_id, marker_corners, marker_width

    # ============================================================
    # ARUCO SETUP
    # ============================================================

    def create_aruco_dictionary(self):
        dictionary_map = {
            "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
            "DICT_4X4_50": aruco.DICT_4X4_50,
            "DICT_4X4_100": aruco.DICT_4X4_100,
            "DICT_5X5_50": aruco.DICT_5X5_50,
            "DICT_5X5_100": aruco.DICT_5X5_100,
            "DICT_6X6_50": aruco.DICT_6X6_50,
            "DICT_6X6_100": aruco.DICT_6X6_100,
            "DICT_6X6_250": aruco.DICT_6X6_250,
        }

        dict_id = dictionary_map.get(
            str(self.aruco_dictionary_name),
            aruco.DICT_ARUCO_ORIGINAL
        )

        if hasattr(aruco, "getPredefinedDictionary"):
            return aruco.getPredefinedDictionary(dict_id)

        return aruco.Dictionary_get(dict_id)

    def create_aruco_parameters(self):
        if hasattr(aruco, "DetectorParameters_create"):
            return aruco.DetectorParameters_create()

        return aruco.DetectorParameters()

    def detect_markers(self, gray):
        try:
            corners, ids, rejected = aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

            return corners, ids, rejected

        except Exception as e:
            self.get_logger().error(f"❌ ArUco detect error: {e}")
            return [], None, []

    # ============================================================
    # ROS IMAGE CALLBACK
    # ============================================================

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except Exception as e:
            self.get_logger().error(f"❌ Помилка конвертації /image_raw: {e}")
            return

        try:
            processed_frame, cmd = self.process_frame(frame)

            if self.publish_cmd_enabled:
                self.cmd_pub.publish(cmd)

            self.publish_processed_image(processed_frame, msg)

        except Exception as e:
            self.get_logger().error(f"❌ Помилка обробки кадру: {e}")

    # ============================================================
    # FRAME PROCESSING
    # ============================================================

    def process_frame(self, frame):
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        obj_mask, obj_contours = self.obj_detector.get_object_mask(frame)
        cracks = self.crack_detector.detect(frame, obj_mask)

        # Центр кадру
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 1)

        cv2.rectangle(
            frame,
            (cx - self.log_threshold, cy - self.log_threshold),
            (cx + self.log_threshold, cy + self.log_threshold),
            (200, 200, 200),
            1,
        )

        # ---------------- ARUCO ----------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect_markers(gray)

        if ids is not None and len(ids) > 0:
            current_id, c, marker_width = self.select_best_marker(corners, ids)

            marker_x = int(c[:, 0].mean())
            marker_y = int(c[:, 1].mean())

            offset_x = float(marker_x - cx)
            offset_y = float(cy - marker_y)

            now = self.now_sec()
            self.last_valid_aruco_id = current_id
            self.last_valid_offset_x = float(offset_x)
            self.last_valid_width = float(marker_width)
            self.last_valid_aruco_time = now

            self.publish_aruco_data(
                current_id,
                offset_x,
                marker_width
            )

            try:
                aruco.drawDetectedMarkers(frame, corners, ids)
            except Exception:
                pass

            cv2.putText(
                frame,
                f"ARUCO ID: {current_id}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"width: {marker_width:.1f} offset_x: {offset_x:.0f}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            if marker_width >= self.marker_stop_width:
                cv2.putText(
                    frame,
                    "ARUCO CLOSE / TRIGGER",
                    (20, 135),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

            # Старий режим керування залишений тільки якщо publish_cmd_enabled=True.
            if self.publish_cmd_enabled:
                if abs(offset_x) > self.center_deadzone_px:
                    cmd.angular.z = -self.turn_kp * float(offset_x)
                    cmd.linear.x = self.search_linear_speed
                else:
                    cmd.linear.x = self.forward_speed
                    cmd.angular.z = 0.0

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            now = self.now_sec()
            lost_time = now - self.last_valid_aruco_time

            if (
                self.last_valid_aruco_id >= 0
                and lost_time <= self.aruco_hold_sec
            ):
                # Не скидаємо ArUco одразу після одного невдалого кадру.
                # Це вирішує ситуацію: id=1 -> id=-1 через 0.03 сек.
                self.publish_aruco_data(
                    self.last_valid_aruco_id,
                    self.last_valid_offset_x,
                    self.last_valid_width
                )

                cv2.putText(
                    frame,
                    f"ARUCO HOLD ID: {self.last_valid_aruco_id}",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    f"hold: {lost_time:.2f}/{self.aruco_hold_sec:.2f}s",
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

            else:
                self.publish_aruco_data(-1, 0.0, 0.0)

                cv2.putText(
                    frame,
                    "NO ARUCO",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

        # ---------------- CRACK / OBJECT VISUALIZATION ----------------
        cv2.drawContours(frame, obj_contours, -1, (255, 0, 0), 2)
        cv2.drawContours(frame, cracks, -1, (0, 0, 255), 3)

        cv2.putText(
            frame,
            f"cracks: {len(cracks)}",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        return frame, cmd

    def publish_processed_image(self, frame, source_msg):
        try:
            out_msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8"
            )

            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = source_msg.header.frame_id

            self.processed_image_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"❌ Помилка публікації /image_processed: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = VisionProcessorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
