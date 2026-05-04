#!/usr/bin/env python3

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int32MultiArray

try:
    import serial
except ImportError:
    serial = None


class LD06Node(Node):
    """
    LD06 / LD19 style LiDAR node for ROS2.

    Publishes:
      /scan                 sensor_msgs/LaserScan
      /lidar/zones           std_msgs/Float32MultiArray
      /lidar/blocked         std_msgs/Int32MultiArray

    /lidar/zones data order:
      [front, front_left, front_right, left, right, rear]

    /lidar/blocked data order:
      [front, front_left, front_right, left, right, rear]

    Angle convention after offset:
      0 deg   = front
      +90 deg = left
      -90 deg = right
      180 deg = rear
    """

    CRC_TABLE = [
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
        0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
        0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
        0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
        0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
        0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
        0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
        0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
        0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
        0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
        0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
        0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
        0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
        0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
        0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
        0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
        0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
        0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
        0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
        0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
        0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
        0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
        0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
        0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
        0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
    ]

    def __init__(self):
        super().__init__('ld06_node')

        if serial is None:
            raise RuntimeError(
                'pyserial is not installed. Install it with: pip3 install pyserial'
            )

        # ---------------- Parameters ----------------

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')

        # Важливо:
        # якщо стрілка на лідарі фізично дивиться вперед, але дані зміщені,
        # підбирай цей параметр: 0, 90, -90, 180 тощо.
        self.declare_parameter('angle_offset_deg', 0.0)

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('zones_topic', '/lidar/zones')
        self.declare_parameter('blocked_topic', '/lidar/blocked')

        self.declare_parameter('publish_rate_hz', 10.0)

        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('range_max_m', 12.0)

        # LD06 іноді дає погані точки з низькою якістю.
        # Якщо точок мало — зменшуй до 5 або 0.
        self.declare_parameter('min_confidence', 10)

        # Роздільність LaserScan.
        # 1.0 = 360 точок.
        # 0.5 = 720 точок.
        self.declare_parameter('angle_resolution_deg', 1.0)

        # Safety threshold для blocked-зон.
        self.declare_parameter('stop_distance_m', 0.40)

        # Зони у градусах відносно переду робота.
        self.declare_parameter('front_min_deg', -20.0)
        self.declare_parameter('front_max_deg', 20.0)

        self.declare_parameter('front_left_min_deg', 20.0)
        self.declare_parameter('front_left_max_deg', 70.0)

        self.declare_parameter('front_right_min_deg', -70.0)
        self.declare_parameter('front_right_max_deg', -20.0)

        self.declare_parameter('left_min_deg', 70.0)
        self.declare_parameter('left_max_deg', 120.0)

        self.declare_parameter('right_min_deg', -120.0)
        self.declare_parameter('right_max_deg', -70.0)

        self.declare_parameter('rear_min_deg', 150.0)
        self.declare_parameter('rear_max_deg', -150.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.range_min_m = float(self.get_parameter('range_min_m').value)
        self.range_max_m = float(self.get_parameter('range_max_m').value)
        self.min_confidence = int(self.get_parameter('min_confidence').value)
        self.angle_resolution_deg = float(
            self.get_parameter('angle_resolution_deg').value
        )
        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.zones_topic = self.get_parameter('zones_topic').value
        self.blocked_topic = self.get_parameter('blocked_topic').value

        # ---------------- Publishers ----------------

        self.scan_pub = self.create_publisher(
            LaserScan,
            self.scan_topic,
            10
        )

        self.zones_pub = self.create_publisher(
            Float32MultiArray,
            self.zones_topic,
            10
        )

        self.blocked_pub = self.create_publisher(
            Int32MultiArray,
            self.blocked_topic,
            10
        )

        # ---------------- Scan buffers ----------------

        self.bin_count = int(round(360.0 / self.angle_resolution_deg))
        self.ranges = [math.inf] * self.bin_count
        self.intensities = [0.0] * self.bin_count
        self.last_update_time = [0.0] * self.bin_count

        self.data_lock = threading.Lock()
        self.running = True

        # ---------------- Serial ----------------

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.05
        )

        self.reader_thread = threading.Thread(
            target=self.read_loop,
            daemon=True
        )
        self.reader_thread.start()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.publish_scan_and_zones)

        self.get_logger().info(
            f'LD06 node started: port={self.port}, baud={self.baudrate}, '
            f'angle_offset={self.angle_offset_deg} deg, bins={self.bin_count}'
        )

    # -------------------------------------------------------------------------
    # CRC
    # -------------------------------------------------------------------------

    def crc8(self, data: bytes) -> int:
        crc = 0
        for b in data:
            crc = self.CRC_TABLE[(crc ^ b) & 0xFF]
        return crc

    # -------------------------------------------------------------------------
    # Angle helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def normalize_0_360(angle_deg: float) -> float:
        angle_deg = angle_deg % 360.0
        if angle_deg < 0.0:
            angle_deg += 360.0
        return angle_deg

    @staticmethod
    def normalize_minus_180_180(angle_deg: float) -> float:
        angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
        return angle_deg

    def angle_to_bin(self, angle_deg_robot: float) -> int:
        angle_0_360 = self.normalize_0_360(angle_deg_robot)
        idx = int(round(angle_0_360 / self.angle_resolution_deg)) % self.bin_count
        return idx

    def angle_in_zone(self, angle_deg: float, zone_min: float, zone_max: float) -> bool:
        """
        angle_deg is in -180..180.
        Normal zone:
          -20..20
        Wrap-around zone:
          150..-150 means angle >= 150 or angle <= -150.
        """
        if zone_min <= zone_max:
            return zone_min <= angle_deg <= zone_max
        else:
            return angle_deg >= zone_min or angle_deg <= zone_max

    # -------------------------------------------------------------------------
    # LD06 reading
    # -------------------------------------------------------------------------

    def read_loop(self):
        buffer = bytearray()

        while self.running and rclpy.ok():
            try:
                data = self.serial.read(256)
                if data:
                    buffer.extend(data)

                while len(buffer) >= 47:
                    # LD06 packet starts with 0x54 0x2C
                    if buffer[0] != 0x54 or buffer[1] != 0x2C:
                        buffer.pop(0)
                        continue

                    packet = bytes(buffer[:47])
                    del buffer[:47]

                    if not self.valid_packet(packet):
                        continue

                    self.parse_packet(packet)

            except Exception as e:
                self.get_logger().warn(f'LD06 read error: {e}')
                time.sleep(0.1)

    def valid_packet(self, packet: bytes) -> bool:
        if len(packet) != 47:
            return False

        if packet[0] != 0x54 or packet[1] != 0x2C:
            return False

        expected_crc = packet[46]
        calculated_crc = self.crc8(packet[:46])

        return calculated_crc == expected_crc

    def parse_packet(self, packet: bytes):
        """
        Packet layout:
          0      header 0x54
          1      ver_len 0x2C
          2-3    speed
          4-5    start angle, centi-degrees
          6-41   12 points: distance uint16 mm + confidence uint8
          42-43  end angle, centi-degrees
          44-45  timestamp
          46     crc
        """

        start_angle_raw = struct.unpack_from('<H', packet, 4)[0]
        end_angle_raw = struct.unpack_from('<H', packet, 42)[0]

        start_angle_deg = start_angle_raw / 100.0
        end_angle_deg = end_angle_raw / 100.0

        angle_span = end_angle_deg - start_angle_deg

        if angle_span < -180.0:
            angle_span += 360.0
        elif angle_span > 180.0:
            angle_span -= 360.0

        point_count = 12
        angle_step = angle_span / float(point_count - 1)

        now = time.time()

        with self.data_lock:
            for i in range(point_count):
                offset = 6 + i * 3

                distance_mm = struct.unpack_from('<H', packet, offset)[0]
                confidence = packet[offset + 2]

                if confidence < self.min_confidence:
                    continue

                if distance_mm <= 0:
                    continue

                distance_m = distance_mm / 1000.0

                if distance_m < self.range_min_m or distance_m > self.range_max_m:
                    continue

                lidar_angle_deg = start_angle_deg + angle_step * i

                # Переводимо кут лідара у кут робота.
                # Після цього 0 градусів має бути напрямком вперед.
                robot_angle_deg = lidar_angle_deg + self.angle_offset_deg
                robot_angle_deg = self.normalize_0_360(robot_angle_deg)

                idx = self.angle_to_bin(robot_angle_deg)

                # Якщо у цей bin прилетіло кілька точок — беремо ближчу.
                old_range = self.ranges[idx]
                if math.isinf(old_range) or distance_m < old_range:
                    self.ranges[idx] = distance_m
                    self.intensities[idx] = float(confidence)
                    self.last_update_time[idx] = now

    # -------------------------------------------------------------------------
    # Zone processing
    # -------------------------------------------------------------------------

    def get_zone_min_distance(self, ranges_snapshot, zone_min_deg, zone_max_deg):
        values = []

        for idx, distance in enumerate(ranges_snapshot):
            if math.isinf(distance) or math.isnan(distance):
                continue

            angle_0_360 = idx * self.angle_resolution_deg
            angle_robot = self.normalize_minus_180_180(angle_0_360)

            if self.angle_in_zone(angle_robot, zone_min_deg, zone_max_deg):
                values.append(distance)

        if not values:
            return math.inf

        return min(values)

    def compute_zones(self, ranges_snapshot):
        front_min = float(self.get_parameter('front_min_deg').value)
        front_max = float(self.get_parameter('front_max_deg').value)

        fl_min = float(self.get_parameter('front_left_min_deg').value)
        fl_max = float(self.get_parameter('front_left_max_deg').value)

        fr_min = float(self.get_parameter('front_right_min_deg').value)
        fr_max = float(self.get_parameter('front_right_max_deg').value)

        left_min = float(self.get_parameter('left_min_deg').value)
        left_max = float(self.get_parameter('left_max_deg').value)

        right_min = float(self.get_parameter('right_min_deg').value)
        right_max = float(self.get_parameter('right_max_deg').value)

        rear_min = float(self.get_parameter('rear_min_deg').value)
        rear_max = float(self.get_parameter('rear_max_deg').value)

        front = self.get_zone_min_distance(ranges_snapshot, front_min, front_max)
        front_left = self.get_zone_min_distance(ranges_snapshot, fl_min, fl_max)
        front_right = self.get_zone_min_distance(ranges_snapshot, fr_min, fr_max)
        left = self.get_zone_min_distance(ranges_snapshot, left_min, left_max)
        right = self.get_zone_min_distance(ranges_snapshot, right_min, right_max)
        rear = self.get_zone_min_distance(ranges_snapshot, rear_min, rear_max)

        return [front, front_left, front_right, left, right, rear]

    # -------------------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------------------

    def publish_scan_and_zones(self):
        now_ros = self.get_clock().now().to_msg()
        now_sec = time.time()

        with self.data_lock:
            ranges_snapshot = list(self.ranges)
            intensities_snapshot = list(self.intensities)
            update_snapshot = list(self.last_update_time)

            # Старі точки прибираємо, щоб робот не бачив “привиди”.
            max_age_sec = 0.5
            for i in range(self.bin_count):
                if update_snapshot[i] <= 0.0:
                    ranges_snapshot[i] = math.inf
                    intensities_snapshot[i] = 0.0
                elif now_sec - update_snapshot[i] > max_age_sec:
                    ranges_snapshot[i] = math.inf
                    intensities_snapshot[i] = 0.0

            # Очищаємо головний буфер після публікації.
            # Так кожен LaserScan буде актуальним, а не накопиченим.
            self.ranges = [math.inf] * self.bin_count
            self.intensities = [0.0] * self.bin_count
            self.last_update_time = [0.0] * self.bin_count

        scan_msg = LaserScan()
        scan_msg.header.stamp = now_ros
        scan_msg.header.frame_id = self.frame_id

        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi
        scan_msg.angle_increment = math.radians(self.angle_resolution_deg)

        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / max(self.publish_rate_hz, 1.0)

        scan_msg.range_min = self.range_min_m
        scan_msg.range_max = self.range_max_m

        scan_msg.ranges = ranges_snapshot
        scan_msg.intensities = intensities_snapshot

        self.scan_pub.publish(scan_msg)

        zones = self.compute_zones(ranges_snapshot)

        zones_msg = Float32MultiArray()
        zones_msg.data = [
            float(x) if not math.isinf(x) else -1.0
            for x in zones
        ]
        self.zones_pub.publish(zones_msg)

        blocked_msg = Int32MultiArray()
        blocked_msg.data = [
            1 if zone_distance != math.inf and zone_distance <= self.stop_distance_m else 0
            for zone_distance in zones
        ]
        self.blocked_pub.publish(blocked_msg)

        self.log_zones_throttled(zones, blocked_msg.data)

    def log_zones_throttled(self, zones, blocked):
        now = time.time()

        if not hasattr(self, '_last_zone_log'):
            self._last_zone_log = 0.0

        if now - self._last_zone_log < 1.0:
            return

        self._last_zone_log = now

        def fmt(x):
            if math.isinf(x):
                return '---'
            return f'{x:.3f}'

        self.get_logger().info(
            'zones m: '
            f'front={fmt(zones[0])}, '
            f'front_left={fmt(zones[1])}, '
            f'front_right={fmt(zones[2])}, '
            f'left={fmt(zones[3])}, '
            f'right={fmt(zones[4])}, '
            f'rear={fmt(zones[5])} | '
            f'blocked={blocked}'
        )

    # -------------------------------------------------------------------------

    def destroy_node(self):
        self.running = False

        try:
            if hasattr(self, 'serial') and self.serial is not None:
                self.serial.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LD06Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
