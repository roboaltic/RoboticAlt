import rclpy
from rclpy.node import Node

import serial
import math

from sensor_msgs.msg import LaserScan


class LD06Node(Node):
    def __init__(self):
        super().__init__('ld06_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser')

        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 8.0)

        # 360 точок = 1 градус на точку
        self.declare_parameter('scan_points', 360)

        # 0.05 = 20 Hz
        self.declare_parameter('publish_period', 0.05)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)

        self.frame_id = self.get_parameter('frame_id').value
        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)

        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.scan_points = int(self.get_parameter('scan_points').value)
        self.publish_period = float(self.get_parameter('publish_period').value)

        self.ser = serial.Serial(port, baud, timeout=0)

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.buffer = bytearray()

        self.ranges = [float('inf')] * self.scan_points
        self.last_publish_points = 0

        self.read_timer = self.create_timer(0.005, self.read_data)
        self.publish_timer = self.create_timer(self.publish_period, self.publish_scan)

        self.get_logger().info(f'LD06 started on {port} @ {baud}')
        self.get_logger().info(
            f'Publishing rolling LaserScan: points={self.scan_points}, '
            f'period={self.publish_period}s'
        )

    def read_data(self):
        data = self.ser.read(1024)

        if data:
            self.buffer.extend(data)
            self.parse_packets()

    def parse_packets(self):
        PACKET_SIZE = 47

        while len(self.buffer) >= PACKET_SIZE:
            if self.buffer[0] != 0x54:
                self.buffer.pop(0)
                continue

            if self.buffer[1] != 0x2C:
                self.buffer.pop(0)
                continue

            packet = self.buffer[:PACKET_SIZE]
            del self.buffer[:PACKET_SIZE]

            self.process_packet(packet)

    def process_packet(self, packet):
        POINTS = 12

        start_angle = ((packet[5] << 8) | packet[4]) / 100.0
        end_angle = ((packet[43] << 8) | packet[42]) / 100.0

        if end_angle < start_angle:
            end_angle += 360.0

        step = (end_angle - start_angle) / (POINTS - 1)

        for i in range(POINTS):
            offset = 6 + i * 3

            dist_mm = packet[offset] | (packet[offset + 1] << 8)
            dist_m = dist_mm / 1000.0

            if not (self.range_min < dist_m < self.range_max):
                continue

            angle_deg = start_angle + i * step
            angle_deg = (angle_deg + self.angle_offset_deg) % 360.0

            # Переводимо 0..360 у індекс масиву
            index = int((angle_deg / 360.0) * self.scan_points) % self.scan_points

            old = self.ranges[index]

            # Якщо в один сектор попало кілька точок — беремо ближчу
            if math.isinf(old) or dist_m < old:
                self.ranges[index] = dist_m

    def publish_scan(self):
        valid_points = sum(1 for r in self.ranges if math.isfinite(r))

        if valid_points < 10:
            self.get_logger().warn(
                f'Not enough valid lidar points: {valid_points}'
            )
            return

        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # ВАЖЛИВО:
        # Це повний 360° scan у форматі -180° ... +180°
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / self.scan_points

        msg.time_increment = 0.0
        msg.scan_time = self.publish_period

        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Переставляємо масив так, щоб:
        # index 0   = -180°
        # index 180 = 0° / перед
        # index 359 = +179°
        half = self.scan_points // 2
        msg.ranges = self.ranges[half:] + self.ranges[:half]

        self.publisher.publish(msg)

        self.get_logger().info(
            f'/scan published: valid_points={valid_points}, '
            f'angle_min=-180, angle_max=180'
        )

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

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
