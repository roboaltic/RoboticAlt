import rclpy
from rclpy.node import Node

import serial
import math

from sensor_msgs.msg import LaserScan


class LD06Node(Node):
    def __init__(self):
        super().__init__('ld06_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400')
        self.declare_parameter('frame_id', 'laser')

        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 8.0)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)

        self.frame_id = self.get_parameter('frame_id').value
        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)

        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.ser = serial.Serial(port, baud, timeout=0)

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.buffer = bytearray()

        # 360 точок: index 0 = 0°, index 90 = 90°, index 180 = 180°, index 270 = 270°
        self.ranges_360 = [float('inf')] * 360
        self.last_angle_deg = None
        self.points_in_rotation = 0

        self.get_logger().info(f'LD06 started on {port} @ {baud}')
        self.get_logger().info('Publishing full 360 LaserScan with 1 degree resolution')

        self.timer = self.create_timer(0.005, self.read_data)

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

        raw_start = start_angle
        raw_end = end_angle

        if end_angle < start_angle:
            end_angle += 360.0

        step = (end_angle - start_angle) / (POINTS - 1)

        for i in range(POINTS):
            offset = 6 + i * 3

            dist_mm = packet[offset] | (packet[offset + 1] << 8)
            dist_m = dist_mm / 1000.0

            angle_deg = start_angle + i * step
            angle_deg = angle_deg % 360.0

            # Корекція орієнтації лідара відносно робота
            angle_deg = (angle_deg + self.angle_offset_deg) % 360.0

            # Якщо пройшли через 360 -> 0, значить завершився один оберт
            if self.last_angle_deg is not None:
                if self.last_angle_deg > 300.0 and angle_deg < 60.0:
                    self.publish_full_scan()
                    self.ranges_360 = [float('inf')] * 360
                    self.points_in_rotation = 0

            self.last_angle_deg = angle_deg

            if self.range_min < dist_m < self.range_max:
                index = int(round(angle_deg)) % 360

                old = self.ranges_360[index]

                # Якщо в цей градус попало кілька точок — беремо ближчу
                if math.isinf(old) or dist_m < old:
                    self.ranges_360[index] = dist_m
                    self.points_in_rotation += 1

    def publish_full_scan(self):
        if self.points_in_rotation < 30:
            return

        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = math.radians(1.0)

        msg.time_increment = 0.0
        msg.scan_time = 0.1

        msg.range_min = self.range_min
        msg.range_max = self.range_max

        msg.ranges = self.ranges_360

        self.publisher.publish(msg)

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
