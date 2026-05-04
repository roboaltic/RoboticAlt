import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSectorTest(Node):
    def __init__(self):
        super().__init__('lidar_sector_test')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('log_period', 0.5)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.log_period = float(self.get_parameter('log_period').value)

        self.last_log_time = 0.0

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.get_logger().info(
            f'Lidar sector test started. Listening: {self.scan_topic}'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def normalize_angle_deg(self, angle_deg):
        return angle_deg % 360.0

    def angle_in_sector(self, angle_deg, start_deg, end_deg):
        angle_deg = self.normalize_angle_deg(angle_deg)
        start_deg = self.normalize_angle_deg(start_deg)
        end_deg = self.normalize_angle_deg(end_deg)

        if start_deg <= end_deg:
            return start_deg <= angle_deg <= end_deg
        else:
            return angle_deg >= start_deg or angle_deg <= end_deg

    def min_valid_distance(self, values):
        if not values:
            return None

        values.sort()
        idx = max(0, min(len(values) - 1, int(len(values) * 0.1)))
        return values[idx]

    def scan_callback(self, msg: LaserScan):
        now = self.now_sec()

        if now - self.last_log_time < self.log_period:
            return

        self.last_log_time = now

        sectors = {
            'FRONT 315..45': [],
            'LEFT 45..135': [],
            'BACK 135..225': [],
            'RIGHT 225..315': [],
        }

        angle = msg.angle_min

        total_valid = 0

        for r in msg.ranges:
            angle_deg = math.degrees(angle)
            angle_deg = self.normalize_angle_deg(angle_deg)

            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                total_valid += 1

                if self.angle_in_sector(angle_deg, 315.0, 45.0):
                    sectors['FRONT 315..45'].append(r)
                elif self.angle_in_sector(angle_deg, 45.0, 135.0):
                    sectors['LEFT 45..135'].append(r)
                elif self.angle_in_sector(angle_deg, 135.0, 225.0):
                    sectors['BACK 135..225'].append(r)
                elif self.angle_in_sector(angle_deg, 225.0, 315.0):
                    sectors['RIGHT 225..315'].append(r)

            angle += msg.angle_increment

        self.get_logger().info(
            f'\n'
            f'===== LIDAR SECTORS =====\n'
            f'angle_min={math.degrees(msg.angle_min):.1f} deg | '
            f'angle_max={math.degrees(msg.angle_max):.1f} deg | '
            f'points={len(msg.ranges)} | valid={total_valid}\n'
            f'FRONT: {self.format_sector(sectors["FRONT 315..45"])}\n'
            f'LEFT : {self.format_sector(sectors["LEFT 45..135"])}\n'
            f'BACK : {self.format_sector(sectors["BACK 135..225"])}\n'
            f'RIGHT: {self.format_sector(sectors["RIGHT 225..315"])}'
        )

    def format_sector(self, values):
        if not values:
            return 'NO DATA'

        d = self.min_valid_distance(values)

        return f'min={d:.3f} m | points={len(values)}'


def main(args=None):
    rclpy.init(args=args)
    node = LidarSectorTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
