import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarSafetyNode(Node):
    def __init__(self):
        super().__init__('lidar_safety')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')

        self.declare_parameter('stop_dist', 0.20)

        # True = перевірка всього 360°
        # False = перевірка тільки передньої зони
        self.declare_parameter('use_full_scan', False)

        # Центр передньої зони
        self.declare_parameter('front_center_deg', 0.0)

        # Ширина передньої зони в один бік
        self.declare_parameter('front_angle_deg', 45.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value

        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.use_full_scan = bool(self.get_parameter('use_full_scan').value)
        self.front_center_deg = float(self.get_parameter('front_center_deg').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)

        self.front_dist = float('inf')
        self.have_scan = False
        self.points_in_zone = 0

        self.last_cmd = Twist()
        self.have_cmd = False

        self.last_debug_log_time = 0.0
        self.last_cmd_log_time = 0.0

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.create_subscription(
            Twist,
            self.input_cmd_topic,
            self.cmd_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_cmd_topic,
            10
        )

        # 20 Hz safety loop
        self.safety_timer = self.create_timer(0.05, self.safety_loop)

        self.get_logger().info(
            f'Lidar safety started: {self.input_cmd_topic} -> {self.output_cmd_topic}'
        )
        self.get_logger().info(
            f'stop_dist={self.stop_dist:.3f}, '
            f'use_full_scan={self.use_full_scan}, '
            f'front_center_deg={self.front_center_deg:.1f}, '
            f'front_angle_deg={self.front_angle_deg:.1f}'
        )

    def normalize_angle_deg(self, angle_deg):
        return angle_deg % 360.0

    def angle_in_front_zone(self, angle_deg):
        angle_deg = self.normalize_angle_deg(angle_deg)
        center = self.normalize_angle_deg(self.front_center_deg)

        diff = (angle_deg - center + 180.0) % 360.0 - 180.0
        return abs(diff) <= self.front_angle_deg

    def scan_callback(self, msg):
        values = []
        angle = msg.angle_min

        for r in msg.ranges:
            angle_deg = math.degrees(angle)

            if self.use_full_scan:
                angle_ok = True
            else:
                angle_ok = self.angle_in_front_zone(angle_deg)

            if angle_ok and math.isfinite(r) and msg.range_min < r < msg.range_max:
                values.append(r)

            angle += msg.angle_increment

        self.points_in_zone = len(values)

        if values:
            values.sort()

            # 10% точка, щоб не брати випадковий шум
            idx = max(0, min(len(values) - 1, int(len(values) * 0.1)))
            self.front_dist = values[idx]
            self.have_scan = True
        else:
            self.front_dist = float('inf')
            self.have_scan = False

        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_debug_log_time > 0.5:
            self.get_logger().info(
                f'SAFETY DIST | front={self.front_dist:.3f} m | '
                f'stop={self.stop_dist:.3f} m | '
                f'points={self.points_in_zone} | '
                f'have_scan={self.have_scan} | '
                f'use_full_scan={self.use_full_scan} | '
                f'center={self.front_center_deg:.1f} | '
                f'angle={self.front_angle_deg:.1f}'
            )
            self.last_debug_log_time = now

    def cmd_callback(self, cmd):
        self.last_cmd = cmd
        self.have_cmd = True

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_cmd_log_time > 0.3:
            self.get_logger().info(
                f'SAFETY CMD | raw_x={cmd.linear.x:.3f} | '
                f'raw_z={cmd.angular.z:.3f} | '
                f'front={self.front_dist:.3f} m | '
                f'stop={self.stop_dist:.3f} m | '
                f'obstacle={self.obstacle_ahead()} | '
                f'wants_forward={self.wants_forward(cmd)}'
            )
            self.last_cmd_log_time = now

        self.publish_safe_cmd()

    def obstacle_ahead(self):
        return self.have_scan and self.front_dist < self.stop_dist

    def wants_forward(self, cmd):
        return cmd.linear.x > 0.0

    def publish_safe_cmd(self):
        if not self.have_cmd:
            return

        if self.obstacle_ahead() and self.wants_forward(self.last_cmd):
            safe_cmd = Twist()

            self.get_logger().warn(
                f'SAFETY STOP | front={self.front_dist:.3f} m '
                f'< stop={self.stop_dist:.3f} m'
            )
        else:
            safe_cmd = self.last_cmd

        self.cmd_pub.publish(safe_cmd)

    def safety_loop(self):
        self.publish_safe_cmd()

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
