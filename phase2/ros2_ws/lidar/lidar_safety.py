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
        self.declare_parameter('use_full_scan', True)
        self.declare_parameter('front_angle_deg', 60.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value

        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.use_full_scan = bool(self.get_parameter('use_full_scan').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)

        self.front_dist = float('inf')
        self.have_scan = False

        self.last_cmd = Twist()
        self.have_cmd = False

        self.last_log_time = 0.0

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Twist, self.input_cmd_topic, self.cmd_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)

        # ВАЖЛИВО: safety тепер сам регулярно перевіряє стан
        self.timer = self.create_timer(0.05, self.safety_loop)

        self.get_logger().info(
            f'Lidar safety started: {self.input_cmd_topic} -> {self.output_cmd_topic}, '
            f'stop_dist={self.stop_dist}, use_full_scan={self.use_full_scan}, '
            f'front_angle_deg={self.front_angle_deg}'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def scan_callback(self, msg):
        values = []
        angle = msg.angle_min

        for r in msg.ranges:
            angle_deg = math.degrees(angle)

            if self.use_full_scan:
                angle_ok = True
            else:
                angle_ok = -self.front_angle_deg <= angle_deg <= self.front_angle_deg

            if angle_ok:
                if math.isfinite(r) and msg.range_min < r < msg.range_max:
                    values.append(r)

            angle += msg.angle_increment

        if values:
            values.sort()
            idx = max(0, min(len(values) - 1, int(len(values) * 0.1)))
            self.front_dist = values[idx]
            self.have_scan = True
        else:
            self.front_dist = float('inf')
            self.have_scan = False

        now = self.now_sec()
        if now - self.last_log_time > 0.5:
            self.get_logger().info(
                f'scan: front_dist={self.front_dist:.3f}, '
                f'stop_dist={self.stop_dist:.3f}, '
                f'have_scan={self.have_scan}, '
                f'points={len(values)}'
            )
            self.last_log_time = now

    def cmd_callback(self, cmd):
        self.last_cmd = cmd
        self.have_cmd = True
        self.publish_safe_cmd()

    def obstacle_ahead(self):
        return self.have_scan and self.front_dist < self.stop_dist

    def wants_forward(self, cmd):
        # Блокуємо рух вперед.
        # Якщо хочеш блокувати ще й поворот на місці — додамо angular.z.
        return cmd.linear.x > 0.0

    def publish_safe_cmd(self):
        if not self.have_cmd:
            return

        if self.obstacle_ahead() and self.wants_forward(self.last_cmd):
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)

            self.get_logger().warn(
                f'SAFETY STOP: front_dist={self.front_dist:.3f} < stop_dist={self.stop_dist:.3f}'
            )
        else:
            self.cmd_pub.publish(self.last_cmd)

    def safety_loop(self):
        # Навіть якщо нових команд з keyboard немає,
        # при появі перешкоди safety сам відправить stop.
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
