import rclpy
from rclpy.node import Node

from robot_msgs.msg import DriveCommand
from geometry_msgs.msg import Twist


class SmartDriver(Node):

    def __init__(self):
        super().__init__('smart_driver')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.sub = self.create_subscription(
            DriveCommand,
            '/drive_cmd',
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.update)

        self.speed = 0.0
        self.turn = 0.0

        self.target_time = 0.0
        self.start_time = None
        self.active = False

        self.get_logger().info("✅ SmartDriver ready")

    # ==========================

    def cmd_callback(self, msg):

        self.speed = msg.speed
        self.turn = msg.turn

        if msg.speed != 0.0:
            self.target_time = abs(msg.distance / msg.speed)
        else:
            self.target_time = 0.0

        self.start_time = self.get_clock().now()
        self.active = True

        self.get_logger().info(
            f"CMD: v={self.speed}, d={msg.distance}, t={self.target_time:.2f}"
        )

    # ==========================

    def update(self):

        twist = Twist()

        if not self.active:
            self.pub.publish(twist)
            return

        now = self.get_clock().now()
        dt = (now - self.start_time).nanoseconds / 1e9

        if self.target_time > 0 and dt >= self.target_time:
            self.stop()
            return

        twist.linear.x = self.speed
        twist.angular.z = self.turn

        self.pub.publish(twist)

    # ==========================

    def stop(self):

        self.active = False

        twist = Twist()
        self.pub.publish(twist)

        self.get_logger().info("🛑 Finished move")


def main():

    rclpy.init()

    node = SmartDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
