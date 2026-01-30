import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio

IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.sub = self.create_subscription(
            String,
            'motor_cmd',
            self.cmd_callback,
            10
        )

        self.h = lgpio.gpiochip_open(0)
        for pin in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_claim_output(self.h, pin)

        self.get_logger().info('Motor GPIO node started')

    def cmd_callback(self, msg):
        cmd = msg.data

        if cmd == 'forward':
            lgpio.gpio_write(self.h, IN1, 1)
            lgpio.gpio_write(self.h, IN2, 0)
            lgpio.gpio_write(self.h, IN3, 1)
            lgpio.gpio_write(self.h, IN4, 0)

        elif cmd == 'stop':
            for pin in [IN1, IN2, IN3, IN4]:
                lgpio.gpio_write(self.h, pin, 0)

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
