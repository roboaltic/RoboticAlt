import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio

GPIO_CHIP = 4   # !!! номер из gpioinfo

ENA = 22
IN1 = 17
IN2 = 27
ENB = 13
IN3 = 26
IN4 = 19


class DiffDriveL298N(Node):
    def __init__(self):
        super().__init__('diff_drive_l298n')

        self.chip = lgpio.gpiochip_open(GPIO_CHIP)

        for pin in [ENA, IN1, IN2, ENB, IN3, IN4]:
            lgpio.gpio_claim_output(self.chip, pin)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("GPIO via initialized")

    def cmd_vel_callback(self, msg):
        speed = msg.linear.x

        if speed > 0:
            self.set_motor(1, True)
            self.set_motor(2, True)
        elif speed < 0:
            self.set_motor(1, False)
            self.set_motor(2, False)
        else:
            self.stop_motors()

    def set_motor(self, motor, forward):
        if motor == 1:
            lgpio.gpio_write(self.chip, IN1, int(forward))
            lgpio.gpio_write(self.chip, IN2, int(not forward))
            lgpio.gpio_write(self.chip, ENA, 1)
        else:
            lgpio.gpio_write(self.chip, IN3, int(forward))
            lgpio.gpio_write(self.chip, IN4, int(not forward))
            lgpio.gpio_write(self.chip, ENB, 1)

    def stop_motors(self):
        for pin in [ENA, ENB, IN1, IN2, IN3, IN4]:
            lgpio.gpio_write(self.chip, pin, 0)

    def destroy_node(self):
        self.stop_motors()
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()


def main():
    rclpy.init()
    node = DiffDriveL298N()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
