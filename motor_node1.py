import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio
import time


class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_l298n')

        # ===== PARAMETERS =====
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('pwm_frequency', 1000)

        self.max_linear = self.get_parameter(
            'max_linear_speed').value
        self.max_angular = self.get_parameter(
            'max_angular_speed').value
        self.pwm_freq = self.get_parameter(
            'pwm_frequency').value

        # ===== GPIO PINS (BCM) =====
        self.in1 = 17
        self.in2 = 27
        self.in3 = 26
        self.in4 = 19
        self.en_a = 22
        self.en_b = 13

        # ===== GPIO CHIP =====
        self.chip = lgpio.gpiochip_open(0)

        for pin in [self.in1, self.in2, self.in3, self.in4,
                    self.en_a, self.en_b]:
            lgpio.gpio_claim_output(self.chip, pin, 0)

        # PWM
        lgpio.tx_pwm(self.chip, self.en_a, self.pwm_freq, 0)
        lgpio.tx_pwm(self.chip, self.en_b, self.pwm_freq, 0)

        self.stop_motors()

        self.get_logger().info("L298N initialized via lgpio")

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_check)

    # ==================================================

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = max(
            -self.max_linear,
            min(self.max_linear, msg.linear.x)
        )
        angular = max(
            -self.max_angular,
            min(self.max_angular, msg.angular.z)
        )

        left = linear - angular
        right = linear + angular

        left_pwm = min(abs(left) / self.max_linear * 100, 100)
        right_pwm = min(abs(right) / self.max_linear * 100, 100)

        # LEFT
        lgpio.gpio_write(self.chip, self.in1, int(left >= 0))
        lgpio.gpio_write(self.chip, self.in2, int(left < 0))

        # RIGHT
        lgpio.gpio_write(self.chip, self.in3, int(right >= 0))
        lgpio.gpio_write(self.chip, self.in4, int(right < 0))

        lgpio.tx_pwm(self.chip, self.en_a, self.pwm_freq, left_pwm)
        lgpio.tx_pwm(self.chip, self.en_b, self.pwm_freq, right_pwm)

    # ==================================================

    def watchdog_check(self):
        dt = (self.get_clock().now() -
              self.last_cmd_time).nanoseconds / 1e9

        if dt > 0.5:
            self.stop_motors()

    def stop_motors(self):
        for pin in [self.in1, self.in2, self.in3, self.in4]:
            lgpio.gpio_write(self.chip, pin, 0)

        lgpio.tx_pwm(self.chip, self.en_a, self.pwm_freq, 0)
        lgpio.tx_pwm(self.chip, self.en_b, self.pwm_freq, 0)

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
