import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio
from time import time


class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_l298n')

        # ===== GPIO PINS (BCM) =====
        # Left motor
        self.IN1 = 17
        self.IN2 = 27
        self.ENA = 22  # PWM

        # Right motor
        self.IN3 = 26
        self.IN4 = 19
        self.ENB = 13  # PWM

        # ===== PWM CONFIG =====
        self.PWM_FREQ = 1000        # Hz
        self.MAX_DUTY = 80.0        # % (потолок мощности)

        # ===== WATCHDOG CONFIG =====
        self.CMD_TIMEOUT = 0.5      # секунд без cmd_vel → стоп
        self.last_cmd_time = time()

        # ===== GPIO INIT =====
        self.chip = lgpio.gpiochip_open(0)

        for pin in [self.IN1, self.IN2, self.IN3, self.IN4]:
            lgpio.gpio_claim_output(self.chip, pin)

        # Остановить моторы на старте
        self.stop_motors()

        # ===== ROS INTERFACES =====
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Watchdog timer
        self.watchdog_timer = self.create_timer(
            0.1,  # период проверки
            self.watchdog_callback
        )

        self.get_logger().info("✅ DiffDrive L298N node started with PWM + watchdog")

    # =====================================================

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time()

        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive model
        left = linear - angular
        right = linear + angular

        # Clamp to [-1.0, 1.0]
        left = max(min(left, 1.0), -1.0)
        right = max(min(right, 1.0), -1.0)

        self.set_motor(self.IN1, self.IN2, self.ENA, left)
        self.set_motor(self.IN3, self.IN4, self.ENB, right)

    # =====================================================

    def watchdog_callback(self):
        if time() - self.last_cmd_time > self.CMD_TIMEOUT:
            self.stop_motors()

    # =====================================================

    def set_motor(self, in1, in2, en, value):
        if value > 0.0:
            lgpio.gpio_write(self.chip, in1, 1)
            lgpio.gpio_write(self.chip, in2, 0)
        elif value < 0.0:
            lgpio.gpio_write(self.chip, in1, 0)
            lgpio.gpio_write(self.chip, in2, 1)
        else:
            lgpio.gpio_write(self.chip, in1, 0)
            lgpio.gpio_write(self.chip, in2, 0)
            lgpio.tx_pwm(self.chip, en, self.PWM_FREQ, 0)
            return

        duty = abs(value) * self.MAX_DUTY
        lgpio.tx_pwm(self.chip, en, self.PWM_FREQ, duty)

    # =====================================================

    def stop_motors(self):
        for pin in [self.IN1, self.IN2, self.IN3, self.IN4]:
            lgpio.gpio_write(self.chip, pin, 0)

        lgpio.tx_pwm(self.chip, self.ENA, self.PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, self.ENB, self.PWM_FREQ, 0)

    # =====================================================

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
