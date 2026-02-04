import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio

# ===== GPIO CONFIG =====
GPIO_CHIP = 4
ENA = 22   # PWM LEFT
IN1 = 17
IN2 = 27
ENB = 13   # PWM RIGHT
IN3 = 26
IN4 = 19

# ===== ROBOT PARAMS =====
WHEEL_BASE = 0.09
MAX_SPEED = 0.3
PWM_FREQ = 1000

class DiffDriveL298N(Node):
    def __init__(self):
        super().__init__('diff_drive_l298n')

        self.chip = lgpio.gpiochip_open(GPIO_CHIP)
        for pin in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_claim_output(self.chip, pin)
        lgpio.gpio_claim_output(self.chip, ENA)
        lgpio.gpio_claim_output(self.chip, ENB)

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

        self.last_left = 0.0
        self.last_right = 0.0

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("✅ DiffDriveL298N initialized")

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_left  = self.limit_speed(v - w * WHEEL_BASE / 2)
        v_right = self.limit_speed(v + w * WHEEL_BASE / 2)

        # публикуем только если изменилось
        if abs(v_left - self.last_left) > 0.01 or abs(v_right - self.last_right) > 0.01:
            self.set_motor_left(v_left)
            self.set_motor_right(v_right)
            self.last_left = v_left
            self.last_right = v_right
            self.get_logger().info(f"cmd_vel updated: left={v_left:.2f}, right={v_right:.2f}")

    def limit_speed(self, v):
        return max(min(v, MAX_SPEED), -MAX_SPEED)

    def set_motor_left(self, speed):
        forward = speed >= 0
        duty = abs(speed) / MAX_SPEED * 100.0
        lgpio.gpio_write(self.chip, IN1, int(forward))
        lgpio.gpio_write(self.chip, IN2, int(not forward))
        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)

    def set_motor_right(self, speed):
        forward = speed >= 0
        duty = abs(speed) / MAX_SPEED * 100.0
        lgpio.gpio_write(self.chip, IN3, int(forward))
        lgpio.gpio_write(self.chip, IN4, int(not forward))
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)

    def stop(self):
        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)
        for pin in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_write(self.chip, pin, 0)

    def destroy_node(self):
        self.stop()
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
