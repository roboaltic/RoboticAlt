import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio
import math


# ===== GPIO CONFIG =====
GPIO_CHIP = 4

ENA = 22   # PWM LEFT
IN1 = 17
IN2 = 27

ENB = 13   # PWM RIGHT
IN3 = 26
IN4 = 19


# ===== ROBOT PARAMS =====
WHEEL_BASE = 0.09      # расстояние между колесами (м)
MAX_SPEED = 0.6        # м/с (условно)
PWM_FREQ = 1000        # Hz


class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_l298n')

        # Open GPIO
        self.chip = lgpio.gpiochip_open(GPIO_CHIP)

        # Setup pins
        for pin in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_claim_output(self.chip, pin)

        lgpio.gpio_claim_output(self.chip, ENA)
        lgpio.gpio_claim_output(self.chip, ENB)

        # Init PWM
        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

        # ROS sub
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("✅ Diff Drive initialized")


    # =============================
    # CMD_VEL CALLBACK
    # =============================
    def cmd_vel_callback(self, msg):

        v = msg.linear.x
        w = msg.angular.z

        # Differential kinematics
        v_left  = v - (w * WHEEL_BASE / 2.0)
        v_right = v + (w * WHEEL_BASE / 2.0)

        # Normalize
        v_left  = self.limit_speed(v_left)
        v_right = self.limit_speed(v_right)

        # Apply motors
        self.set_motor_left(v_left)
        self.set_motor_right(v_right)


    # =============================
    # SPEED LIMIT
    # =============================
    def limit_speed(self, v):

        if v > MAX_SPEED:
            v = MAX_SPEED
        elif v < -MAX_SPEED:
            v = -MAX_SPEED

        return v


    # =============================
    # LEFT MOTOR
    # =============================
    def set_motor_left(self, speed):

        forward = speed >= 0
        duty = abs(speed) / MAX_SPEED * 100.0

        lgpio.gpio_write(self.chip, IN1, int(forward))
        lgpio.gpio_write(self.chip, IN2, int(not forward))

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)


    # =============================
    # RIGHT MOTOR
    # =============================
    def set_motor_right(self, speed):

        forward = speed >= 0
        duty = abs(speed) / MAX_SPEED * 100.0

        lgpio.gpio_write(self.chip, IN3, int(forward))
        lgpio.gpio_write(self.chip, IN4, int(not forward))

        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)


    # =============================
    # STOP
    # =============================
    def stop(self):

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

        for pin in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_write(self.chip, pin, 0)


    def destroy_node(self):

        self.stop()
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()


# =============================
# MAIN
# =============================
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
