import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import lgpio


class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_l298n')

        # ================= PARAMETERS =================

        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('max_pwm', 80.0)

        self.max_linear = self.get_parameter(
            'max_linear_speed').value

        self.max_angular = self.get_parameter(
            'max_angular_speed').value

        self.pwm_freq = self.get_parameter(
            'pwm_frequency').value

        self.max_pwm = self.get_parameter(
            'max_pwm').value


        # ================= GPIO =================

        self.in1 = 17
        self.in2 = 27
        self.in3 = 26
        self.in4 = 19
        self.en_a = 22
        self.en_b = 13

        self.chip = lgpio.gpiochip_open(0)

        for p in [
            self.in1, self.in2, self.in3, self.in4,
            self.en_a, self.en_b
        ]:
            lgpio.gpio_claim_output(self.chip, p, 0)

        self.stop_motors()


        # ================= ROS =================

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )

        # основной цикл управления (фон)
        self.control_timer = self.create_timer(
            0.02,   # 50 Hz
            self.control_loop
        )


        # ================= STATE =================

        self.target_v = 0.0
        self.target_w = 0.0

        self.last_cmd_time = self.get_clock().now()


        self.get_logger().info(
            "✅ DiffDrive L298N started (NO odometry)"
        )


    # =========================================================

    def cmd_vel_cb(self, msg):

        self.target_v = max(
            min(msg.linear.x, self.max_linear),
            -self.max_linear
        )

        self.target_w = max(
            min(msg.angular.z, self.max_angular),
            -self.max_angular
        )

        self.last_cmd_time = self.get_clock().now()


    # =========================================================

    def control_loop(self):

        # ---- Watchdog (антизалипание) ----

        dt = (
            self.get_clock().now() -
            self.last_cmd_time
        ).nanoseconds / 1e9

        if dt > 0.5:
            self.stop_motors()
            return


        # ---- Нормализация ----

        v = self.target_v / self.max_linear
        w = self.target_w / self.max_angular

        v = max(min(v, 1.0), -1.0)
        w = max(min(w, 1.0), -1.0)


        # ---- Дифференциальный привод ----

        left = v - w
        right = v + w

        left = max(min(left, 1.0), -1.0)
        right = max(min(right, 1.0), -1.0)


        # ---- Моторы ----

        self.set_motor(self.in1, self.in2, self.en_a, left)
        self.set_motor(self.in3, self.in4, self.en_b, right)


    # =========================================================

    def set_motor(self, in1, in2, en, value):

        # deadzone
        if abs(value) < 0.03:
            lgpio.gpio_write(self.chip, in1, 0)
            lgpio.gpio_write(self.chip, in2, 0)
            lgpio.tx_pwm(self.chip, en, self.pwm_freq, 0)
            return


        # направление
        if value > 0:
            lgpio.gpio_write(self.chip, in1, 1)
            lgpio.gpio_write(self.chip, in2, 0)
        else:
            lgpio.gpio_write(self.chip, in1, 0)
            lgpio.gpio_write(self.chip, in2, 1)


        # PWM
        duty = abs(value) * self.max_pwm

        lgpio.tx_pwm(self.chip, en, self.pwm_freq, duty)


    # =========================================================

    def stop_motors(self):

        for p in [
            self.in1, self.in2,
            self.in3, self.in4
        ]:
            lgpio.gpio_write(self.chip, p, 0)

        lgpio.tx_pwm(self.chip, self.en_a, self.pwm_freq, 0)
        lgpio.tx_pwm(self.chip, self.en_b, self.pwm_freq, 0)


    # =========================================================

    def destroy_node(self):

        self.stop_motors()
        lgpio.gpiochip_close(self.chip)

        super().destroy_node()


def main():

    rclpy.init()

    node = DiffDriveL298N()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
