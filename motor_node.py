import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# ===== GPIO CONFIG =====
# Left motor
IN1 = 17
IN2 = 27
ENA = 22  # PWM

# Right motor
IN3 = 26
IN4 = 19
ENB = 13  # PWM

PWM_FREQ = 1000
MAX_DUTY = 100


class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_l298n')

        GPIO.setmode(GPIO.BCM)
        for pin in [IN1, IN2, ENA, IN3, IN4, ENB]:
            GPIO.setup(pin, GPIO.OUT)

        self.pwm_left = GPIO.PWM(ENA, PWM_FREQ)
        self.pwm_right = GPIO.PWM(ENB, PWM_FREQ)

        self.pwm_left.start(0)
        self.pwm_right.start(0)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def set_motor(self, speed, in1, in2, pwm):
        if speed > 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)

        pwm.ChangeDutyCycle(min(abs(speed), MAX_DUTY))

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left = linear - angular
        right = linear + angular

        left_pwm = int(left * 100)
        right_pwm = int(right * 100)

        self.set_motor(left_pwm, IN1, IN2, self.pwm_left)
        self.set_motor(right_pwm, IN3, IN4, self.pwm_right)

    def destroy_node(self):
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
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
