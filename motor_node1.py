import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ===== GPIO SAFE IMPORT =====
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO_AVAILABLE = True
except Exception:
    GPIO_AVAILABLE = False


class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,   # <-- ВОТ ТУТ CALLBACK ПРИВЯЗАН
            10
        )

        # --- GPIO PINS ---
        self.in1 = 17
        self.in2 = 22
        self.in3 = 26
        self.in4 = 19

        if GPIO_AVAILABLE:
            GPIO.setup(self.in1, GPIO.OUT)
            GPIO.setup(self.in2, GPIO.OUT)
            GPIO.setup(self.in3, GPIO.OUT)
            GPIO.setup(self.in4, GPIO.OUT)
        else:
            self.get_logger().warn(
                "GPIO not available — running in software-only mode"
            )

    # ===== CALLBACK =====
    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x

        if linear == 0.0:
            self.stop_motors()
            return

        forward = linear > 0

        if GPIO_AVAILABLE:
            # Left motor
            GPIO.output(self.in1, GPIO.HIGH if forward else GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW if forward else GPIO.HIGH)

            # Right motor
            GPIO.output(self.in3, GPIO.HIGH if forward else GPIO.LOW)
            GPIO.output(self.in4, GPIO.LOW if forward else GPIO.HIGH)

    def stop_motors(self):
        if GPIO_AVAILABLE:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.LOW)
            GPIO.output(self.in4, GPIO.LOW)


def main():
    rclpy.init()
    node = DiffDriveL298N()
    rclpy.spin(node)
    rclpy.shutdown()
