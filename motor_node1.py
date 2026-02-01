import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ===== SAFE GPIO CHECK =====
GPIO_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        self.in1 = 17
        self.in2 = 27
        self.in3 = 26
        self.in4 = 19

        # --- GPIO INIT ---
        if GPIO is not None:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.in1, GPIO.OUT)
                GPIO.setup(self.in2, GPIO.OUT)
                GPIO.setup(self.in3, GPIO.OUT)
                GPIO.setup(self.in4, GPIO.OUT)
                GPIO_AVAILABLE = True
                self.get_logger().info("GPIO initialized successfully")
            except RuntimeError:
                GPIO_AVAILABLE = False
                self.get_logger().warn(
                    "GPIO present but no SoC access — software-only mode"
                )
        else:
            self.get_logger().warn(
                "RPi.GPIO not available — software-only mode"
            )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )


    # ===== CALLBACK =====
def cmd_vel_callback(self, msg: Twist):
    linear = msg.linear.x

    if not GPIO_AVAILABLE:
        return

    if linear > 0:
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)
    elif linear < 0:
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
    else:
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)


def main():
    rclpy.init()
    node = DiffDriveL298N()
    rclpy.spin(node)
    rclpy.shutdown()
