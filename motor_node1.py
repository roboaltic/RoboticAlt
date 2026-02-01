import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ===== SAFE GPIO IMPORT =====
GPIO_AVAILABLE = False
GPIO = None

try:
    import RPi.GPIO as GPIO
except ImportError:
    pass


  def __init__(self):
        super().__init__('diff_drive_node')
class DiffDriveL298N(Node):

  
        self.in1 = 17
       class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_l298n')

        # ===== PARAMETERS =====
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.2)

        self.max_linear = self.get_parameter(
            'max_linear_speed').value
        self.max_angular = self.get_parameter(
            'max_angular_speed').value

        # ===== GPIO PINS =====
        self.in1 = 17
        self.in2 = 27
        self.in3 = 26
        self.in4 = 19
        self.en_a = 22
        self.en_b = 13

        # ===== GPIO INIT =====
        global GPIO_AVAILABLE
        if GPIO is not None:
            try:
                GPIO.setmode(GPIO.BCM)

                GPIO.setup(self.in1, GPIO.OUT)
                GPIO.setup(self.in2, GPIO.OUT)
                GPIO.setup(self.in3, GPIO.OUT)
                GPIO.setup(self.in4, GPIO.OUT)
                GPIO.setup(self.en_a, GPIO.OUT)
                GPIO.setup(self.en_b, GPIO.OUT)

                self.pwm_left = GPIO.PWM(self.en_a, 1000)
                self.pwm_right = GPIO.PWM(self.en_b, 1000)

                self.pwm_left.start(0)
                self.pwm_right.start(0)

                GPIO_AVAILABLE = True
                self.get_logger().info("GPIO initialized")

            except RuntimeError:
                self.get_logger().warn(
                    "GPIO detected but no SoC access (software-only mode)"
                )
        else:
            self.get_logger().warn(
                "RPi.GPIO not available (software-only mode)"
            )

        # ===== ROS SUB =====
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ===== WATCHDOG =====
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_check)


    # ===== CALLBACK =====
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        # --- LIMITS ---
        linear = max(
            -self.max_linear,
            min(self.max_linear, msg.linear.x)
        )
        angular = max(
            -self.max_angular,
            min(self.max_angular, msg.angular.z)
        )

        # --- DIFF DRIVE ---
        left = linear - angular
        right = linear + angular

        left_pwm = min(abs(left) * 100, 100)
        right_pwm = min(abs(right) * 100, 100)

        if not GPIO_AVAILABLE:
            return

        # LEFT MOTOR
        GPIO.output(self.in1, left >= 0)
        GPIO.output(self.in2, left < 0)

        # RIGHT MOTOR
        GPIO.output(self.in3, right >= 0)
        GPIO.output(self.in4, right < 0)

        self.pwm_left.ChangeDutyCycle(left_pwm)
        self.pwm_right.ChangeDutyCycle(right_pwm)

    def watchdog_check(self):
        dt = (self.get_clock().now() -
              self.last_cmd_time).nanoseconds / 1e9

        if dt > 0.5 and GPIO_AVAILABLE:
            self.pwm_left.ChangeDutyCycle(0)
            self.pwm_right.ChangeDutyCycle(0)

def main():
    rclpy.init()
    node = DiffDriveL298N()
    rclpy.spin(node)
    rclpy.shutdown()
