#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('route_script_node')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        time.sleep(2.0)


def move(pub, speed, distance):
    msg = Twist()
    msg.linear.x = speed

    duration = distance / speed

    print(f'Move {distance:.2f} m @ {speed:.2f} m/s')

    start = time.time()

    while time.time() - start < duration:
        pub.publish(msg)
        time.sleep(0.05)

    stop(pub)


def turn_right_90(pub):

    msg = Twist()
    msg.angular.z = -0.6

    duration = 2.6

    print('Turn right 90 deg')

    start = time.time()

    while time.time() - start < duration:
        pub.publish(msg)
        time.sleep(0.05)

    stop(pub)


def stop(pub):

    msg = Twist()
    pub.publish(msg)
    time.sleep(0.5)


def main():

    rclpy.init()

    node = CmdVelPublisher()

    pub = node.pub

    print('=== START ROUTE ===')

    # 1
    move(pub, 0.09, 0.30)

    # 2
    move(pub, 0.12, 0.20)

    # 3
    move(pub, 0.09, 0.69)

    # 4
    move(pub, 0.09, 0.10)

    # 5
    turn_right_90(pub)

    # 6
    move(pub, 0.09, 0.69)

    # 7
    turn_right_90(pub)

    # 8
    move(pub, 0.12, 0.20)

    # 9
    move(pub, 0.09, 0.90)

    # 10
    turn_right_90(pub)

    # 11
    move(pub, 0.09, 0.60)

    # 12
    turn_right_90(pub)

    print('=== ROUTE FINISHED ===')

    stop(pub)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
