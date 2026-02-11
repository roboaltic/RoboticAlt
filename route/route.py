#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import yaml
import time
import os


class RouteExecutor(Node):

    def __init__(self):
        super().__init__('route_executor')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Path to route file
        self.route_file = os.path.expanduser(
            '~/ros2_ws/routes/route.yaml'
        )

        self.steps = self.load_route()

        self.current_step = 0
        self.step_start = time.time()

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.get_logger().info("Route Executor started")


    # =============================

    def load_route(self):

        if not os.path.exists(self.route_file):
            self.get_logger().error(
                f"Route file not found: {self.route_file}"
            )
            return []

        with open(self.route_file, 'r') as f:
            data = yaml.safe_load(f)

        return data.get('steps', [])


    # =============================

    def update(self):

        if self.current_step >= len(self.steps):

            self.stop_robot()
            self.get_logger().info("Route finished")
            rclpy.shutdown()
            return


        step = self.steps[self.current_step]

        duration = step['time']

        now = time.time()

        if now - self.step_start >= duration:

            self.current_step += 1
            self.step_start = now
            return


        msg = Twist()

        msg.linear.x = step['linear'][0]
        msg.linear.y = step['linear'][1]
        msg.linear.z = step['linear'][2]

        msg.angular.x = step['angular'][0]
        msg.angular.y = step['angular'][1]
        msg.angular.z = step['angular'][2]

        self.publisher.publish(msg)


    # =============================

    def stop_robot(self):

        msg = Twist()
        self.publisher.publish(msg)



def main():

    rclpy.init()

    node = RouteExecutor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
