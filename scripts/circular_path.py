#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class CircularPathController(Node):
    def __init__(self):
        super().__init__('circular_path_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.radius = 1.5
        self.linear_speed = 0.3
        self.angular_speed = self.linear_speed / self.radius
        self.get_logger().info('Circular Path Controller Initialized')
        self.get_logger().info(f'Radius: {self.radius}m')
        
    def execute_circular_path(self):
        self.get_logger().info('='*50)
        self.get_logger().info('STARTING CIRCULAR PATH EXECUTION')
        self.get_logger().info('='*50)
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        circumference = 2 * math.pi * self.radius
        duration = circumference / self.linear_speed
        self.get_logger().info(f'Duration: {duration:.2f}s')
        start_time = time.time()
        last_update = start_time
        while (time.time() - start_time) < duration:
            self.publisher_.publish(msg)
            if (time.time() - last_update) >= 2.0:
                progress = ((time.time() - start_time) / duration) * 100
                self.get_logger().info(f'Progress: {progress:.1f}%')
                last_update = time.time()
            time.sleep(0.05)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('='*50)
        self.get_logger().info('CIRCULAR PATH COMPLETED!')
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    controller = CircularPathController()
    try:
        controller.execute_circular_path()
        time.sleep(1)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

