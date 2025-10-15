#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class SquarePathController(Node):
    def __init__(self):
        super().__init__('square_path_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.side_length = 2.0
        self.linear_speed = 0.3
        self.angular_speed = 0.3
        self.get_logger().info('Square Path Controller Initialized')
        
    def move_forward(self, distance):
        msg = Twist()
        msg.linear.x = self.linear_speed
        duration = distance / self.linear_speed
        self.get_logger().info(f'Moving forward {distance}m for {duration:.2f}s')
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Stopped')
        time.sleep(0.5)
        
    def turn_90_degrees(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        angle = math.pi / 2
        duration = angle / self.angular_speed
        self.get_logger().info(f'Turning 90 degrees for {duration:.2f}s')
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Turn complete')
        time.sleep(0.5)
    
    def execute_square_path(self):
        self.get_logger().info('='*50)
        self.get_logger().info('STARTING SQUARE PATH EXECUTION')
        self.get_logger().info('='*50)
        for i in range(4):
            self.get_logger().info(f'\n--- SIDE {i+1} of 4 ---')
            self.move_forward(self.side_length)
            if i < 3:
                self.turn_90_degrees()
        self.get_logger().info('='*50)
        self.get_logger().info('SQUARE PATH COMPLETED SUCCESSFULLY!')
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    controller = SquarePathController()
    try:
        controller.execute_square_path()
        time.sleep(1)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    except Exception as e:
        controller.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

