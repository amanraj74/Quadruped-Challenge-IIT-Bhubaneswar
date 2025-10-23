#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time

class EnhancedAutonomousNavigation(Node):
    def __init__(self):
        super().__init__('enhanced_autonomous_navigation')
        
        # Publishers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Navigation parameters
        self.goal_x = 5.0  # Goal position X
        self.goal_y = 5.0  # Goal position Y
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Enhanced obstacle avoidance parameters
        self.min_safe_distance = 0.6  # meters
        self.warning_distance = 1.0    # slow down distance
        
        # Obstacle detection zones
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.front_distance = 10.0
        self.left_distance = 10.0
        self.right_distance = 10.0
        
        # Control parameters
        self.max_linear_speed = 0.3
        self.min_linear_speed = 0.1
        self.current_linear_speed = self.max_linear_speed
        self.angular_speed = 0.6
        self.goal_tolerance = 0.3
        
        # State machine
        self.state = "SEEKING_GOAL"  # SEEKING_GOAL, AVOIDING_OBSTACLE, STUCK_RECOVERY, REACHED_GOAL
        self.previous_state = "SEEKING_GOAL"
        
        # Stuck detection
        self.stuck_counter = 0
        self.max_stuck_count = 50
        self.last_position = (0.0, 0.0)
        self.position_check_counter = 0
        
        # Statistics
        self.obstacles_avoided = 0
        self.total_distance = 0.0
        
        self.get_logger().info('='*70)
        self.get_logger().info('🏆 ENHANCED AUTONOMOUS NAVIGATION SYSTEM INITIALIZED')
        self.get_logger().info('='*70)
        self.get_logger().info(f'📍 Goal Position: ({self.goal_x:.1f}, {self.goal_y:.1f})')
        self.get_logger().info(f'🛡️  Safe Distance: {self.min_safe_distance}m')
        self.get_logger().info(f'⚠️  Warning Distance: {self.warning_distance}m')
        self.get_logger().info(f'🎯 Goal Tolerance: {self.goal_tolerance}m')
        self.get_logger().info('='*70)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        self.log_timer = self.create_timer(2.0, self.log_status)  # Log every 2s
        
    def laser_callback(self, msg):
        """Enhanced laser scan processing with multi-zone detection"""
        
        ranges = msg.ranges
        num_readings = len(ranges)
        
        if num_readings == 0:
            return
        
        # Divide scan into zones: LEFT, FRONT, RIGHT
        # Assuming 360-degree scan
        zone_width = num_readings // 6
        
        # FRONT zone (center ±30 degrees)
        center = num_readings // 2
        front_start = center - zone_width
        front_end = center + zone_width
        
        # LEFT zone
        left_start = num_readings - zone_width
        left_end = num_readings
        
        # RIGHT zone  
        right_start = 0
        right_end = zone_width
        
        # Process FRONT zone
        front_ranges = []
        for i in range(max(0, front_start), min(num_readings, front_end)):
            r = ranges[i]
            if not math.isinf(r) and not math.isnan(r) and r > 0.0:
                front_ranges.append(r)
        
        if front_ranges:
            self.front_distance = min(front_ranges)
            self.front_clear = self.front_distance > self.min_safe_distance
        else:
            self.front_distance = 10.0
            self.front_clear = True
        
        # Process LEFT zone
        left_ranges = []
        for i in range(left_start, left_end):
            r = ranges[i]
            if not math.isinf(r) and not math.isnan(r) and r > 0.0:
                left_ranges.append(r)
        
        if left_ranges:
            self.left_distance = min(left_ranges)
            self.left_clear = self.left_distance > self.min_safe_distance
        else:
            self.left_distance = 10.0
            self.left_clear = True
        
        # Process RIGHT zone
        right_ranges = []
        for i in range(right_start, right_end):
            r = ranges[i]
            if not math.isinf(r) and not math.isnan(r) and r > 0.0:
                right_ranges.append(r)
        
        if right_ranges:
            self.right_distance = min(right_ranges)
            self.right_clear = self.right_distance > self.min_safe_distance
        else:
            self.right_distance = 10.0
            self.right_clear = True
    
    def calculate_distance_to_goal(self):
        """Calculate Euclidean distance to goal"""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        return math.sqrt(dx**2 + dy**2)
    
    def check_if_stuck(self):
        """Detect if robot is stuck"""
        self.position_check_counter += 1
        
        if self.position_check_counter >= 20:  # Check every 2 seconds
            distance_moved = math.sqrt(
                (self.current_x - self.last_position[0])**2 + 
                (self.current_y - self.last_position[1])**2
            )
            
            if distance_moved < 0.05:  # Moved less than 5cm
                self.stuck_counter += 1
                if self.stuck_counter > 3:
                    return True
            else:
                self.stuck_counter = 0
            
            self.last_position = (self.current_x, self.current_y)
            self.position_check_counter = 0
        
        return False
    
    def adaptive_speed_control(self):
        """Adjust speed based on obstacle proximity"""
        min_distance = min(self.front_distance, self.left_distance, self.right_distance)
        
        if min_distance < self.warning_distance:
            # Slow down as obstacle gets closer
            speed_ratio = (min_distance - self.min_safe_distance) / (self.warning_distance - self.min_safe_distance)
            speed_ratio = max(0.0, min(1.0, speed_ratio))
            self.current_linear_speed = self.min_linear_speed + (self.max_linear_speed - self.min_linear_speed) * speed_ratio
        else:
            self.current_linear_speed = self.max_linear_speed
    
    def intelligent_turning(self):
        """Decide turning direction based on which side is more open"""
        if self.left_clear and not self.right_clear:
            return self.angular_speed  # Turn left
        elif self.right_clear and not self.left_clear:
            return -self.angular_speed  # Turn right
        elif self.left_distance > self.right_distance:
            return self.angular_speed  # Turn toward more open space (left)
        else:
            return -self.angular_speed  # Turn toward more open space (right)
    
    def control_loop(self):
        """Main control loop with enhanced state machine"""
        
        if self.state == "REACHED_GOAL":
            return
        
        twist = Twist()
        distance_to_goal = self.calculate_distance_to_goal()
        
        # Check if goal reached
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('='*70)
            self.get_logger().info('🎯 GOAL REACHED SUCCESSFULLY!')
            self.get_logger().info(f'📊 Statistics:')
            self.get_logger().info(f'   • Obstacles Avoided: {self.obstacles_avoided}')
            self.get_logger().info(f'   • Total Distance: {self.total_distance:.2f}m')
            self.get_logger().info('='*70)
            self.stop_robot()
            self.state = "REACHED_GOAL"
            return
        
        # Check if stuck
        if self.check_if_stuck() and self.state != "STUCK_RECOVERY":
            self.get_logger().warn('🚨 STUCK DETECTED! Initiating recovery...')
            self.previous_state = self.state
            self.state = "STUCK_RECOVERY"
        
        # Adaptive speed control
        self.adaptive_speed_control()
        
        # State machine
        if self.state == "STUCK_RECOVERY":
            # Recovery behavior: back up and turn
            twist.linear.x = -0.1  # Back up
            twist.angular.z = self.angular_speed
            
            self.stuck_counter -= 1
            if self.stuck_counter <= 0:
                self.get_logger().info('✅ Recovery complete. Resuming navigation.')
                self.state = self.previous_state
                self.stuck_counter = 0
        
        elif self.state == "AVOIDING_OBSTACLE":
            # Intelligent obstacle avoidance
            if not self.front_clear or not self.left_clear or not self.right_clear:
                twist.linear.x = 0.0
                twist.angular.z = self.intelligent_turning()
                self.obstacles_avoided += 1
            else:
                # All clear, resume seeking
                self.get_logger().info('✅ Obstacle cleared. Resuming goal seeking.')
                self.state = "SEEKING_GOAL"
        
        elif self.state == "SEEKING_GOAL":
            # Check for obstacles
            if not self.front_clear:
                self.get_logger().warn(f'⚠️  OBSTACLE AHEAD! Distance: {self.front_distance:.2f}m')
                self.state = "AVOIDING_OBSTACLE"
            else:
                # Move toward goal
                twist.linear.x = self.current_linear_speed
                twist.angular.z = 0.0
                
                # Update position (dead reckoning)
                dt = 0.1
                self.current_x += self.current_linear_speed * dt * math.cos(self.current_yaw)
                self.current_y += self.current_linear_speed * dt * math.sin(self.current_yaw)
                self.current_yaw += twist.angular.z * dt
                self.total_distance += self.current_linear_speed * dt
        
        # Publish velocity command
        self.publisher_.publish(twist)
    
    def log_status(self):
        """Periodic status logging"""
        if self.state != "REACHED_GOAL":
            distance = self.calculate_distance_to_goal()
            self.get_logger().info(
                f'📍 Status: {self.state} | '
                f'Distance to goal: {distance:.2f}m | '
                f'Position: ({self.current_x:.2f}, {self.current_y:.2f}) | '
                f'Speed: {self.current_linear_speed:.2f}m/s | '
                f'Front: {self.front_distance:.2f}m | '
                f'Left: {self.left_distance:.2f}m | '
                f'Right: {self.right_distance:.2f}m'
            )
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = EnhancedAutonomousNavigation()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('🛑 Navigation interrupted by user')
    finally:
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

