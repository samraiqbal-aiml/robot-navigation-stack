#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Navigation variables
        self.current_pose = Point()
        self.goal_pose = Point(x=5.0, y=5.0)  # Target position
        self.reached_goal = False
        
        # Control parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.goal_tolerance = 0.3
        
        self.get_logger().info('🚀 Simple Navigator Started!')
        self.get_logger().info(f'🎯 Navigating to: ({self.goal_pose.x}, {self.goal_pose.y})')
        
        # Start navigation loop
        self.create_timer(0.1, self.navigation_loop)
    
    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
    
    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    
    def calculate_angle_to_goal(self):
        """Calculate angle to goal position"""
        dx = self.goal_pose.x - self.current_pose.x
        dy = self.goal_pose.y - self.current_pose.y
        return math.atan2(dy, dx)
    
    def navigation_loop(self):
        """Main navigation control loop"""
        if self.reached_goal:
            return
        
        # Calculate distance to goal
        distance = self.calculate_distance(self.current_pose, self.goal_pose)
        
        if distance < self.goal_tolerance:
            # Stop the robot
            twist_msg = Twist()
            self.cmd_pub.publish(twist_msg)
            self.reached_goal = True
            self.get_logger().info('🎉 Goal reached!')
            return
        
        # Calculate required angle
        target_angle = self.calculate_angle_to_goal()
        
        # Simple control logic
        twist_msg = Twist()
        
        # Move forward if roughly aligned with goal
        if abs(target_angle) < 0.2:  # ~11 degrees
            twist_msg.linear.x = self.linear_speed
        else:
            # Rotate towards goal
            twist_msg.angular.z = self.angular_speed if target_angle > 0 else -self.angular_speed
        
        self.cmd_pub.publish(twist_msg)
        
        # Log progress
        self.get_logger().info(f'📊 Distance to goal: {distance:.2f}m', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
