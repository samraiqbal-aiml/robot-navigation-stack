#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

class SimplePathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')
        
        # Simple grid-based path planner
        self.grid_size = 10
        self.obstacles = [(2, 2), (3, 3), (7, 7)]  # Example obstacles
        
        self.get_logger().info('🗺️ Simple Path Planner Started!')
    
    def plan_path(self, start, goal):
        """Simple straight-line path planning with obstacle check"""
        self.get_logger().info(f'📍 Planning path from {start} to {goal}')
        
        # For demo purposes, return a straight path
        # In real implementation, this would use A* or RRT
        path = [start, goal]
        
        self.get_logger().info(f'🛣️ Path planned: {path}')
        return path

def main(args=None):
    rclpy.init(args=args)
    node = SimplePathPlanner()
    
    # Example usage
    start = (0, 0)
    goal = (8, 8)
    path = node.plan_path(start, goal)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
