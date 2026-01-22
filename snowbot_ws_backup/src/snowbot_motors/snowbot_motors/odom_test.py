#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class OdomTester(Node):
    def __init__(self):
        super().__init__('odom_tester')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.current_odom = None
        self.start_odom = None
        
        self.get_logger().info('Odometry tester ready')
    
    def odom_callback(self, msg):
        self.current_odom = msg
    
    def wait_for_odom(self):
        """Wait until we receive odometry data"""
        self.get_logger().info('Waiting for odometry data...')
        while self.current_odom is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Odometry received!')
    
    def get_position(self):
        """Get current X, Y position"""
        if self.current_odom:
            x = self.current_odom.pose.pose.position.x
            y = self.current_odom.pose.pose.position.y
            return x, y
        return 0.0, 0.0
    
    def get_distance_traveled(self):
        """Calculate distance from start position"""
        if self.start_odom and self.current_odom:
            dx = self.current_odom.pose.pose.position.x - self.start_odom.pose.pose.position.x
            dy = self.current_odom.pose.pose.position.y - self.start_odom.pose.pose.position.y
            return math.sqrt(dx**2 + dy**2)
        return 0.0
    
    def stop(self):
        """Stop the robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def drive_forward(self, speed, duration):
        """Drive forward at specified speed for duration seconds"""
        self.get_logger().info(f'Driving forward at {speed} m/s for {duration} seconds...')
        
        # Record start position
        self.start_odom = self.current_odom
        start_x, start_y = self.get_position()
        self.get_logger().info(f'Start position: X={start_x:.3f}m, Y={start_y:.3f}m')
        
        # Drive forward
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        
        start_time = time.time()
        while (time.time() - start_time) < duration and rclpy.ok():
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Stop
        self.stop()
        time.sleep(0.5)  # Let it settle
        
        # Report results
        end_x, end_y = self.get_position()
        distance = self.get_distance_traveled()
        
        self.get_logger().info(f'End position: X={end_x:.3f}m, Y={end_y:.3f}m')
        self.get_logger().info(f'Distance traveled (odometry): {distance:.3f}m')
        self.get_logger().info(f'Expected distance: {speed * duration:.3f}m')
        
        error_percent = abs(distance - (speed * duration)) / (speed * duration) * 100
        self.get_logger().info(f'Error: {error_percent:.1f}%')
        
        return distance
    
    def turn_test(self, angular_speed, duration):
        """Turn in place at specified angular speed"""
        self.get_logger().info(f'Turning at {angular_speed} rad/s for {duration} seconds...')
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed
        
        start_time = time.time()
        while (time.time() - start_time) < duration and rclpy.ok():
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        self.stop()
        self.get_logger().info('Turn complete')

def main(args=None):
    rclpy.init(args=args)
    tester = OdomTester()
    
    try:
        # Wait for odometry
        tester.wait_for_odom()
        time.sleep(1)
        
        print("\n" + "="*50)
        print("ODOMETRY ACCURACY TEST")
        print("="*50 + "\n")
        
        # Test 1: Slow forward
        print("\n--- TEST 1: Slow Forward (0.2 m/s for 5 seconds) ---")
        tester.drive_forward(0.2, 5.0)
        time.sleep(2)
        
        # Test 2: Medium forward
        print("\n--- TEST 2: Medium Forward (0.3 m/s for 5 seconds) ---")
        tester.drive_forward(0.3, 5.0)
        time.sleep(2)
        
        # Test 3: Turn test
        print("\n--- TEST 3: Turn in Place (0.5 rad/s for 3 seconds) ---")
        tester.turn_test(0.5, 3.0)
        time.sleep(2)
        
        # Final stop
        tester.stop()
        
        print("\n" + "="*50)
        print("TEST COMPLETE")
        print("="*50)
        print("\nMeasure the actual distances traveled and compare")
        print("to the odometry values above.")
        print("\nIf odometry is off, adjust wheel_radius in motor_node.py")
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.stop()
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
