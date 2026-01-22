#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class IndoorOdomTester(Node):
    def __init__(self):
        super().__init__('indoor_odom_tester')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.current_odom = None
        self.start_odom = None
        
        self.get_logger().info('Indoor odometry tester ready - SLOW SPEED FOR 20FT SPACE')
    
    def odom_callback(self, msg):
        self.current_odom = msg
    
    def wait_for_odom(self):
        self.get_logger().info('Waiting for odometry data...')
        while self.current_odom is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Odometry received!')
    
    def get_position(self):
        if self.current_odom:
            x = self.current_odom.pose.pose.position.x
            y = self.current_odom.pose.pose.position.y
            return x, y
        return 0.0, 0.0
    
    def get_distance_traveled(self):
        if self.start_odom and self.current_odom:
            dx = self.current_odom.pose.pose.position.x - self.start_odom.pose.pose.position.x
            dy = self.current_odom.pose.pose.position.y - self.start_odom.pose.pose.position.y
            return math.sqrt(dx**2 + dy**2)
        return 0.0
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def drive_forward(self, speed, duration):
        self.get_logger().info(f'Driving forward at {speed} m/s for {duration}s...')
        
        self.start_odom = self.current_odom
        start_x, start_y = self.get_position()
        self.get_logger().info(f'Start: X={start_x:.3f}m, Y={start_y:.3f}m')
        
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        
        start_time = time.time()
        while (time.time() - start_time) < duration and rclpy.ok():
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        self.stop()
        time.sleep(1.0)
        
        end_x, end_y = self.get_position()
        distance = self.get_distance_traveled()
        
        self.get_logger().info(f'End: X={end_x:.3f}m, Y={end_y:.3f}m')
        self.get_logger().info(f'Odometry distance: {distance:.3f}m')
        self.get_logger().info(f'Expected: {speed * duration:.3f}m')
        
        error_percent = abs(distance - (speed * duration)) / (speed * duration) * 100
        self.get_logger().info(f'Error: {error_percent:.1f}%')
        self.get_logger().info(f'Actual distance to measure: ~{distance*3.28:.1f} feet')
        
        return distance

def main(args=None):
    rclpy.init(args=args)
    tester = IndoorOdomTester()
    
    try:
        tester.wait_for_odom()
        time.sleep(2)
        
        print("\n" + "="*60)
        print("INDOOR ODOMETRY TEST - 20FT SPACE")
        print("="*60 + "\n")
        
        # Test 1: Very slow - fits in 20ft
        print("--- TEST 1: 0.15 m/s (0.5 ft/s) for 4 seconds (~6.5ft) ---")
        dist1 = tester.drive_forward(0.15, 4.0)
        
        # Test 2: Even slower, straighter
        print("\n--- TEST 2: 0.10 m/s (0.33 ft/s) for 5 seconds (~5.2ft) ---")
        dist2 = tester.drive_forward(0.10, 5.0)
        
        tester.stop()
        
        print("\n" + "="*60)
        print("TEST COMPLETE - MEASURE WITH TAPE!")
        print("="*60)
        print(f"Test 1 traveled: {dist1*3.28:.1f} feet")
        print(f"Test 2 traveled: {dist2*3.28:.1f} feet")
        print("\nMeasure actual distances and compare!")
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.stop()
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
