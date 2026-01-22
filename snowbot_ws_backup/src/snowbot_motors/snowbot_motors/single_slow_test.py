#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class SlowTester(Node):
    def __init__(self):
        super().__init__('slow_tester')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.current_odom = None
    
    def odom_callback(self, msg):
        self.current_odom = msg
    
    def wait_ready(self):
        while self.current_odom is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def test_slow(self):
        self.wait_ready()
        start_x = self.current_odom.pose.pose.position.x
        
        print("DRIVING 0.02 m/s (0.065 ft/s) for 3 seconds...")
        print("Expected: ~0.06 meters (2 inches)")
        
        msg = Twist()
        msg.linear.x = 0.02  # SUPER SLOW - 0.065 ft/s
        start_time = time.time()
        
        while (time.time() - start_time) < 3.0:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Stop
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.5)
        
        end_x = self.current_odom.pose.pose.position.x
        distance = end_x - start_x
        
        print(f"Odometry distance: {distance:.3f}m ({distance*3.28:.2f}ft)")
        print("Measure with ruler and compare!")
        print("Press Ctrl+C to exit")

def main(args=None):
    rclpy.init(args=args)
    tester = SlowTester()
    try:
        tester.test_slow()
        rclpy.spin(tester)
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
