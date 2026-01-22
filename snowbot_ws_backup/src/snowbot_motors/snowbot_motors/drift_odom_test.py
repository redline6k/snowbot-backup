#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class DriftOdomTester(Node):
    def __init__(self):
        super().__init__('drift_odom_tester')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.current_pose = None
        self.odom_count = 0

    def odom_cb(self, msg):
        self.current_pose = msg
        self.odom_count += 1
        self.get_logger().info(f"Odom received #{self.odom_count}")

    def wait_for_odom(self):
        print("Waiting for /odom...")
        for i in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pose:
                print(f"✓ Got odom! Count: {self.odom_count}")
                return True
        print("✗ No odom after 5s")
        return False

    def run_linear_test(self):
        if not self.wait_for_odom():
            return

        start_x = self.current_pose.pose.pose.position.x
        print(f"START X: {start_x:.3f}m")

        twist = Twist()
        twist.linear.x = 0.1
        start_time = time.time()
        print("DRIVING 0.1m/s for 3s...")
        while time.time() - start_time < 3.0:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(1)

        end_x = self.current_pose.pose.pose.position.x
        distance = abs(end_x - start_x)
        print(f"END X: {end_x:.3f}m")
        print(f"ODOMETRY DISTANCE: {distance:.3f}m")
        print("🎯 MEASURE PHYSICAL DISTANCE WITH TAPE!")
        print("Tune wheel_radius = wheel_radius * (measured / odom_dist)")

    def run_turn_test(self):
        if not self.wait_for_odom():
            return

        print("\n=== TURN TEST (90°) ===")
        twist = Twist()
        twist.angular.z = 1.0
        start_yaw = self.current_pose.pose.pose.orientation.z
        duration = math.pi / 2 / 1.0
        start_time = time.time()

        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(1)

        end_yaw = self.current_pose.pose.pose.orientation.z
        dyaw = abs(end_yaw - start_yaw)
        arc_length = 0.26 * dyaw  # wheel_base * angle
        print(f"90° Expected: 1.57 rad, arc 0.41m")
        print(f"Got: {dyaw:.2f} rad, arc {arc_length:.2f}m")
        print("Measure arc length with string/tape")

def main():
    rclpy.init()
    tester = DriftOdomTester()
    print("=== SNOWBOT ODOM CALIBRATION ===")
    tester.run_linear_test()
    tester.run_turn_test()
    input("\nPress Enter to quit...")
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
