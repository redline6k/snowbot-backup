#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray  # adjust type if yours differs
import time

class BenchEncoderTest(Node):
    def __init__(self):
        super().__init__('bench_encoder_test')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.enc_sub = self.create_subscription(
            Float32MultiArray,  # [left_vel, right_vel] from your encoder_node
            '/wheel_velocities',
            self.enc_cb,
            10
        )
        self.left_sum = 0.0
        self.right_sum = 0.0
        self.count = 0

    def enc_cb(self, msg):
        if len(msg.data) < 2:
            return
        left = msg.data[0]
        right = msg.data[1]
        self.left_sum += left
        self.right_sum += right
        self.count += 1

    def run_test(self, speed=0.2, duration=5.0):
        # Let encoders/odom settle
        self.get_logger().info('Waiting 1s for /wheel_velocities...')
        start_wait = time.time()
        while time.time() - start_wait < 1.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'Starting bench test: {speed} m/s for {duration}s')

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        self.left_sum = 0.0
        self.right_sum = 0.0
        self.count = 0

        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # stop
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(1.0)

        if self.count == 0:
            self.get_logger().warn('No encoder data received!')
            return

        left_avg = self.left_sum / self.count
        right_avg = self.right_sum / self.count
        ratio = right_avg / left_avg if left_avg != 0.0 else 0.0

        self.get_logger().info(
            f'Enc averages over {duration:.1f}s: '
            f'LEFT={left_avg:.3f}, RIGHT={right_avg:.3f}, R/L ratio={ratio:.3f}'
        )
        print(f'\nRESULT: LEFT={left_avg:.3f}, RIGHT={right_avg:.3f}, R/L={ratio:.3f}')
        print('If R/L > 1.0, right is faster; if < 1.0, left is faster.')
        print('Use this ratio as a feed-forward trim in your motor controller.')

def main():
    rclpy.init()
    node = BenchEncoderTest()
    node.run_test(speed=0.2, duration=5.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
