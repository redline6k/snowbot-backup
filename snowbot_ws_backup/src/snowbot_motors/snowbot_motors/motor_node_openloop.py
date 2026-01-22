#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from adafruit_pca9685 import PCA9685
import board
import busio
import math
from tf_transformations import quaternion_from_euler


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # Initialize PCA9685
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c_bus, address=0x40)
        self.pca.frequency = 50

        # Sabertooth channels
        self.SABERTOOTH_S1 = 4  # Left track
        self.SABERTOOTH_S2 = 5  # Right track

        # Robot parameters (ADJUST THESE FOR YOUR ROBOT)
        self.wheel_radius = 0.020  # meters (5cm radius = 10cm diameter)
        self.wheel_base = 0.260  # meters (distance between tracks)
        self.gear_ratio = 30.0  # Motor shaft to wheel ratio

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        self.encoder_sub = self.create_subscription(
            Float32MultiArray, "/wheel_velocities", self.encoder_callback, 10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # Set motors to neutral
        self.set_motors(1500, 1500)
        self.get_logger().info("Motor controller with odometry initialized")

    def pwm_to_duty_cycle(self, microseconds):
        """Convert microseconds (1000-2000) to PCA9685 duty cycle"""
        return int((microseconds / 20000.0) * 65535)

    def set_motors(self, left_us, right_us):
        """Set motor speeds in microseconds"""
        left_us = max(1000, min(2000, left_us))
        right_us = max(1000, min(2000, right_us))

        self.pca.channels[self.SABERTOOTH_S1].duty_cycle = self.pwm_to_duty_cycle(
            left_us
        )
        self.pca.channels[self.SABERTOOTH_S2].duty_cycle = self.pwm_to_duty_cycle(
            right_us
        )

    def cmd_vel_callback(self, msg):
        """Convert Twist to differential drive commands"""
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive mixing
        left_speed = linear - angular
        right_speed = linear + angular

        # Clamp and convert to PWM
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        left_us = int(1500 + (left_speed * 500))
        right_us = int(1500 + (right_speed * 500))

        self.set_motors(left_us, right_us)

        self.get_logger().info(f"Motors: L={left_us}us R={right_us}us")

    def encoder_callback(self, msg):
        """Process encoder velocities and publish odometry"""
        left_motor_vel = msg.data[0]  # rad/s at motor shaft
        right_motor_vel = msg.data[1]

        # Convert motor shaft velocity to wheel velocity
        left_wheel_vel = left_motor_vel / self.gear_ratio
        right_wheel_vel = right_motor_vel / self.gear_ratio

        # Calculate linear velocities at each wheel
        v_left = left_wheel_vel * self.wheel_radius  # m/s
        v_right = right_wheel_vel * self.wheel_radius

        # Calculate robot linear and angular velocity
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        # Update odometry
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt > 0:
            delta_x = v * math.cos(self.theta) * dt
            delta_y = v * math.sin(self.theta) * dt
            delta_theta = omega * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Publish odometry
            self.publish_odometry(v, omega, current_time)

        self.last_time = current_time

    def publish_odometry(self, v, omega, timestamp):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

    def shutdown(self):
        """Stop motors on shutdown"""
        self.get_logger().info("Shutting down - stopping motors")
        self.set_motors(1500, 1500)
        self.pca.deinit()


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.shutdown()
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
