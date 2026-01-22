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


class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, setpoint, measured_value, current_time):
        """Compute PID output"""
        error = setpoint - measured_value

        if self.last_time is None:
            self.last_time = current_time
            self.prev_error = error
            return 0.0

        dt = current_time - self.last_time
        if dt <= 0.0:
            return 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, 100.0), -100.0)
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(min(output, self.output_max), self.output_min)

        # Update state
        self.prev_error = error
        self.last_time = current_time

        return output

    def reset(self):
        """Reset PID state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None


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

        # Robot parameters
        self.wheel_radius = 0.020  # 40mm diameter ÷ 2 = 20mm = 0.020 meters
        self.wheel_base = 0.260  # 260mm = 0.260 meters
        self.gear_ratio = 31.2  # 30:1 gear reduction

        # PID Controllers for each motor (wheel velocity in rad/s)
        # Tuning parameters: adjust these for your robot
        kp = 50.0  # Proportional gain
        ki = 20.0  # Integral gain
        kd = 5.0  # Derivative gain

        self.left_pid = PIDController(kp, ki, kd, -500, 500)  # Output is PWM offset
        self.right_pid = PIDController(kp, ki, kd, -500, 500)

        # Desired velocities (rad/s at motor shaft)
        self.desired_left_vel = 0.0
        self.desired_right_vel = 0.0

        # Measured velocities from encoders
        self.measured_left_vel = 0.0
        self.measured_right_vel = 0.0

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

        # PID control loop timer (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)

        # Set motors to neutral
        self.set_motors(1500, 1500)
        self.get_logger().info("Motor controller with PID initialized")
        self.get_logger().info(f"PID gains: Kp={kp}, Ki={ki}, Kd={kd}")

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
        """Convert Twist to desired wheel velocities"""
        linear = msg.linear.x  # m/s
        angular = msg.angular.z  # rad/s

        # Convert robot velocities to wheel velocities (m/s)
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)

        # Convert wheel velocities (m/s) to motor shaft velocities (rad/s)
        # wheel_vel = v / wheel_radius, motor_vel = wheel_vel * gear_ratio
        self.desired_left_vel = (v_left / self.wheel_radius) * self.gear_ratio
        self.desired_right_vel = (v_right / self.wheel_radius) * self.gear_ratio

        # Reset PID if stopping
        if abs(linear) < 1e-3 and abs(angular) < 1e-3:
            self.left_pid.reset()
            self.right_pid.reset()

        self.get_logger().info(
            f"Desired: L={self.desired_left_vel:.2f} R={self.desired_right_vel:.2f} rad/s",
            throttle_duration_sec=1.0,
        )

    def control_loop(self):
        """PID control loop - runs at 50 Hz"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # If no commanded motion, hold neutral and reset PID
        if abs(self.desired_left_vel) < 1e-3 and abs(self.desired_right_vel) < 1e-3:
            self.left_pid.reset()
            self.right_pid.reset()
            self.set_motors(1500, 1500)
            return

        # Compute PID outputs
        left_correction = self.left_pid.compute(
            self.desired_left_vel, self.measured_left_vel, current_time
        )
        right_correction = self.right_pid.compute(
            self.desired_right_vel, self.measured_right_vel, current_time
        )

        # Convert desired velocities to base PWM (simple feedforward)
        # Map velocity range to PWM: 0 rad/s = 1500us (neutral)
        # Rough estimate: max motor speed ~50 rad/s = 500us offset
        left_base_pwm = 1500 + int(self.desired_left_vel * 10)
        right_base_pwm = 1500 + int(self.desired_right_vel * 10)

        # Add PID correction
        left_pwm = left_base_pwm + int(left_correction)
        right_pwm = right_base_pwm + int(right_correction)

        # Set motors
        self.set_motors(left_pwm, right_pwm)

        # Debug output
        if abs(self.desired_left_vel) > 0.1 or abs(self.desired_right_vel) > 0.1:
            self.get_logger().debug(
                f"L: des={self.desired_left_vel:.1f} meas={self.measured_left_vel:.1f} "
                f"pwm={left_pwm} | R: des={self.desired_right_vel:.1f} "
                f"meas={self.measured_right_vel:.1f} pwm={right_pwm}"
            )

    def encoder_callback(self, msg):
        """Process encoder velocities and publish odometry"""
        self.measured_left_vel = msg.data[0]  # rad/s at motor shaft
        self.measured_right_vel = msg.data[1]

        # Convert motor shaft velocity to wheel velocity
        left_wheel_vel = self.measured_left_vel / self.gear_ratio
        right_wheel_vel = self.measured_right_vel / self.gear_ratio

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
