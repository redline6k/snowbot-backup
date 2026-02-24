#!/usr/bin/env python3
"""
PCA9685/Sabertooth motor node with smoothing for smooth teleop.
Reduces jerky motion via velocity ramping and softened deadzone handling.
"""
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
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_min, output_max
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, setpoint, measured_value, current_time):
        error = setpoint - measured_value
        if self.last_time is None:
            self.last_time = current_time
            return 0.0
        dt = current_time - self.last_time
        if dt <= 0.0:
            return 0.0

        # Anti-windup integral
        self.integral = max(min(self.integral + (error * dt), 50.0), -50.0)
        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error
        self.last_time = current_time
        return max(min(p_term + i_term + d_term, self.output_max), self.output_min)

    def reset(self):
        self.prev_error = self.integral = 0.0
        self.last_time = None


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # Declare parameters for smoothing
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("max_vel_accel_per_sec", 150.0)  # velocity units/sec
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("vel_deadband", 0.1)
        self.declare_parameter("use_breakout_kick", False)  # Disabled by default – can cause jerk

        control_rate = self.get_parameter("control_rate_hz").value
        self.max_accel = self.get_parameter("max_vel_accel_per_sec").value / control_rate
        self.cmd_timeout = self.get_parameter("cmd_timeout_sec").value
        self.vel_deadband = self.get_parameter("vel_deadband").value
        self.use_breakout_kick = self.get_parameter("use_breakout_kick").value

        # 1. Hardware Initialization
        try:
            i2c_bus = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c_bus, address=0x40)
            self.pca.frequency = 50
        except Exception as e:
            self.get_logger().error(f"I2C/PCA9685 Init Failed: {e}")

        # 2. Parameters
        self.SABERTOOTH_S1, self.SABERTOOTH_S2 = 4, 5
        self.wheel_radius, self.wheel_base, self.gear_ratio = 0.0175, 0.260, 31.2

        # 3. PID Gains (Gentle to prevent stuttering)
        kp, ki, kd = 0.0, 0.0, 0.0
        self.left_pid = PIDController(kp, ki, kd, -300, 300)
        self.right_pid = PIDController(kp, ki, kd, -300, 300)

        # 4. State Variables
        self.target_left_vel = self.target_right_vel = 0.0
        self.smoothed_left_vel = self.smoothed_right_vel = 0.0
        self.measured_left_vel = self.measured_right_vel = 0.0
        self.x = self.y = self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # 5. Comms
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
        self.encoder_sub = self.create_subscription(
            Float32MultiArray, "/wheel_velocities", self.encoder_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.set_motors(1500, 1500)
        self.get_logger().info(
            f"SNOWBOT: smoothing enabled ({control_rate}Hz, "
            f"max_accel={self.max_accel * control_rate:.0f}/s)"
        )

    def set_motors(self, left_us, right_us):
        # PWM to Duty Cycle Conversion
        l_duty = int((max(1000, min(2000, left_us)) / 20000.0) * 65535)
        r_duty = int((max(1000, min(2000, right_us)) / 20000.0) * 65535)
        self.pca.channels[self.SABERTOOTH_S1].duty_cycle = l_duty
        self.pca.channels[self.SABERTOOTH_S2].duty_cycle = r_duty

    def cmd_vel_callback(self, msg: Twist):
        scale = 0.10
        v_l = (msg.linear.x * scale) - (msg.angular.z * scale * self.wheel_base / 2.0)
        v_r = (msg.linear.x * scale) + (msg.angular.z * scale * self.wheel_base / 2.0)
        self.target_left_vel = (v_l / self.wheel_radius) * self.gear_ratio
        self.target_right_vel = (v_r / self.wheel_radius) * self.gear_ratio
        self.last_cmd_time = self.get_clock().now()

    def _clamp_step(self, current: float, target: float) -> float:
        """Rate-limit velocity change for smooth acceleration."""
        delta = target - current
        if delta > self.max_accel:
            return current + self.max_accel
        if delta < -self.max_accel:
            return current - self.max_accel
        return target

    def _apply_deadband(self, val: float) -> float:
        if abs(val) < self.vel_deadband:
            return 0.0
        return val

    def control_loop(self):
        curr_t = self.get_clock().now().nanoseconds / 1e9
        now = self.get_clock().now()
        dt_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9

        # Timeout: ramp to stop if no cmd_vel
        if dt_since_cmd > self.cmd_timeout:
            self.target_left_vel = 0.0
            self.target_right_vel = 0.0

        # Apply deadband to reduce joystick jitter
        t_left = self._apply_deadband(self.target_left_vel)
        t_right = self._apply_deadband(self.target_right_vel)

        # Ramp smoothed velocities toward targets
        self.smoothed_left_vel = self._clamp_step(self.smoothed_left_vel, t_left)
        self.smoothed_right_vel = self._clamp_step(self.smoothed_right_vel, t_right)

        # Stop check
        if abs(self.smoothed_left_vel) < self.vel_deadband and abs(
            self.smoothed_right_vel
        ) < self.vel_deadband:
            self.left_pid.reset()
            self.right_pid.reset()
            self.set_motors(1500, 1500)
            return

        # PID Corrections (use smoothed velocities)
        l_corr = self.left_pid.compute(
            self.smoothed_left_vel, self.measured_left_vel, curr_t
        )
        r_corr = self.right_pid.compute(
            self.smoothed_right_vel, self.measured_right_vel, curr_t
        )

        mult = 7.5
        limit = 120

        l_pwm = 1500 + int(self.smoothed_left_vel * mult + l_corr)
        r_pwm = 1500 + int(self.smoothed_right_vel * mult + r_corr)

        # Breakout Force – optional; can cause jerk when enabled
        if self.use_breakout_kick:
            if 1500 < l_pwm < 1512:
                l_pwm = 1518
            if 1488 < l_pwm < 1500:
                l_pwm = 1482
            if 1500 < r_pwm < 1512:
                r_pwm = 1518
            if 1488 < r_pwm < 1500:
                r_pwm = 1482

        self.set_motors(
            max(1500 - limit, min(1500 + limit, l_pwm)),
            max(1500 - limit, min(1500 + limit, r_pwm)),
        )

    def encoder_callback(self, msg: Float32MultiArray):
        # 0.15 filter makes the PID ignore track jitter
        alpha = 0.15
        self.measured_left_vel = (alpha * msg.data[0]) + (
            1.0 - alpha
        ) * self.measured_left_vel
        self.measured_right_vel = (alpha * msg.data[1]) + (
            1.0 - alpha
        ) * self.measured_right_vel

        # Odom
        v = (
            ((self.measured_left_vel + self.measured_right_vel) / 2.0) / self.gear_ratio
        ) * self.wheel_radius
        omega = (
            ((self.measured_right_vel - self.measured_left_vel) / self.gear_ratio)
            * self.wheel_radius
        ) / self.wheel_base

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt > 0:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += omega * dt
            self.publish_odometry(v, omega, now)
        self.last_time = now

    def publish_odometry(self, v, omega, timestamp):
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = timestamp.to_msg(), "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        q = quaternion_from_euler(0, 0, self.theta)
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = q
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, omega
        self.odom_pub.publish(odom)

    def shutdown(self):
        self.set_motors(1500, 1500)
        self.pca.deinit()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
