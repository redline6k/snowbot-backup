#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
import busio
import adafruit_tca9548a
import time
import math


class EncoderNode(Node):
    def __init__(self):
        super().__init__("encoder_node")

        # Initialize I2C and multiplexer
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tca = adafruit_tca9548a.TCA9548A(i2c)

        # Publisher for wheel velocities [left_vel, right_vel] in rad/s
        self.velocity_pub = self.create_publisher(
            Float32MultiArray, "/wheel_velocities", 10
        )

        # Previous angle and time for velocity calculation
        self.prev_left_angle = None
        self.prev_right_angle = None
        self.prev_time = None

        # Accumulated total rotation (for multi-revolution tracking)
        self.left_total_angle = 0.0
        self.right_total_angle = 0.0

        # Noise filter threshold (rad/s) - ignore velocities below this
        self.velocity_deadband = 0.1

        # Timer for reading encoders at 50 Hz
        self.timer = self.create_timer(0.02, self.read_encoders)

        self.get_logger().info("Encoder node started - publishing to /wheel_velocities")

    def read_as5600_angle(self, channel):
        """Read raw angle from AS5600 (0-4095 for 0-360°)"""
        AS5600_ADDR = 0x36
        ANGLE_HIGH = 0x0C

        angle_bytes = bytearray(2)

        while not channel.try_lock():
            pass

        try:
            channel.writeto(AS5600_ADDR, bytes([ANGLE_HIGH]))
            channel.readfrom_into(AS5600_ADDR, angle_bytes)
        finally:
            channel.unlock()

        raw_angle = (angle_bytes[0] << 8) | angle_bytes[1]
        return raw_angle / 4096.0 * 2 * math.pi  # Convert to radians (0-2π)

    def read_encoders(self):
        """Read both encoders and calculate velocities"""
        current_time = time.time()

        try:
            # Read current angles (in radians)
            left_angle = self.read_as5600_angle(self.tca[0])
            right_angle = self.read_as5600_angle(self.tca[1])

            if self.prev_left_angle is not None:
                dt = current_time - self.prev_time

                # Calculate angular change, handling 0-2π wraparound
                left_delta = left_angle - self.prev_left_angle
                right_delta = right_angle - self.prev_right_angle

                # Handle wraparound
                if left_delta > math.pi:
                    left_delta -= 2 * math.pi
                elif left_delta < -math.pi:
                    left_delta += 2 * math.pi

                if right_delta > math.pi:
                    right_delta -= 2 * math.pi
                elif right_delta < -math.pi:
                    right_delta += 2 * math.pi

                # Accumulate total rotation
                self.left_total_angle += left_delta
                self.right_total_angle += right_delta

                # Calculate velocities in rad/s (motor shaft speed)
                left_vel = left_delta / dt
                right_vel = right_delta / dt

                # Apply deadband filter to remove noise
                if abs(left_vel) < self.velocity_deadband:
                    left_vel = 0.0
                if abs(right_vel) < self.velocity_deadband:
                    right_vel = 0.0

                # Publish velocities
                msg = Float32MultiArray()
                msg.data = [left_vel, -right_vel]  # Flip right for ROS convention
                self.velocity_pub.publish(msg)

            # Update previous values
            self.prev_left_angle = left_angle
            self.prev_right_angle = right_angle
            self.prev_time = current_time

        except Exception as e:
            self.get_logger().error(f"Encoder read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
