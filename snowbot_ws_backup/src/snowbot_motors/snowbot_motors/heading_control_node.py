#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import MagneticField
import math
from simple_pid import PID  # pip install simple-pid


class HeadingControlNode(Node):
    def __init__(self):
        super().__init__("heading_control_node")

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_cb, 10
        )  # From teleop/waypoints
        self.mag_sub = self.create_subscription(
            MagneticField, "/rm3100/mag", self.mag_cb, 10
        )

        # Publisher → motor_node
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_corrected", 10)

        # PID: heading error → angular correction
        self.pid = PID(1.5, 0.1, 0.3, setpoint=0.0, sample_time=0.01)  # rad/s output
        self.pid.output_limits = (-1.5, 1.5)  # Max turn rate

        # State
        self.current_heading = 0.0
        self.desired_heading = 0.0  # Default straight (0=N)
        self.incoming_twist = Twist()
        self.heading_received = False

        self.get_logger().info(
            "HeadingControl: /cmd_vel_in → PID yaw → /cmd_vel_corrected"
        )

    def mag_cb(self, msg):
        # Compute yaw from mag: atan2(-my, mx) for ENU convention
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        self.current_heading = math.atan2(-my, mx)  # radians [-pi,pi]
        self.heading_received = True

    def cmd_vel_cb(self, msg):
        self.incoming_twist = msg
        self.publish_corrected()

    def publish_corrected(self):
        if not self.heading_received:
            self.cmd_vel_pub.publish(self.incoming_twist)
            return

        # Heading error (shortest angle)
        error = self.normalize_angle(self.desired_heading - self.current_heading)

        # PID computes angular correction
        angular_correction = self.pid(error)

        # Apply to incoming cmd_vel
        corrected = Twist()
        corrected.linear.x = self.incoming_twist.linear.x
        corrected.angular.z = self.incoming_twist.angular.z + angular_correction

        # Saturate
        corrected.angular.z = max(-2.0, min(2.0, corrected.angular.z))

        self.cmd_vel_pub.publish(corrected)
        self.get_logger().debug(
            f"Heading E:{math.degrees(error):.1f}° PID:{angular_correction:.2f} → Z:{corrected.angular.z:.2f}"
        )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    rclpy.init()
    node = HeadingControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
