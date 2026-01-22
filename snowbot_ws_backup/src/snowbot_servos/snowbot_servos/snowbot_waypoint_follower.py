#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from geometry_msgs.msg import Twist
import math
import tf_transformations


def normalize_angle(angle_deg):
    angle_deg = angle_deg % 360.0
    if angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg


class SnowbotWaypoint(Node):
    def __init__(self):
        super().__init__("snowbot_waypoint_follower")
        self.get_logger().info("Snowbot Waypoint Follower READY - GPS + Mag compass")
        self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gps_cb, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, "/rm3100/mag", self.mag_cb, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.current_lat = 41.677542
        self.current_lon = -71.184699
        self.current_heading = 0.0
        self.gps_valid = False
        self.waypoint_idx = 0
        self.mag_offsets = [50.2, 87.8, -153.2]
        self.waypoints = [
            (41.677549, -71.184747),  # WP0: Current noisy GPS
            (41.677544, -71.184750),  # WP1: 5m SW
            (41.677544, -71.184742),  # WP2: 5m SE
            (41.677554, -71.184742),  # WP3: 10m N
            (41.677549, -71.184747),  # WP4: Home
        ]

    def mag_cb(self, msg):
        mx = msg.magnetic_field.x * 1e6 - self.mag_offsets[0]
        my = msg.magnetic_field.y * 1e6 - self.mag_offsets[1]
        if abs(mx) > 400 or abs(my) > 400:
            return

        heading = math.degrees(math.atan2(my, mx))
        self.current_heading = normalize_angle(heading)
        self.get_logger().info(
            f"RM3100 Mag: [{mx:.1f},{my:.1f}] heading:{self.current_heading:6.1f}°"
        )

    def gps_cb(self, msg):
        cov = msg.position_covariance[0]
        # Accept any non-error fix (status >= 0), and keep a loose cov check
        self.gps_valid = msg.status.status >= 0 and cov < 50.0
        if self.gps_valid:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            self.get_logger().info(
                f"GPS OK cov:{cov:.4f} pos:{self.current_lat:.7f},{self.current_lon:.7f}"
            )

    def timer_cb(self):
        if not self.gps_valid:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info("No GPS fix - STOPPED")
            return
        if self.waypoint_idx >= len(self.waypoints):
            self.waypoint_idx = 0
            self.get_logger().info("Looping waypoints")
        wp_lat, wp_lon = self.waypoints[self.waypoint_idx]
        dlat = math.radians(wp_lat - self.current_lat)
        dlon = math.radians(wp_lon - self.current_lon)
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(math.radians(self.current_lat))
            * math.cos(math.radians(wp_lat))
            * math.sin(dlon / 2) ** 2
        )
        dist = 6371000.0 * 2 * math.asin(math.sqrt(a))
        y = math.sin(dlon) * math.cos(math.radians(wp_lat))
        x = math.cos(math.radians(self.current_lat)) * math.sin(
            math.radians(wp_lat)
        ) - math.sin(math.radians(wp_lat)) * math.cos(math.radians(wp_lon)) * math.cos(
            dlon
        )
        bearing = normalize_angle(math.degrees(math.atan2(y, x)))
        yaw_err = normalize_angle(bearing - self.current_heading)
        cmd = Twist()
        if dist < 5.0:
            self.get_logger().info("WAYPOINT REACHED!")
            self.waypoint_idx += 1
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = min(0.40, dist * 0.12)
            cmd.angular.z = max(min(yaw_err * 0.10, 1.0), -1.0)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"Dist:{dist:5.1f}m Err:{yaw_err:6.1f}° lin:{cmd.linear.x:.2f} ang:{cmd.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SnowbotWaypoint()
    timer = node.create_timer(0.1, node.timer_cb)
    rclpy.spin(node)
    node.destroy_timer(timer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
