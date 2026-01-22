#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, MagneticField
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math


def normalize_angle(angle_deg):
    angle_deg = angle_deg % 360.0
    if angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg


class SnowbotWaypoint(Node):
    def __init__(self):
        super().__init__("snowbot_waypoint_follower")

        # Fall River waypoints
        self.waypoints = [
            (41.67756, -71.18458966666667),  # WP0: Current position
            (41.67756, -71.184680),  # WP1: 10m West
        ]

        self.current_wp_idx = 0
        self.current_lat = None
        self.current_lon = None
        self.current_heading = 0.0

        self.waypoint_mode_pub = self.create_publisher(Bool, "/motor_waypoint_mode", 10)
        self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gps_cb, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, "/rm3100/mag", self.mag_cb, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.10, self.timer_cb)

        # Tuning parameters - snow fixes
        self.wp_tolerance = 1.0  # meters (was 3.0, tighter but stable)
        self.max_lin = 0.25  # m/s snow traction (was 0.45)
        self.min_lin = 0.10
        self.max_ang = 0.40  # rad/s (was 1.2)
        self.wp_decel_dist = 3.0  # ramp down before tolerance

        # Activate motors
        mode_msg = Bool()
        mode_msg.data = True
        self.waypoint_mode_pub.publish(mode_msg)
        self.get_logger().info("Snowbot Waypoint READY - GPS+RM3100")

    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def mag_cb(self, msg):
        mx = msg.magnetic_field.x * 1e6  # nT → μT
        my = msg.magnetic_field.y * 1e6
        if abs(mx) > 400 or abs(my) > 400:
            return
        heading = math.degrees(math.atan2(my, mx))  # Calibrated axes
        heading += 18.0  # Your North offset
        self.current_heading = normalize_angle(heading)
        self.get_logger().info(f"Mag[{mx:.0f},{my:.0f}] H:{self.current_heading:.1f}°")

    def timer_cb(self):
        if self.current_lat is None or self.current_lon is None:
            return  # Silent skip

        wp_lat, wp_lon = self.waypoints[self.current_wp_idx]
        dlat = math.radians(wp_lat - self.current_lat)
        dlon = math.radians(wp_lon - self.current_lon)
        y = math.sin(dlon) * math.cos(math.radians(wp_lat))
        x = math.cos(math.radians(self.current_lat)) * math.sin(dlat) - math.sin(
            math.radians(self.current_lat)
        ) * math.cos(math.radians(wp_lat)) * math.cos(dlon)
        bearing = math.degrees(math.atan2(y, x))
        bearing = normalize_angle(bearing)

        heading_error = normalize_angle(bearing - self.current_heading)
        dist = self.haversine(self.current_lat, self.current_lon, wp_lat, wp_lon)

        self.get_logger().info(
            f"WP{self.current_wp_idx} D:{dist:.1f}m E:{heading_error:+.1f}° [{self.current_heading:.0f}°→{bearing:.0f}°]"
        )

        if dist < self.wp_tolerance:
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            self.get_logger().info(
                f"WP{self.current_wp_idx-1} REACHED! → WP{self.current_wp_idx}"
            )
            return

        twist = Twist()
        # Base commands
        twist.linear.x = min(self.max_lin, dist / 15.0)  # gentler ramp
        twist.angular.z = heading_error / 45.0
        # Clamp limits
        twist.linear.x = max(self.min_lin, min(twist.linear.x, self.max_lin))
        twist.angular.z = max(-self.max_ang, min(twist.angular.z, self.max_ang))
        # Decel near WP
        if dist < self.wp_decel_dist:
            twist.linear.x *= dist / self.wp_decel_dist
        self.get_logger().info(
            f"PUBLISH lin:{twist.linear.x:.2f} ang:{twist.angular.z:.2f}"
        )
        self.cmd_pub.publish(twist)

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi, dlambda = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
        a = (
            math.sin(dphi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        )
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def main(args=None):
    rclpy.init(args=args)
    node = SnowbotWaypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
