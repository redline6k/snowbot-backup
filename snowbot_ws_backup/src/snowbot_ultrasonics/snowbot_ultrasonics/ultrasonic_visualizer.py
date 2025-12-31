#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class UltrasonicVisualizer(Node):
    def __init__(self):
        super().__init__('ultrasonic_visualizer')
        
        # Publisher for markers
        self.marker_pub = self.create_publisher(MarkerArray, '/ultrasonic_markers', 10)
        
        # Store latest readings
        self.readings = {}
        
        # Subscribe to all ultrasonic sensors
        for i in range(6):  # Sensors 0-5
            self.create_subscription(
                Range,
                f'/ultrasound_{i}',
                lambda msg, sensor_id=i: self.ultrasonic_callback(sensor_id, msg),
                10
            )
        
        # Sensor positions and orientations relative to robot base
        # [x_meters, y_meters, yaw_degrees]
        # x+ = forward, y+ = left, yaw: 0=forward, 90=left, 180=back, -90=right
        self.sensor_poses = {
            0: [0.30, 0.20, 0],    # Left Front
            1: [0.30, -0.20, 0],  # Right Front
            2: [0.0, 0.20, 90],     # Left Center
            3: [0.0, -0.20, -90],   # Right Center
            4: [-0.30, 0.20, 180],  # Left Rear
            5: [-0.30, -0.20, 180],# Right Rear
        }
        
        # Timer to publish markers
        self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('Ultrasonic visualizer ready')
    
    def ultrasonic_callback(self, sensor_id, msg):
        """Store ultrasonic reading"""
        self.readings[sensor_id] = msg.range
    
    def publish_markers(self):
        """Publish visualization markers for all sensors"""
        marker_array = MarkerArray()
        
        for sensor_id, distance in self.readings.items():
            if sensor_id not in self.sensor_poses:
                continue
            
            x, y, yaw_deg = self.sensor_poses[sensor_id]
            
            # Create cone marker for detection range
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ultrasonic_cones"
            marker.id = sensor_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Position at sensor location
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            
            # Orientation based on sensor direction
            import math
            yaw_rad = math.radians(yaw_deg)
            marker.pose.orientation.z = math.sin(yaw_rad / 2)
            marker.pose.orientation.w = math.cos(yaw_rad / 2)
            
            # Scale arrow based on distance reading
            marker.scale.x = distance  # Length
            marker.scale.y = 0.05      # Shaft diameter
            marker.scale.z = 0.05      # Head diameter
            
            # Color based on distance (red=close, green=far)
            if distance < 0.5:
                marker.color.r = 1.0
                marker.color.g = 0.0
            elif distance < 1.0:
                marker.color.r = 1.0
                marker.color.g = 0.5
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 0  # Don't auto-delete
            
            marker_array.markers.append(marker)
            
            # Add text label with distance
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "ultrasonic_labels"
            text_marker.id = sensor_id + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.3
            
            text_marker.text = f"S{sensor_id}: {distance:.2f}m"
            text_marker.scale.z = 0.1  # Text height
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
