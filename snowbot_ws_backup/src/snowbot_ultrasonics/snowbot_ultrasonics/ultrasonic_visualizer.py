#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class UltrasonicVisualizer(Node):
    def __init__(self):
        super().__init__('ultrasonic_visualizer')
        
        # Publisher for markers
        self.marker_pub = self.create_publisher(MarkerArray, '/ultrasonic_markers', 10)
        
        # Store latest readings with timestamps
        self.readings = {}
        self.last_update_time = {}
        
        # Subscribe to all ultrasonic sensors - fixed lambda closure issue
        for i in range(6):
            self.create_subscription(
                Range,
                f'/ultrasound_{i}',
                self.make_callback(i),
                10
            )
        
        # Sensor positions and orientations relative to robot base
        # [x_meters, y_meters, yaw_degrees]
        self.sensor_poses = {
            0: [0.30, 0.20, 0],      # Left Front
            1: [0.30, -0.20, 0],     # Right Front
            2: [0.0, 0.20, 90],      # Left Center
            3: [0.0, -0.20, -90],    # Right Center
            4: [-0.30, 0.20, 180],   # Left Rear
            5: [-0.30, -0.20, 180],  # Right Rear
        }
        
        # Timer to publish markers
        self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('Ultrasonic visualizer ready')
    
    def make_callback(self, sensor_id):
        """Create callback with proper closure"""
        def callback(msg):
            self.ultrasonic_callback(sensor_id, msg)
        return callback
    
    def ultrasonic_callback(self, sensor_id, msg):
        """Store ultrasonic reading"""
        self.readings[sensor_id] = msg.range
        self.last_update_time[sensor_id] = self.get_clock().now()
    
    def publish_markers(self):
        """Publish visualization markers for all sensors"""
        marker_array = MarkerArray()
        current_time = self.get_clock().now()
        
        for sensor_id in range(6):
            if sensor_id not in self.sensor_poses:
                continue
            
            # Check if we have recent data (within 1 second)
            if sensor_id not in self.last_update_time:
                continue
            
            time_diff = (current_time - self.last_update_time[sensor_id]).nanoseconds / 1e9
            if time_diff > 1.0:
                # Stale data - delete marker
                self.add_delete_marker(marker_array, sensor_id)
                continue
            
            distance = self.readings.get(sensor_id, float('inf'))
            x, y, yaw_deg = self.sensor_poses[sensor_id]
            
            # Skip infinite readings or clamp to max range
            if math.isinf(distance) or distance > 4.0:
                # Show faded marker for out-of-range
                distance = 0.1  # Tiny arrow
                out_of_range = True
            else:
                out_of_range = False
            
            # Create arrow marker for detection
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
            yaw_rad = math.radians(yaw_deg)
            marker.pose.orientation.z = math.sin(yaw_rad / 2)
            marker.pose.orientation.w = math.cos(yaw_rad / 2)
            
            # Scale arrow based on distance reading
            marker.scale.x = max(0.05, distance)  # Length (minimum size)
            marker.scale.y = 0.05      # Shaft diameter
            marker.scale.z = 0.05      # Head diameter
            
            # Color based on distance or out-of-range status
            if out_of_range:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 0.3
            elif distance < 0.5:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            elif distance < 1.0:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            
            marker.lifetime.sec = 0
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
            
            if out_of_range:
                text_marker.text = f"S{sensor_id}: infm"
            else:
                text_marker.text = f"S{sensor_id}: {distance:.2f}m"
            
            text_marker.scale.z = 0.1
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def add_delete_marker(self, marker_array, sensor_id):
        """Add delete markers for stale sensors"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ultrasonic_cones"
        marker.id = sensor_id
        marker.action = Marker.DELETE
        marker_array.markers.append(marker)
        
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = "ultrasonic_labels"
        text_marker.id = sensor_id + 100
        text_marker.action = Marker.DELETE
        marker_array.markers.append(text_marker)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
