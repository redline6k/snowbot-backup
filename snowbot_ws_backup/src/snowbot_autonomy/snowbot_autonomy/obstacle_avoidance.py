#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Safety threshold (meters)
        self.safety_distance = 0.5
        
        # Subscribe to all 6 ultrasonic sensors
        self.sensors = {}
        for i in range(6):
            self.create_subscription(
                Range,
                f'/ultrasound_{i}',
                lambda msg, idx=i: self.sensor_callback(msg, idx),
                10
            )
            self.sensors[i] = float('inf')
        
        # Subscribe to cmd_vel input (from teleop or autonomous)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_input',
            self.cmd_vel_callback,
            10
        )
        
        # Publish safe cmd_vel output to motors
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Obstacle avoidance ACTIVE - safety distance: 0.5m')
    
    def sensor_callback(self, msg, sensor_id):
        """Update sensor reading"""
        self.sensors[sensor_id] = msg.range
    
    def cmd_vel_callback(self, msg):
        """Check for obstacles before forwarding commands"""
        # Get minimum distances
        front_sensors = [self.sensors.get(0, float('inf')), 
                        self.sensors.get(1, float('inf'))]
        side_sensors = [self.sensors.get(2, float('inf')), 
                       self.sensors.get(3, float('inf'))]
        rear_sensors = [self.sensors.get(4, float('inf')), 
                       self.sensors.get(5, float('inf'))]
        
        min_front = min(front_sensors)
        min_side = min(side_sensors)
        min_rear = min(rear_sensors)
        
        safe_cmd = Twist()
        
        # Forward motion - check front sensors
        if msg.linear.x > 0 and min_front < self.safety_distance:
            self.get_logger().warn(f'OBSTACLE FRONT: {min_front:.2f}m - STOPPING')
            safe_cmd.linear.x = 0.0
        else:
            safe_cmd.linear.x = msg.linear.x
        
        # Reverse motion - check rear sensors
        if msg.linear.x < 0 and min_rear < self.safety_distance:
            self.get_logger().warn(f'OBSTACLE REAR: {min_rear:.2f}m - STOPPING')
            safe_cmd.linear.x = 0.0
        
        # Allow rotation always (for escaping)
        safe_cmd.angular.z = msg.angular.z
        
        self.cmd_vel_pub.publish(safe_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
