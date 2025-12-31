#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray


class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')

        # Publishers - use /cmd_vel_input so obstacle_avoidance can filter
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_input', 10)
        self.auger_pub = self.create_publisher(Float64, '/auger_speed', 10)
        self.servo_pub = self.create_publisher(Float64MultiArray, '/servo_cmd', 10)

        # Subscribers for all 6 ultrasonic sensors
        self.distances = {i: 5.0 for i in range(6)}  # Initialize all to max range

        for i in range(6):
            self.create_subscription(
                Range,
                f'/ultrasound_{i}',
                lambda msg, sensor_id=i: self.ultrasonic_callback(sensor_id, msg),
                10,
            )

        # Subscribe to IMU heading for straight-line driving
        self.heading = 0.0
        self.target_heading = 0.0
        self.create_subscription(
            Float64,
            '/heading',
            self.heading_callback,
            10
        )

        # Autonomous state
        self.enabled = True
        self.forward_speed = 0.5  # m/s - increased from 0.3
        self.turn_speed = 0.8     # rad/s - increased from 0.5

        # Decision thresholds
        self.turn_distance = 1.2   # Start turning if object closer than this
        self.slow_distance = 1.5   # Slow down if object within this

        # Raise lift servos at startup for testing (run once)
        self.lift_raised = False
        self.lift_timer = self.create_timer(1.0, self.raise_lift)

        # Control timer - 10 Hz
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Autonomous controller ready!')
        self.get_logger().info('Publishing to /cmd_vel_input (filtered by obstacle_avoidance)')
        self.get_logger().info('Sensor layout: 0=L/F, 1=R/F, 2=L/C, 3=R/C, 4=L/R, 5=R/R')

    def raise_lift(self):
        """Raise lift servos to fully raised position for testing (run once)."""
        if self.lift_raised:
            return
        self.lift_raised = True

        # Your servo_node expects Float64MultiArray:
        # data[0] = lift in [0.0, 1.0]  (0.0 = fully raised, 1.0 = fully lowered)
        # data[1] = chute command in [-1.0, 1.0]
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]  # lift=0.0 (fully raised), chute=0.0 (stop)
        self.servo_pub.publish(msg)
        self.get_logger().info('Lift servos RAISED (lift=0.0)')

        # Stop this timer so it only runs once
        self.lift_timer.cancel()

    def heading_callback(self, msg):
        """Store current heading from IMU."""
        self.heading = msg.data

    def ultrasonic_callback(self, sensor_id, msg):
        """Store latest sensor reading."""
        self.distances[sensor_id] = msg.range

    def control_loop(self):
        """Main autonomous control logic."""
        if not self.enabled:
            self.stop()
            return

        # Get sensor readings
        lf = self.distances[0]  # Left Front
        rf = self.distances[1]  # Right Front
        lc = self.distances[2]  # Left Center
        rc = self.distances[3]  # Right Center
        lr = self.distances[4]  # Left Rear
        rr = self.distances[5]  # Right Rear

        # Closest front sensor
        front_min = min(lf, rf)

        cmd = Twist()

        # OBSTACLE AHEAD - turn to avoid
        if front_min < self.turn_distance:
            self.get_logger().info(f'Obstacle at {front_min:.2f}m - turning')
            cmd.linear.x = 0.2  # Move slowly forward (increased from 0.1)

            # Turn away from the closer obstacle
            if lf < rf:
                cmd.angular.z = -self.turn_speed  # Turn right
                self.get_logger().info('Turning RIGHT')
            else:
                cmd.angular.z = self.turn_speed   # Turn left
                self.get_logger().info('Turning LEFT')
            
            # Update target heading when turning
            self.target_heading = self.heading

        # SLOW APPROACH - object in range but not immediate
        elif front_min < self.slow_distance:
            cmd.linear.x = self.forward_speed * 0.6  # 60% speed
            cmd.angular.z = 0.0
            self.get_logger().info(f'Slowing - object at {front_min:.2f}m')

        # CLEAR PATH - full speed ahead with heading stabilization
        else:
            cmd.linear.x = self.forward_speed
            
            # Use IMU heading to maintain straight line
            heading_error = self.heading - self.target_heading
            
            # Normalize angle to [-180, 180]
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360
            
            # Proportional correction to maintain heading
            cmd.angular.z = -heading_error * 0.02  # Gentle correction
            
            # Minor course corrections based on side sensors
            if lc < 1.0 and rc > 1.5:
                cmd.angular.z -= 0.15  # Drift right
            elif rc < 1.0 and lc > 1.5:
                cmd.angular.z += 0.15  # Drift left

        # Publish command (obstacle_avoidance will filter for safety)
        self.cmd_vel_pub.publish(cmd)

        # Run auger while moving forward
        if cmd.linear.x > 0.05:
            auger_msg = Float64()
            auger_msg.data = 0.5  # 50% auger speed
            self.auger_pub.publish(auger_msg)
        else:
            auger_msg = Float64()
            auger_msg.data = 0.0
            self.auger_pub.publish(auger_msg)

    def stop(self):
        """Stop all motion."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        auger_msg = Float64()
        auger_msg.data = 0.0
        self.auger_pub.publish(auger_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
