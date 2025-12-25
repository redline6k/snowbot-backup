#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import time


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Publish to cmd_vel_input (obstacle avoidance will forward to /cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_input', 10)
        
        # Speed settings
        self.linear_speed = 0.3
        self.angular_speed = 0.5

        # Timeout-based auto stop
        self.last_cmd_time = time.time()
        self.timeout = 0.5  # seconds without keypress before auto-stop
        self.timer = self.create_timer(0.1, self.timeout_callback)
        
        self.get_logger().info('Teleop Keyboard Ready!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/S - Forward/Backward')
        self.get_logger().info('  A/D - Turn Left/Right')
        self.get_logger().info('  SPACE - Emergency Stop')
        self.get_logger().info('  Q - Quit')
        
    def get_key(self):
        """Read a single keypress (blocking)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def timeout_callback(self):
        """Send zero Twist if no key pressed recently."""
        if time.time() - self.last_cmd_time > self.timeout:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

    def run(self):
        """Main control loop."""
        twist = Twist()

        try:
            while rclpy.ok():
                key = self.get_key().lower()

                twist.linear.x = 0.0
                twist.angular.z = 0.0

                if key == 'w':
                    twist.linear.x = self.linear_speed
                    self.get_logger().info('Forward')
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                    self.get_logger().info('Backward')
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                    self.get_logger().info('Turn Left')
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                    self.get_logger().info('Turn Right')
                elif key == ' ':
                    self.get_logger().warn('EMERGENCY STOP')
                elif key == 'q':
                    self.get_logger().info('Quitting...')
                    break

                self.cmd_vel_pub.publish(twist)
                self.last_cmd_time = time.time()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Send stop command on exit
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    # Spin node in background so timers work while run() blocks on key input
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.run()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
