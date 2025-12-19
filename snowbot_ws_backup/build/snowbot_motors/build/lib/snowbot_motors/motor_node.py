import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        GPIO.setmode(GPIO.BCM)
        # Left motor now on right channel (IN3/IN4/ENB)
        self.left_in1 = 27  # IN3
        self.left_in2 = 22  # IN4
        self.left_pwm_pin = 13  # ENB
        # Right motor now on left channel (IN1/IN2/ENA)
        self.right_in3 = 17 # IN1
        self.right_in4 = 18 # IN2
        self.right_pwm_pin = 12 # ENA

        GPIO.setup([self.left_in1, self.left_in2, self.right_in3, self.right_in4], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([self.left_pwm_pin, self.right_pwm_pin], GPIO.OUT)
        self.left_pwm = GPIO.PWM(self.left_pwm_pin, 1000)
        self.right_pwm = GPIO.PWM(self.right_pwm_pin, 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

        self.set_motor_direction(self.left_in1, self.left_in2, 0)
        self.set_motor_direction(self.right_in3, self.right_in4, 0)

        self.get_logger().info('Tracks ready – motors swapped, direction fixed')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        angular_z = -angular_z   # ← FLIPS TURN DIRECTION (RIGHT = +0.5, LEFT = -0.5)

        turn_scale = 0.8
        left_speed = linear_x - (angular_z * turn_scale)
        right_speed = linear_x + (angular_z * turn_scale)
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        # Flip sign so forward command goes forward
        # Flip forward/reverse (already done)
        left_speed = -left_speed
        right_speed = -right_speed

        # Flip turn direction so +angular.z = right turn
        angular_z = -angular_z

        self.get_logger().info(f'Left: {left_speed:.2f}, Right: {right_speed:.2f}')
        self.set_motor(self.left_in1, self.left_in2, self.left_pwm, left_speed)
        self.set_motor(self.right_in3, self.right_in4, self.right_pwm, right_speed)

    def set_motor(self, in_forward, in_reverse, pwm, speed):
        if speed > 0:
            self.set_motor_direction(in_forward, in_reverse, 1)
            pwm.ChangeDutyCycle(speed * 100)
        elif speed < 0:
            self.set_motor_direction(in_forward, in_reverse, -1)
            pwm.ChangeDutyCycle(abs(speed) * 100)
        else:
            self.set_motor_direction(in_forward, in_reverse, 0)
            pwm.ChangeDutyCycle(0)

    def set_motor_direction(self, in_forward, in_reverse, direction):
        if direction == 1:  # Forward
            GPIO.output(in_forward, GPIO.HIGH)
            GPIO.output(in_reverse, GPIO.LOW)
        elif direction == -1:  # Reverse
            GPIO.output(in_forward, GPIO.LOW)
            GPIO.output(in_reverse, GPIO.HIGH)
        else:
            GPIO.output(in_forward, GPIO.LOW)
            GPIO.output(in_reverse, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left_pwm.stop()
        node.right_pwm.stop()
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
