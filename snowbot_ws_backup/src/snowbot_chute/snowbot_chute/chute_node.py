#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import RPi.GPIO as GPIO


class ChuteNode(Node):
    def __init__(self):
        super().__init__('chute_node')

        self.chute_pin = 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.chute_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.chute_pin, 50)  # 50 Hz
        self.pwm.start(7.5)  # stopped (1500 µs = 7.5% duty)

        # Parallax Feedback 360° exact specs
        self.stop_duty = 7.5   # 1500 µs
        self.max_delta = 1.1   # ±220 µs = ±1.1% duty for full speed

        self.create_subscription(Float64, '/chute_speed', self.callback, 10)

        self.get_logger().info('Parallax Feedback 360° Chute READY – GPIO 26 – CORRECT DUTY CYCLE')

    def callback(self, msg):
        speed = max(-1.0, min(1.0, msg.data))
        duty = self.stop_duty + (speed * self.max_delta)
        self.pwm.ChangeDutyCycle(duty)

        direction = "LEFT" if speed > 0 else "RIGHT" if speed < 0 else "STOP"
        self.get_logger().info(f'Chute {direction} — duty {duty:.2f}%')


def main():
    rclpy.init()
    node = ChuteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pwm.ChangeDutyCycle(node.stop_duty)
        node.pwm.stop()
        GPIO.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
