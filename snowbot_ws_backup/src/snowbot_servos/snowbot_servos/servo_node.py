#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO
import time


class SnowbotNode(Node):
    def __init__(self):
        super().__init__('snowbot_node')

        self.kit = ServoKit(channels=16, address=0x40)
        self.kit._pca.frequency = 50

        # Extended range for lift and chute
        for ch in [0, 1, 2]:
            self.kit.servo[ch].actuation_range = 240
            self.kit.servo[ch].set_pulse_width_range(500, 2500)

        # ESC on channel 3
        self.kit.servo[3].set_pulse_width_range(1000, 2000)

        # End stops (normally closed — HIGH = hit, LOW = not hit)
        self.left_stop = 23
        self.right_stop = 24
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.left_stop, self.right_stop], GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Calibration
        self.raised_ch0, self.raised_ch1 = 120, 200
        self.lowered_ch0, self.lowered_ch1 = 230, 70
        self.chute_stop = 120
        self.chute_max = 30

        # Safety startup
        self.kit._pca.channels[2].duty_cycle = 0
        time.sleep(0.5)
        self.kit.servo[2].angle = self.chute_stop
        self.get_logger().info(f'Chute servo set to stop position: {self.chute_stop}')
        
        self.kit.servo[3].angle = 90

        self.create_subscription(Float64MultiArray, '/servo_cmd', self.servo_cb, 10)
        self.create_subscription(Float64, '/auger_speed', self.auger_cb, 10)

        self.get_logger().info('Lift + Continuous Chute (with end stops) + Auger ESC READY')

    def servo_cb(self, msg):
        if len(msg.data) < 2:
            return
        lift = max(0.0, min(1.0, msg.data[0]))
        chute_raw = max(-1.0, min(1.0, msg.data[1]))

        # Lift
        ch0 = self.raised_ch0 + lift * (self.lowered_ch0 - self.raised_ch0)
        ch1 = self.raised_ch1 + lift * (self.lowered_ch1 - self.raised_ch1)
        self.kit.servo[0].angle = ch0
        self.kit.servo[1].angle = ch1

        # Read and log endstop state EVERY time
        left_state = GPIO.input(self.left_stop)
        right_state = GPIO.input(self.right_stop)
        left_hit = left_state == GPIO.HIGH
        right_hit = right_state == GPIO.HIGH

        # Always log GPIO states when chute is commanded to move
        self.get_logger().info(f'Chute: {chute_raw:.2f} | Left GPIO: {left_state} ({left_hit}) | Right GPIO: {right_state} ({right_hit})')

        # Chute with end stop safety
        if chute_raw > 0.05 and left_hit:
            target = self.chute_stop
            self.get_logger().warn('LEFT ENDSTOP HIT - stopping chute')
        elif chute_raw < -0.05 and right_hit:
            target = self.chute_stop
            self.get_logger().warn('RIGHT ENDSTOP HIT - stopping chute')
        elif abs(chute_raw) > 0.05:
            # Only move if command is significant
            target = self.chute_stop + (chute_raw * self.chute_max)
            self.get_logger().info(f'Moving chute to: {target:.1f}')
        else:
            # Stop when no command
            target = self.chute_stop

        self.kit.servo[2].angle = target

    def auger_cb(self, msg):
        speed = max(0.0, min(1.0, msg.data))
        angle = 90 + speed * 90
        self.kit.servo[3].angle = angle


def main():
    rclpy.init()
    node = SnowbotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Stop everything
        node.kit._pca.channels[2].duty_cycle = 0
        node.kit.servo[3].angle = 90
        GPIO.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
