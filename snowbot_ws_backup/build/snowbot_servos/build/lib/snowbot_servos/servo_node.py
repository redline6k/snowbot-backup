#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from adafruit_servokit import ServoKit
import time


class SnowbotNode(Node):
    def __init__(self):
        super().__init__('snowbot_node')

        self.kit = ServoKit(channels=16, address=0x40)
        self.kit._pca.frequency = 50

        # Extended range for lift servos only (0 & 1)
        for ch in [0, 1]:
            self.kit.servo[ch].actuation_range = 240
            self.kit.servo[ch].set_pulse_width_range(500, 2500)

        # ESC on channel 3
        self.kit.servo[3].set_pulse_width_range(1000, 2000)

        # SAFETY: force ESC off on startup
        self.kit.servo[3].angle = 90
        time.sleep(0.5)

        # YOUR FINAL LIFT CALIBRATION
        self.raised_ch0, self.raised_ch1 = 120, 200
        self.lowered_ch0, self.lowered_ch1 = 230, 70

        # Only subscribe to lift command (single value)
        self.create_subscription(Float64MultiArray, '/servo_cmd', self.servo_cb, 10)
        self.create_subscription(Float64, '/auger_speed', self.auger_cb, 10)

        self.get_logger().info('Lift (0/1) + Auger ESC (3) READY – CHUTE DISABLED FOR TESTING')

    def lift_cb(self, msg):
        lift = max(0.0, min(1.0, msg.data))

        ch0 = self.raised_ch0 + lift * (self.lowered_ch0 - self.raised_ch0)
        ch1 = self.raised_ch1 + lift * (self.lowered_ch1 - self.raised_ch1)
        self.kit.servo[0].angle = ch0
        self.kit.servo[1].angle = ch1

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
        pass
    finally:
        node.kit.servo[3].angle = 90  # ESC off
        rclpy.shutdown()


if __name__ == '__main__':
    main()
