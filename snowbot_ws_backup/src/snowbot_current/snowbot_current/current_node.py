#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15 import ads1x15
import time


class CurrentNode(Node):
    def __init__(self):
        super().__init__('current_node')

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c, address=0x48)
        self.chan = AnalogIn(ads, ads1x15.Pin.A0)  # ← CORRECT PIN ACCESS

        # ACS758 sensitivity (mV/A) — CHANGE TO YOUR MODEL
        self.sensitivity_mv_per_a = 40.0  # e.g., 50A = 40 mV/A

        # Auto zero-calibration
        self.get_logger().info('Calibrating zero offset... (no load!)')
        time.sleep(2)
        readings = []
        for _ in range(100):
            readings.append(self.chan.voltage)
            time.sleep(0.01)
        self.zero_voltage = sum(readings) / len(readings)
        self.get_logger().info(f'Zero offset: {self.zero_voltage:.4f} V')

        self.pub = self.create_publisher(Float32, '/current', 10)
        self.create_timer(0.1, self.publish_current)

        self.get_logger().info('ACS758 current sensor READY – publishing to /current (A)')

    def publish_current(self):
        voltage = self.chan.voltage
        current = (voltage - self.zero_voltage) * 1000 / self.sensitivity_mv_per_a

        msg = Float32()
        msg.data = float(current)
        self.pub.publish(msg)

        self.get_logger().info(f'Current: {current:.2f} A (V: {voltage:.4f})')


def main():
    rclpy.init()
    node = CurrentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
