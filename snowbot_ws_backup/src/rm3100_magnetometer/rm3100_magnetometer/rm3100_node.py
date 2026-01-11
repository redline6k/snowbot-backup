#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import smbus2
import struct

class RM3100Node(Node):
    def __init__(self):
        super().__init__('rm3100_node')
        self.pub = self.create_publisher(MagneticField, '/rm3100/mag', 10)
        self.bus = smbus2.SMBus(1)
        self.addr = 0x23  # Your detected address
        
        # Initialize RM3100 - Continuous Mode 75Hz
        self.bus.write_byte_data(self.addr, 0x01, 0x79)  # CMM register: all axes
        self.bus.write_byte_data(self.addr, 0x0B, 0x96)  # TMRC: 75Hz rate
        
        self.timer = self.create_timer(0.013, self.read_mag)  # 75Hz
        self.get_logger().info('RM3100 READY at I2C 0x23, 75Hz')

    def read_mag(self):
        try:
            # Read status to check DRDY
            status = self.bus.read_byte_data(self.addr, 0x34)
            if not (status & 0x80):  # DRDY bit
                return
            
            # Read 9 bytes from 0x24 (MX, MY, MZ as 24-bit signed)
            data = self.bus.read_i2c_block_data(self.addr, 0x24, 9)
            
            # Convert 24-bit signed to int
            mx_raw = self.to_signed24(data[0:3])
            my_raw = self.to_signed24(data[3:6])
            mz_raw = self.to_signed24(data[6:9])
            
            # RM3100: ~13 LSB/μT (75 cycle count default)
            mx = mx_raw / 13.0 / 1e6  # Convert to Tesla
            my = my_raw / 13.0 / 1e6
            mz = mz_raw / 13.0 / 1e6
            
            msg = MagneticField()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'rm3100_link'
            msg.magnetic_field.x = mx
            msg.magnetic_field.y = my
            msg.magnetic_field.z = mz
            self.pub.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f'RM3100 read error: {e}')

    def to_signed24(self, data):
        val = (data[0] << 16) | (data[1] << 8) | data[2]
        if val & 0x800000:
            val -= 0x1000000
        return val

def main():
    rclpy.init()
    node = RM3100Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

