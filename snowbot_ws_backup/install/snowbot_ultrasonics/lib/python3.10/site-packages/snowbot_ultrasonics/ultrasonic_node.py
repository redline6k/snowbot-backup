#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import smbus2


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # I2C bus and Nano address
        self.bus = smbus2.SMBus(1)
        self.nano_addr = 0x08

        # 6 publishers
        self.pubs = [self.create_publisher(Range, f'/ultrasound_{i}', 10) for i in range(6)]

        # Read every 100 ms
        self.timer = self.create_timer(2.0, self.read_distances)

        self.get_logger().info('Ultrasonic Nano I2C reader READY – distances incoming!')

    def read_distances(self):
        try:
            # Read 12 bytes from Nano (6 × 2 bytes)
            data = self.bus.read_i2c_block_data(self.nano_addr, 0, 12)

            for i in range(6):
                low = data[i*2]
                high = data[i*2 + 1]
                distance_cm = (high << 8) | low
                if distance_cm == 999:
                    distance_m = float('inf')
                else:
                    distance_m = distance_cm / 100.0

                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'ultrasound_{i}'
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 0.26
                msg.min_range = 0.02
                msg.max_range = 4.0
                msg.range = distance_m

                self.pubs[i].publish(msg)

                if distance_m < 4.0:
                    self.get_logger().info(f'Sensor {i}: {distance_m:.2f} m')

        except Exception as e:
            self.get_logger().warn(f'I2C read failed: {e}')


def main():
    rclpy.init()
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
