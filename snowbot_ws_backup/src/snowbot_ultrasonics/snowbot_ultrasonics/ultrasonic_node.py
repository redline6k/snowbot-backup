#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import smbus2
from collections import deque
import statistics


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # I2C bus and Nano address
        self.bus = smbus2.SMBus(1)
        self.nano_addr = 0x08

        # 6 publishers
        self.pubs = [self.create_publisher(Range, f'/ultrasound_{i}', 10) for i in range(6)]

        # Read every 100 ms (10 Hz for better obstacle detection)
        self.timer = self.create_timer(0.1, self.read_distances)

        # Median filter buffers for noise reduction
        self.reading_buffers = [deque(maxlen=5) for _ in range(6)]
        
        # For throttled logging
        self.last_distances = [float('inf')] * 6
        self.log_threshold = 0.1  # Log only if changed by 10cm

        self.get_logger().info('Ultrasonic Nano I2C reader READY – distances incoming!')

    def read_distances(self):
        max_retries = 3
        data = None
        
        # Retry logic for I2C failures
        for attempt in range(max_retries):
            try:
                # Read 12 bytes from Nano (6 × 2 bytes)
                data = self.bus.read_i2c_block_data(self.nano_addr, 0, 12)
                break
            except Exception as e:
                if attempt == max_retries - 1:
                    self.get_logger().warn(f'I2C read failed after {max_retries} attempts: {e}')
                    return
                continue
        
        if data is None:
            return

        # Single timestamp for all sensors in this cycle
        now = self.get_clock().now().to_msg()

        for i in range(6):
            low = data[i*2]
            high = data[i*2 + 1]
            distance_cm = (high << 8) | low
            
            if distance_cm == 999:
                distance_m = float('inf')
            else:
                distance_m = distance_cm / 100.0

            # Apply median filter for noise reduction
            self.reading_buffers[i].append(distance_m)
            if len(self.reading_buffers[i]) >= 3:
                filtered_distance = statistics.median(self.reading_buffers[i])
            else:
                filtered_distance = distance_m

            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = f'ultrasound_{i}'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26  # Adjust based on your sensor specs
            msg.min_range = 0.02
            msg.max_range = 4.0
            msg.range = filtered_distance

            self.pubs[i].publish(msg)

            # Throttled logging - only log significant changes
            if filtered_distance < 4.0 and abs(filtered_distance - self.last_distances[i]) > self.log_threshold:
                self.get_logger().info(f'Sensor {i}: {filtered_distance:.2f} m')
                self.last_distances[i] = filtered_distance


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
