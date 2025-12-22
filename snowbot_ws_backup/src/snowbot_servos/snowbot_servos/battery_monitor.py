#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import board
import adafruit_ina219


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Initialize INA219
        i2c = board.I2C()
        self.ina219 = adafruit_ina219.INA219(i2c, addr=0x41)
        
        # Battery parameters
        self.declare_parameter('min_voltage', 6.0)  # 2S LiPo minimum (3.0V per cell)
        self.declare_parameter('max_voltage', 8.4)  # 2S LiPo maximum (4.2V per cell)
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        self.min_voltage = self.get_parameter('min_voltage').value
        self.max_voltage = self.get_parameter('max_voltage').value
        
        # Publisher
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        
        # Timer for periodic publishing
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_battery_state)
        
        self.get_logger().info(f'Battery monitor ready on address 0x41')

    def publish_battery_state(self):
        try:
            # Read from INA219
            voltage = self.ina219.bus_voltage
            current = self.ina219.current / 1000.0  # Convert mA to A
            power = self.ina219.power
            
            # Calculate percentage (simple linear estimation)
            percentage = max(0.0, min(1.0, 
                (voltage - self.min_voltage) / (self.max_voltage - self.min_voltage)))
            
            # Create BatteryState message
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.voltage = voltage
            msg.current = -current  # Negative = discharging
            msg.percentage = percentage
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            msg.present = True
            
            # Check for low battery
            if voltage < self.min_voltage + 0.5:
                self.get_logger().warn(f'⚠️  LOW BATTERY: {voltage:.2f}V ({percentage*100:.0f}%)')
            
            self.battery_pub.publish(msg)
            self.get_logger().info(f'Battery: {voltage:.2f}V, {current:.2f}A, {power:.2f}W, {percentage*100:.0f}%')
            
        except Exception as e:
            self.get_logger().error(f'Failed to read INA219: {str(e)}')


def main():
    rclpy.init()
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down battery monitor...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
