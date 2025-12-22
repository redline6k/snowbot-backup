#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, FluidPressure
import board
import adafruit_bmp280


class WeatherMonitorNode(Node):
    def __init__(self):
        super().__init__('weather_monitor')
        
        # Initialize BMP280
        i2c = board.I2C()
        self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
        self.bmp280.sea_level_pressure = 1013.25
        
        # Parameters
        self.declare_parameter('publish_rate', 1.0)  # Hz
        rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '/weather/temperature', 10)
        self.pressure_pub = self.create_publisher(FluidPressure, '/weather/pressure', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_weather)
        
        self.get_logger().info('Weather monitor ready (BMP280 at 0x76)')

    def publish_weather(self):
        try:
            # Read from BMP280
            temp_c = self.bmp280.temperature
            temp_f = temp_c * 9/5 + 32
            pressure_hpa = self.bmp280.pressure
            pressure_pa = pressure_hpa * 100  # Convert to Pascals for ROS
            
            # Temperature message
            temp_msg = Temperature()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = 'base_link'
            temp_msg.temperature = temp_c
            temp_msg.variance = 0.5  # ±0.5°C typical accuracy
            
            # Pressure message
            pressure_msg = FluidPressure()
            pressure_msg.header.stamp = self.get_clock().now().to_msg()
            pressure_msg.header.frame_id = 'base_link'
            pressure_msg.fluid_pressure = pressure_pa
            pressure_msg.variance = 100.0  # ±1 hPa = 100 Pa
            
            self.temp_pub.publish(temp_msg)
            self.pressure_pub.publish(pressure_msg)
            
            self.get_logger().info(f'Weather: {temp_c:.1f}°C ({temp_f:.1f}°F), {pressure_hpa:.1f} hPa')
            
        except Exception as e:
            self.get_logger().error(f'Failed to read BMP280: {str(e)}')


def main():
    rclpy.init()
    node = WeatherMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down weather...')
