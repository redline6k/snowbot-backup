#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import board
import busio
from adafruit_icm20x import ICM20948, AccelRange, GyroRange


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Initialize I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)

        # ICM20948 @ 0x68
        try:
            self.icm = ICM20948(i2c, address=0x68)
            self.icm.accelerometer_range = AccelRange.RANGE_4G
            self.icm.gyro_range = GyroRange.RANGE_2000_DPS
            self.get_logger().info('ICM20948 initialized @ 0x68')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ICM20948: {e}')
            raise

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Timer - 100Hz update rate
        self.timer = self.create_timer(0.01, self.publish_data)

        self.get_logger().info('IMU node ready – ICM20948 publishing on /imu/data and /imu/mag')

    def publish_data(self):
        # Read sensor data with X/Y swapped
        accel = self.icm.acceleration
        gyro = self.icm.gyro
        mag = self.icm.magnetic

        # Publish IMU data (accel + gyro)
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Linear acceleration (m/s²) - X/Y swapped
        imu_msg.linear_acceleration.x = accel[1]
        imu_msg.linear_acceleration.y = accel[0]
        imu_msg.linear_acceleration.z = accel[2]
        
        # Angular velocity (rad/s) - X/Y swapped
        imu_msg.angular_velocity.x = gyro[1]
        imu_msg.angular_velocity.y = gyro[0]
        imu_msg.angular_velocity.z = gyro[2]
        
        self.imu_pub.publish(imu_msg)

        # Publish magnetometer data (μT) - X/Y swapped
        mag_msg = MagneticField()
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = 'imu_link'
        mag_msg.magnetic_field.x = mag[1] * 1e-6  # Convert μT to Tesla
        mag_msg.magnetic_field.y = mag[0] * 1e-6
        mag_msg.magnetic_field.z = mag[2] * 1e-6
        
        self.mag_pub.publish(mag_msg)


def main():
    rclpy.init()
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
