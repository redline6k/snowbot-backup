#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import board
import busio
from adafruit_icm20x import ICM20948, AccelRange, GyroRange
from adafruit_mpu6050 import MPU6050


class DualImuNode(Node):
    def __init__(self):
        super().__init__('dual_imu_node')

        i2c = busio.I2C(board.SCL, board.SDA)

        # ICM20948 @ 0x68
        self.icm = ICM20948(i2c, address=0x68)
        self.icm.accelerometer_range = AccelRange.RANGE_4G
        self.icm.gyro_range = GyroRange.RANGE_2000_DPS

        # MPU9250/MPU6500 — try 0x69 first, fall back to 0x68
        self.mpu = None
        try:
            self.mpu = MPU6050(i2c, address=0x69)
            self.get_logger().info('MPU9250 found @ 0x69')
        except:
            try:
                self.mpu = MPU6050(i2c, address=0x68)
                self.get_logger().info('MPU9250 found @ 0x68')
            except Exception as e:
                self.get_logger().warn(f'MPU9250 not detected: {e}')

        # Publishers
        self.icm_pub = self.create_publisher(Imu, '/imu/icm20948', 10)
        self.mpu_pub = self.create_publisher(Imu, '/imu/mpu9250', 10)
        self.mag_pub = self.create_publisher(Vector3, '/magnetometer', 10)

        self.timer = self.create_timer(0.01, self.publish_data)

        self.get_logger().info('Dual IMU node READY – ICM20948 (0x68) + MPU9250 (auto-detected) – X/Y swapped')

    def publish_data(self):
        # ICM20948 — X/Y swapped
        accel = self.icm.acceleration
        gyro = self.icm.gyro
        mag = self.icm.magnetic

        icm_msg = Imu()
        icm_msg.header.stamp = self.get_clock().now().to_msg()
        icm_msg.header.frame_id = 'imu_icm20948'
        icm_msg.linear_acceleration.x = accel[1]
        icm_msg.linear_acceleration.y = accel[0]
        icm_msg.linear_acceleration.z = accel[2]
        icm_msg.angular_velocity.x = gyro[1]
        icm_msg.angular_velocity.y = gyro[0]
        icm_msg.angular_velocity.z = gyro[2]
        self.icm_pub.publish(icm_msg)

        mag_msg = Vector3()
        mag_msg.x = mag[1]
        mag_msg.y = mag[0]
        mag_msg.z = mag[2]
        self.mag_pub.publish(mag_msg)

        # MPU9250 — only if found
        if self.mpu:
            accel_mpu = self.mpu.acceleration
            gyro_mpu = self.mpu.gyro

            mpu_msg = Imu()
            mpu_msg.header.stamp = self.get_clock().now().to_msg()
            mpu_msg.header.frame_id = 'imu_mpu9250'
            mpu_msg.linear_acceleration.x = accel_mpu[1]
            mpu_msg.linear_acceleration.y = accel_mpu[0]
            mpu_msg.linear_acceleration.z = accel_mpu[2]
            mpu_msg.angular_velocity.x = gyro_mpu[1]
            mpu_msg.angular_velocity.y = gyro_mpu[0]
            mpu_msg.angular_velocity.z = gyro_mpu[2]
            self.mpu_pub.publish(mpu_msg)


def main():
    rclpy.init()
    node = DualImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
