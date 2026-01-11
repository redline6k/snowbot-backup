#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
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

        # Magnetometer calibration state (hard-iron offsets in uT)
        self.mag_min = [1e9, 1e9, 1e9]
        self.mag_max = [-1e9, -1e9, -1e9]
        # Keep your latest offsets
        self.mag_offsets = [-25.8, 41.625, -18.9]

        # Set to True while doing figure-8 calibration, then set False and hard-code offsets
        self.do_calib = False

        # Timer - 100Hz update rate
        self.timer = self.create_timer(0.01, self.publish_data)

        self.get_logger().info('IMU node ready – ICM20948 publishing on /imu/data and /imu/mag')

    def publish_data(self):
        # Read sensor data (ICM20948 gives accel, gyro, mag in sensor frame)
        accel = self.icm.acceleration   # m/s^2
        gyro = self.icm.gyro            # rad/s
        mag = self.icm.magnetic         # microtesla (uT)

        # ---------------- IMU (accel + gyro) ----------------
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Linear acceleration (now use correct axes, no swap)
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        # Angular velocity (now use correct axes, no swap)
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        # Orientation left as zero (fusion node will fill this if used)

        self.imu_pub.publish(imu_msg)

        # ---------------- Magnetometer with calibration ----------------
        # Raw mag in uT (no swap now that mounting is fixed)
        mx_raw = mag[0]
        my_raw = mag[1]
        mz_raw = mag[2]

        if self.do_calib:
            raw_vals = (mx_raw, my_raw, mz_raw)
            for i, v in enumerate(raw_vals):
                if v < self.mag_min[i]:
                    self.mag_min[i] = v
                if v > self.mag_max[i]:
                    self.mag_max[i] = v

            self.mag_offsets = [
                (self.mag_min[i] + self.mag_max[i]) / 2.0 for i in range(3)
            ]

            self.get_logger().info(
                f'Mag calib min={self.mag_min}, max={self.mag_max}, off={self.mag_offsets}'
            )

        # Apply offsets (uT → corrected uT)
        mx_corr_uT = mx_raw - self.mag_offsets[0]
        my_corr_uT = my_raw - self.mag_offsets[1]
        mz_corr_uT = mz_raw - self.mag_offsets[2]

        # Convert to Tesla for MagneticField message
        uT_to_T = 1e-6
        mx_T = mx_corr_uT * uT_to_T
        my_T = my_corr_uT * uT_to_T
        mz_T = mz_corr_uT * uT_to_T

        mag_msg = MagneticField()
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = 'imu_link'
        mag_msg.magnetic_field.x = mx_T
        mag_msg.magnetic_field.y = my_T
        mag_msg.magnetic_field.z = mz_T

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
