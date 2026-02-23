from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # --- NEW HIGH-PRECISION GPS & NTRIP BRIDGE ---
        # Note: This replaces the nmea_navsat_driver block
        Node(
            package="snowbot_gps",  # Ensure this matches your package name
            executable="ntrip_bridge.py",
            name="ntrip_bridge_node",
            output="screen",
            parameters=[{
                "port": "/dev/serial0",
                "baud": 460800,
                "frame_id": "gps_link",
            }],
        ),

        # IMU
        Node(
            package="snowbot_imu",
            executable="imu_node",
            name="dual_imu_node",
            output="screen",
        ),
        
        # Ultrasonics
        Node(
            package="snowbot_ultrasonics",
            executable="ultrasonic_node",
            name="ultrasonic_node",
            output="screen",
        ),
        
        # Motors
        Node(
            package="snowbot_motors",
            executable="motor_node",
            name="motor_node",
            output="screen",
        ),

        # Battery & Current Monitors
        Node(
            package="snowbot_servos",
            executable="battery_monitor",
            name="battery_monitor",
            output="screen",
        ),
        Node(
            package="snowbot_current",
            executable="current_node",
            name="current_node",
            output="screen",
        ),

        # Servo/Chute Control
        Node(
            package="snowbot_servos",
            executable="servo_node",
            name="servo_node",
            output="screen",
        ),

        # Front Camera
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="front_camera_node",
            namespace="front_camera",
            parameters=[{
                "video_device": "/dev/video0",
                "image_size": [320, 240],
                "pixel_format": "YUYV",
                "camera_frame_id": "front_camera_link",
                "output_encoding": "bgr8",
            }],
            remappings=[
                ("image_raw", "front_camera/image_raw"),
                ("camera_info", "front_camera/camera_info"),
            ],
        ),

        # Rear Camera
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="rear_camera_node",
            namespace="rear_camera",
            parameters=[{
                "video_device": "/dev/video2",
                "image_size": [320, 240],
                "pixel_format": "YUYV",
                "camera_frame_id": "rear_camera_link",
                "output_encoding": "bgr8",
            }],
            remappings=[
                ("image_raw", "rear_camera/image_raw"),
                ("camera_info", "rear_camera/camera_info"),
            ],
        ),

        # Foxglove Bridge
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{"port": 8765}],
            output="screen",
        ),

        # Magnetometer & IMU Fusion
        Node(
            package="rm3100_magnetometer",
            executable="rm3100_node",
            name="rm3100_node",
            output="screen",
        ),
        
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",  # Updated to match the actual file name
            name="imu_fusion",
            remappings=[
                ("imu/data_raw", "/dual_imu_node/imu/data_raw"),
                ("imu/mag", "/rm3100_node/mag"),
            ],
            parameters=[{"use_mag": True, "publish_tf": True, "world_frame": "enu"}],
            output="screen",
        ),

        # Heading Control
        Node(
            package="snowbot_motors",
            executable="heading_control_node",
            name="heading_control",
            output="screen",
        ),
    ])