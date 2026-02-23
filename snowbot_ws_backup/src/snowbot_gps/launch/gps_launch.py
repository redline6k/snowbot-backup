from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 1. The GPS Driver (The Listener)
            Node(
                package="nmea_navsat_driver",
                executable="nmea_serial_driver",
                name="gps_node",
                parameters=[
                    {
                        "port": "/dev/ttyAMA0",
                        "baud": 460800,  # CRITICAL: LG290P default
                        "frame_id": "gps_link",
                    }
                ],
                output="screen",
            ),
            # 2. The NTRIP Client (The Correction Feed)
            Node(
                package="ntrip_client",
                executable="ntrip_client_node",
                name="ntrip_client_node",
                parameters=[
                    {
                        "host": "rtk.massdot.state.ma.us",  # MaCORS Host
                        "port": 2101,
                        "mountpoint": "MA_MSM5",  # High-precision mountpoint
                        "username": "YOUR_USERNAME",  # From your MaCORS registration
                        "password": "YOUR_PASSWORD",
                        "ntrip_version": "Ntrip2",
                    }
                ],
                output="screen",
            ),
        ]
    )
