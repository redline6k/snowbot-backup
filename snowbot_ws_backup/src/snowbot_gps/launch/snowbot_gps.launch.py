from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="nmea_navsat_driver",
                executable="nmea_serial_driver",
                name="nmea_serial_driver",
                parameters=[
                    {
                        "port": "/dev/serial0",
                        "baud": 460800,
                        "frame_id": "gps",
                        "err_on_invalid_sentence": False,
                    }
                ],
                # Ensure the driver publishes to the global /nmea_sentence topic
                remappings=[("/nmea_sentence", "/nmea_sentence")],
                output="screen",
            ),
            Node(
                package="ntrip_client",
                executable="ntrip_ros.py",
                name="ntrip_client",
                parameters=[
                    {
                        "host": "macorsrtk.massdot.state.ma.us",
                        "port": 10000,
                        "mountpoint": "RTCM3_NEAR",
                        "username": "redline6k",
                        "password": "mstrtech877",  # Use r"password" if you have special chars
                        "ntrip_version": "Ntrip1",
                        # This forces the node to wait for a position before connecting
                        "nmea_max_count": 1,
                    }
                ],
                remappings=[("nmea_sentence", "/nmea_sentence")],
                output="screen",
            ),
        ]
    )
