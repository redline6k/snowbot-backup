from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU
        Node(
            package='snowbot_imu',
            executable='imu_node',
            name='dual_imu_node',
            output='screen'
        ),

        # Ultrasonics
        Node(
            package='snowbot_ultrasonics',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen'
        ),

        # Ultrasonic Visualizer
        Node(
            package='snowbot_ultrasonics',
            executable='ultrasonic_visualizer',
            name='ultrasonic_visualizer',
            output='screen'
        ),

        # Motors
        Node(
            package='snowbot_motors',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),

        # Battery Monitor
        Node(
            package='snowbot_servos',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen'
        ),

        # Current Monitor
        Node(
            package='snowbot_current',
            executable='current_node',
            name='current_node',
            output='screen'
        ),

        # Servo/Chute Control
        Node(
            package='snowbot_servos',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),

        # GPS
        Node(
            package="nmea_navsat_driver",
            executable="nmea_serial_driver",
            name="gps_node",
            parameters=[{
                "port": "/dev/ttyAMA0",
                "baud": 9600,
                "frame_id": "gps_link",
                "time_ref_source": "gps",
                "use_RMC": False,
            }],
            output="screen"
        ),

# Camera
Node(
    package='v4l2_camera',
    executable='v4l2_camera_node',
    name='v4l2_camera_node',
    parameters=[{
        'video_device': '/dev/video0',
        'image_size': [320, 240],
        'camera_frame_id': 'camera_link',
    }],
    output='screen'
),

	# Rear Camera
#	Node(
#	    package='usb_cam',
#	    executable='usb_cam_node_exe',
#	    name='rear_camera',
#	    namespace='camera_rear',
#	    parameters=[{
#	        'video_device': '/dev/video2',
#	        'image_width': 160,
#	        'image_height': 120,
#	        'framerate': 5.0,
#	        'pixel_format': 'mjpeg2rgb',  # Handles MJPEG properly
#	        'camera_name': 'rear_camera',
#	    }],
#	    output='screen'
#	),

        # Camera compression throttle for Foxglove
 #       Node(
  #          package='topic_tools',
   #         executable='throttle',
#            name='camera_front_throttle',
#           arguments=['messages', '/camera_front/image_raw/compressed', '5.0', '/camera_front/image_viz/compressed'],
#            output='screen'
#        ),
#        Node(
#            package='topic_tools',
#            executable='throttle',
#            name='camera_rear_throttle',
#            arguments=['messages', '/camera_rear/image_raw/compressed', '5.0', '/camera_rear/image_viz/compressed'],
#            output='screen'
#        ),


        # Foxglove Bridge for web interface
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
            }],
            output='screen'
        ),
    ])
