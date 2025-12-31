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

	# Front camera (existing)
	Node(
	    package='v4l2_camera',
	    executable='v4l2_camera_node',
	    name='front_camera_node',
	    namespace='front_camera',
	    parameters=[{
	        'video_device': '/dev/video0',  # Front camera device
	        'image_size': [320, 240],
	        'pixel_format': 'YUYV',
	        'camera_frame_id': 'front_camera_link',
	        'output_encoding': 'bgr8'  # Fixed encoding to avoid cv_bridge crash
	    }],
	    remappings=[
	        ('image_raw', 'front_camera/image_raw'),
	        ('camera_info', 'front_camera/camera_info')
	    ]
	),

	# Rear camera (new)
	Node(
	    package='v4l2_camera',
	    executable='v4l2_camera_node',
	    name='rear_camera_node',
	    namespace='rear_camera',
	    parameters=[{
	        'video_device': '/dev/video2',  # Rear camera device (adjust as needed)
	        'image_size': [320, 240],
	        'pixel_format': 'YUYV',
	        'camera_frame_id': 'rear_camera_link',
	        'output_encoding': 'bgr8'
	    }],
	    remappings=[
	        ('image_raw', 'rear_camera/image_raw'),
	        ('camera_info', 'rear_camera/camera_info')
	    ]
	),

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
