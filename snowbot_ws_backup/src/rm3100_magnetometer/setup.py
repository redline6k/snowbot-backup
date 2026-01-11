from setuptools import setup

package_name = 'rm3100_magnetometer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@snowbot.local',
    description='RM3100 magnetometer driver for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rm3100_node = rm3100_magnetometer.rm3100_node:main',
        ],
    },
)
