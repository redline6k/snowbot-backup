from setuptools import setup
import os
from glob import glob

package_name = 'snowbot_autonomy'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rafael Paz',
    maintainer_email='pi@snowbot.local',
    description='Autonomous navigation for snowbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = snowbot_autonomy.obstacle_avoidance:main',
            'teleop_keyboard = snowbot_autonomy.teleop_keyboard:main',
        ],
    },
)
