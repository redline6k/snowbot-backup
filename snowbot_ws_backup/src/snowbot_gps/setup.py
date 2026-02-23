from setuptools import setup
import os
from glob import glob

package_name = 'snowbot_gps'

setup(
    name=package_name,
    version='0.0.0',
    # We add 'scripts' here if you want to import things from it, 
    # but for a standalone node, keeping it as is works.
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Ensure this refers to the file relative to setup.py
        (os.path.join('lib', package_name), ['snowbot_gps/ntrip_bridge.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@snowbot.local',
    description='GPS and RTK for Snowbot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # --- ADD THIS LINE ---
            # Syntax: 'executable_name = folder.filename:function'
            'ntrip_bridge = snowbot_gps.ntrip_bridge:main'
        ],
    },
)