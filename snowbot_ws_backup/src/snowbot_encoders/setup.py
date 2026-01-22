from setuptools import find_packages, setup

package_name = 'snowbot_encoders'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='redline6k@gmail.com',
    description='AS5600 encoder reader for Snowbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_node = snowbot_encoders.encoder_node:main',
        ],
    },
)
