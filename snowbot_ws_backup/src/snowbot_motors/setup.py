from setuptools import find_packages, setup

package_name = "snowbot_motors"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="pi@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "motor_node = snowbot_motors.motor_node:main",
            "odom_test = snowbot_motors.odom_test:main",
            "indoor_odom_test = snowbot_motors.indoor_odom_test:main",
            "drift_odom_test = snowbot_motors.drift_odom_test:main",
            "bench_encoder_test = snowbot_motors.bench_encoder_test:main",
            "heading_control_node = snowbot_motors.heading_control_node:main",
        ],
    },
)
