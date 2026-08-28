import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mdrobot_ros2_driver"

setup(
    name=package_name,
    version="1.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description="Generic ROS 2 driver node for MDROBOT MD-series motor controllers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_driver_node = mdrobot_ros2_driver.motor_driver_node:main",
            "mecanum_driver_node = mdrobot_ros2_driver.mecanum_driver_node:main",
        ],
    },
)
