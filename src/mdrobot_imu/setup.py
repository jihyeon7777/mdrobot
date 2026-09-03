import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mdrobot_imu"

setup(
    name=package_name,
    version="1.3.0",
    packages=find_packages(include=["mdrobot_imu", "mdrobot_imu.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    # pyserial comes from apt (python3-serial) on the target and is declared in
    # package.xml, where rosdep reads it.
    install_requires=["setuptools"],
    extras_require={"dev": ["pytest>=7"]},
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description=(
        "WITMOTION HWT901B attitude sensor as sensor_msgs/Imu, for a robot that "
        "works out of sight under a vehicle."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_node = mdrobot_imu.imu_node:main",
            "imu_survey = mdrobot_imu.survey:main",
        ],
    },
)
