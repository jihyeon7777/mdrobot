import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mdrobot_gpio"

setup(
    name=package_name,
    version="1.3.0",
    packages=find_packages(include=["mdrobot_gpio", "mdrobot_gpio.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    # lgpio comes from apt (python3-lgpio); listing it here would have pip
    # fetch a build of it during colcon build.
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description="Indicator LEDs on the Pi's header pins, driven from ROS topics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "led_node = mdrobot_gpio.led_node:main",
            "led_test = mdrobot_gpio.led_test:main",
        ],
    },
)
