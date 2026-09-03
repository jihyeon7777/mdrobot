import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mdrobot_mecanum"

setup(
    name=package_name,
    version="1.3.0",
    packages=find_packages(include=["mdrobot_mecanum", "mdrobot_mecanum.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    # mdrobot is deliberately NOT listed: it is not on PyPI, so pip would try to
    # fetch it and fail. It is declared in package.xml, where colcon/rosdep read it,
    # and installed alongside this package from the same workspace.
    install_requires=["setuptools", "PyYAML>=6"],
    extras_require={
        "serial": ["pyserial>=3.5"],
        "dev": ["pytest>=7"],
    },
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description=(
        "Mecanum drive layer for a 4-wheel base on two dual-channel MDROBOT "
        "controllers: kinematics, wheel-identification wizard, keyboard teleop."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "identify = mdrobot_mecanum.identify:main",
            "teleop_keyboard = mdrobot_mecanum.teleop_keyboard:main",
        ],
    },
)
