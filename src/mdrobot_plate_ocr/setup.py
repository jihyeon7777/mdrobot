import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mdrobot_plate_ocr"

setup(
    name=package_name,
    version="1.3.0",
    packages=find_packages(include=["mdrobot_plate_ocr", "mdrobot_plate_ocr.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    # opencv, numpy and Pillow are deliberately NOT listed: on the target they
    # come from apt (python3-opencv etc.) and are declared in package.xml, where
    # colcon/rosdep read them. Listing them here invites pip to fetch a ~300 MB
    # wheel during colcon build. Tesseract is not a Python package at all.
    install_requires=["setuptools"],
    extras_require={"dev": ["pytest>=7"]},
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description=(
        "Korean licence plate OCR from a USB camera, published as a ROS 2 topic; "
        "debug JPEGs instead of an image stream, for SSH-only robots."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plate_ocr_node = mdrobot_plate_ocr.plate_ocr_node:main",
            "plate_probe = mdrobot_plate_ocr.probe:main",
        ],
    },
)
