from setuptools import find_packages, setup

package_name = "mdrobot"

setup(
    name=package_name,
    version="1.4.0",
    packages=find_packages(include=["mdrobot", "mdrobot.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={
        "serial": ["pyserial>=3.5"],
        "dev": ["pytest>=7"],
    },
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description=(
        "RS485 / Modbus RTU communication library for MDROBOT MD-series motor "
        "controllers (single- and dual-channel)."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
)
