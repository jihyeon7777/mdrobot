import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mdrobot_description"

setup(
    name=package_name,
    version="1.3.0",
    packages=find_packages(include=["mdrobot_description", "mdrobot_description.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        # Meshes are installed wholesale so a package:// path resolves. Drop
        # STL/DAE files into meshes/ and rebuild.
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description="URDF, meshes and an RViz view of the machine, driven by live hardware.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mesh_check = mdrobot_description.mesh_check:main",
            "mesh_simplify = mdrobot_description.mesh_simplify:main",
        ],
    },
)
