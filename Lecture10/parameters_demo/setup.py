from setuptools import find_packages, setup
import os
from glob import glob

package_name = "parameters_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        # Include parameter files
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zeid kootbally",
    maintainer_email="zeidk@umd.edu",
    description="An educational ROS2 package demonstrating parameter usage through sensor simulation. This package illustrates how to define, declare, get, and dynamically reconfigure parameters in ROS2 nodes. Students will learn how to customize node behavior at launch time and during runtime using the parameter system, focusing on practical applications through sensor simulation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_demo = parameters_demo.camera_demo:main",
            "lidar_demo = parameters_demo.lidar_demo:main",
            "radar_demo = parameters_demo.radar_demo:main",
            "processing_demo = parameters_demo.processing_demo:main",
        ],
    },
)
