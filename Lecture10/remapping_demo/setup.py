from setuptools import find_packages, setup
import os
from glob import glob

package_name = "remapping_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zeid kootbally",
    maintainer_email="zeidk@umd.edu",
    description="A ROS 2 package demonstrating node, topic, and parameter name remapping. This package provides a camera node that can be instantiated multiple times with different names, topics, and parameter values. It serves as an educational resource showing how to use ROS 2's remapping capabilities to create flexible and reusable nodes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["camera_demo = remapping_demo.camera_demo:main"],
    },
)
