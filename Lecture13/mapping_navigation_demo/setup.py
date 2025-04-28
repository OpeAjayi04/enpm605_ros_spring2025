from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mapping_navigation_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Include map files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeidk',
    maintainer_email='zeidk@umd.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "navigation_node = mapping_navigation_demo.navigation_demo_interface:main",
        ],
    },
)
