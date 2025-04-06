from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'launch_files_demo'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeid kootbally',
    maintainer_email='zeidk@umd.edu',
    description='Demonstration of launch file features',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'camera_demo = launch_files_demo.camera_demo:main'
        ],
    },
)
