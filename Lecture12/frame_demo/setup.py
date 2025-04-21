from setuptools import find_packages, setup

package_name = 'frame_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeid',
    maintainer_email='adon.sf4@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "color_beacon_frame_demo=frame_demo.color_beacon_frame_demo:main",
        "aruco_opencv_detector_demo=frame_demo.aruco_opencv_detector_demo:main",
        "kdl_frame_demo=frame_demo.kdl_frame_demo:main",
        ],
    },
)
