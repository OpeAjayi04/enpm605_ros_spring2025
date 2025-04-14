from setuptools import find_packages, setup

package_name = 'rosbot_interaction_demo'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cube_publisher_demo = rosbot_interaction_demo.cube_publisher_demo:main",
            "pid_controller_demo = rosbot_interaction_demo.pid_controller_demo:main",
        ],
    },
)
