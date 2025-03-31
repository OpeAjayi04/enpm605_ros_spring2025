from setuptools import find_packages, setup

package_name = 'qos_demo'

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
    maintainer='zeid kootbally',
    maintainer_email='zeidk@umd.edu',
    description='Demonstration of Quality of Service (QoS)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # demo1
            'fast_publisher_demo = qos_demo.qos_demo1:main_fast_publisher',
            'fast_subscriber_demo = qos_demo.qos_demo1:main_fast_subscriber',
            # demo2
            'overflow_publisher_demo = qos_demo.qos_demo2:main_overflow_publisher',
            'slow_subscriber_demo = qos_demo.qos_demo2:main_slow_subscriber',
            # demo3
            'best_effort_publisher_demo = qos_demo.qos_demo3:main_best_effort_publisher',
            'reliable_subscriber_demo = qos_demo.qos_demo3:main_reliable_subscriber',
            # demo4
            'latched_publisher_demo = qos_demo.qos_demo4:main_latched_publisher',
            'late_subscriber_demo = qos_demo.qos_demo4:main_late_subscriber',
        ],
    },
)
