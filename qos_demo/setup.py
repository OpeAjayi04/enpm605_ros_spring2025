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
    maintainer='zeid',
    maintainer_email='adon.sf4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fast_publisher_demo = qos_demo.qos_demo1:main_fast_publisher',
            'fast_subscriber_demo = qos_demo.qos_demo1:main_fast_subscriber',
        ],
    },
)
