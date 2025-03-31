from setuptools import find_packages, setup

package_name = 'managed_nodes_demo'

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
    description='Demonstration of lifecycle (managed) nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_demo = managed_nodes_demo.sensor_demo:main',
            'navigation_demo = managed_nodes_demo.navigation_demo:main',
            'localization_demo = managed_nodes_demo.localization_demo:main',
        ],
    },
)
