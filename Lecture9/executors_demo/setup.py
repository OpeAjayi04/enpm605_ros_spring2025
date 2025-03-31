from setuptools import find_packages, setup

package_name = 'executors_demo'

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
    maintainer_email='zeidk@umd.edu',
    description='Demo showing the use of single-threaded and multi-threaded executors',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_mutex_demo = executors_demo.dual_mutex_demo:main',
            'reentrant_mutex_demo = executors_demo.reentrant_mutex_demo:main',
            'reentrant_only_demo = executors_demo.reentrant_only_demo:main',
            'singlethreaded_demo = executors_demo.singlethreaded_demo:main',
        ],
    },
)
