from setuptools import setup

package_name = 'rosgpt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Anis Koubaa',
    author_email='anis.koubaa@gmail.com',
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    keywords=['ROS', 'ChatGPT'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Creative Commons Attribution-NonCommercial License (CC BY-NC)',
        'Programming Language :: Python :: 3.10', #could work with other version. Tested with 3.10
    ],
    description='A ROS2 package for processing and executing unstructured textual commands using ChatGPT in human-robot interaction scenarios.',
    license='Creative Commons Attribution-NonCommercial (CC BY-NC)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosgpt = rosgpt.rosgpt:main',
            'rosgptparser_turtlesim = rosgpt.rosgptparser_turtlesim:main',
            'rosgpt_client_node = rosgpt.rosgpt_client_node:main',
            'anthropic_server = rosgpt.ros_anthropic_server:main',
            'anthropic_client = rosgpt.ros_anthropic_client:main',
            'anthropic_nav2_client = rosgpt.ros_anthropic_nav2_client:main',
            'anthropic_nav2_server = rosgpt.ros_anthropic_nav2_server:main',
            'anthropic_nav2_control = rosgpt.ros_anthropic_nav2_control:main',
            'anthropic_nav2_voice_control = rosgpt.ros_anthropic_nav2_voice_control:main',
        ],
    },
)
