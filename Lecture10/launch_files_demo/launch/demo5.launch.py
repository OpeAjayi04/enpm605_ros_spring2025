from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():
    ld = LaunchDescription()

    # Nodes needed for navigation
    navigation_group = GroupAction([
        Node(package='launch_files_demo', executable='lidar_demo'),
        Node(package='launch_files_demo', executable='camera_demo')
    ])
    
    temperature_node = Node(package='launch_files_demo', executable='temperature_demo')

    ld.add_action(navigation_group)
    # ld.add_action(temperature_node)
    return ld