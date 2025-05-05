from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_rosbot_gazebo_arg = DeclareLaunchArgument(
        "enable_rosbot_gazebo",
        default_value="true",
        description="Enable the ROSbot Gazebo simulation",
    )
    
    # Set path for the map file
    map_yaml_path = PathJoinSubstitution(
        [FindPackageShare("mapping_navigation_demo"), "maps", "husarion_world.yaml"]
    )
    
    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=map_yaml_path,
        description="Full path to map yaml file to load"
    )

    # Log the map path for debugging
    log_map_path = LogInfo(
        msg=["Map file path: ", LaunchConfiguration("map")]
    )

    # Parameter files
    nav2_params_path = PathJoinSubstitution(
        [FindPackageShare("mapping_navigation_demo"), "config", "nav2_params.yaml"]
    )

    # 1. ROSbot Gazebo simulation launch
    rosbot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbot_gazebo'),
                'launch',
                'simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'start_rviz': 'false',
            'x': '0.0',
            'y': '2.0',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_rosbot_gazebo")),
    )

    # 2. Add a static transform publisher to ensure map frame exists before localization starts
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # 3. Nav2 Localization - use the standard Nav2 localization launch instead of individual nodes
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py'
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items(),
    )
    
    # 3. Nav2 Navigation Stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': nav2_params_path,
        }.items(),
    )
    
    # 4. RViz with navigation configuration
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Add the launch arguments first
    ld.add_action(use_sim_time_arg)
    ld.add_action(enable_rosbot_gazebo_arg)
    ld.add_action(map_file_arg)
    ld.add_action(log_map_path)
    
    # Add actions in the correct order
    ld.add_action(rosbot_gazebo_launch)
    ld.add_action(static_tf_node)  # Add the static transform first to ensure map frame exists
    ld.add_action(localization_launch)  # Using standard localization launch file
    ld.add_action(navigation_launch)
    ld.add_action(rviz_node)
    
    return ld