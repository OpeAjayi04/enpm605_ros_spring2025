# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     ExecuteProcess,
#     IncludeLaunchDescription,
# )
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():
#     # Create a LaunchDescription object
#     ld = LaunchDescription()

#     map_file_path = PathJoinSubstitution(
#         [FindPackageShare("mapping_navigation_demo"), "maps", "husarion_world.yaml"]
#     )

#     nav2_file_path = PathJoinSubstitution(
#         [FindPackageShare("mapping_navigation_demo"), "config", "nav2_params.yaml"]
#     )
    
#     default_slam_params_path = PathJoinSubstitution(
#         [
#             FindPackageShare("mapping_navigation_demo"),
#             "config",
#             "mapper_params_online_async.yaml",
#         ]
#     )

#     # Declare a launch argument to enable or disable the ROSbot Gazebo simulation
#     enable_rosbot_gazebo_arg = DeclareLaunchArgument(
#         "enable_rosbot_gazebo",
#         default_value="true",
#         description="Enable the ROSbot Gazebo simulation",
#     )

#     # ROSbot Gazebo simulation launch
#     rosbot_gazebo_launch = ExecuteProcess(
#         cmd=[
#             "ros2",
#             "launch",
#             "rosbot_gazebo",
#             "simulation.launch.py",
#             "start_rviz:=true",
#             "y:=2.0",
#         ],
#         output="screen",
#         condition=IfCondition(LaunchConfiguration("enable_rosbot_gazebo")),
#     )

#     # SLAM Toolbox launch - improved version
#     slam_toolbox_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [
#                 PathJoinSubstitution(
#                     [
#                         FindPackageShare("slam_toolbox"),
#                         "launch",
#                         "online_async_launch.py",
#                     ]
#                 )
#             ]
#         ),
#         launch_arguments={
#             "use_sim_time": "true",
#             "slam_params_file": default_slam_params_path,
#         }.items()
#     )
    
#     # Nav2 launch with map argument
#     nav2_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [
#                 PathJoinSubstitution(
#                     [
#                         FindPackageShare("nav2_bringup"),
#                         "launch",
#                         "navigation_launch.py",
#                     ]
#                 )
#             ]
#         ),
#         launch_arguments={
#             "use_sim_time": "true",
#             "autostart": "true",  # Add autostart if you want Nav2 to start immediately
#             "map": map_file_path,
#             "params_file": nav2_file_path,
#         }.items(),
#     )
    
#     navigation_demo_node = Node(
#         package='mapping_navigation_demo',
#         executable='navigation_node',
#         name='navigation_node',
#         parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#         output='screen'
#     )

#     # Add the launch arguments and groups to the LaunchDescription
#     ld.add_action(enable_rosbot_gazebo_arg)
#     ld.add_action(rosbot_gazebo_launch)
#     ld.add_action(nav2_launch)
#     ld.add_action(slam_toolbox_launch)
#     ld.add_action(navigation_demo_node)

#     # Return the LaunchDescription object
#     return ld


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
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
    
    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mapping_navigation_demo"), "maps", "husarion_world.yaml"]
        ),
        description="Full path to map yaml file to load"
    )

    # Parameter files
    nav2_file_path = PathJoinSubstitution(
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
            'start_rviz': 'false',  # Let's manage RViz separately for better control
            'y': '2.0',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_rosbot_gazebo")),
    )

    # 2. Map Server and AMCL for localization (using nav2_bringup localization.launch.py)
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
            'autostart': 'true',
            'params_file': nav2_file_path,
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
            'params_file': nav2_file_path,
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
    
    # 5. Your navigation demo node
    navigation_demo_node = Node(
        package='mapping_navigation_demo',
        executable='navigation_node',
        name='navigation_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Add the launch arguments first
    ld.add_action(use_sim_time_arg)
    ld.add_action(enable_rosbot_gazebo_arg)
    ld.add_action(map_file_arg)
    
    # Add actions in correct order
    ld.add_action(rosbot_gazebo_launch)     # Start simulation first
    ld.add_action(localization_launch)      # Start localization second (map server + AMCL)
    ld.add_action(navigation_launch)        # Start navigation stack third
    ld.add_action(rviz_node)                # Start RViz fourth
    # Uncomment the line below when you're ready to start your navigation node
    # ld.add_action(navigation_demo_node)   # Start your navigation node last
    
    # Return the LaunchDescription object
    return ld