import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    # Declare arguments #
    slam_config_file = LaunchConfiguration('slam_config_file')
    rviz2_file = LaunchConfiguration('rviz2_file')

    declare_arg_slam_config_file = DeclareLaunchArgument(
        'slam_config_file',
        default_value=os.path.join(
            get_package_share_directory('mirs_slam'),
            'config',
            'slam_toolbox_config.yaml'),
        description='The full path to the config file for SLAM')

    declare_arg_rviz2_config_path = DeclareLaunchArgument(
        'rviz2_file', default_value=os.path.join(
            get_package_share_directory('mirs_slam'),
            'rviz',
            'default.rviz'),
        description='The full path to the rviz file'
    )

    # Launch the online_async_node
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        )
    )

    # Nodes #
    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        output='screen',
        parameters=[slam_config_file],
    )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz2_file],
    )

    ld = LaunchDescription()
    ld.add_action(declare_arg_slam_config_file)
    ld.add_action(declare_arg_rviz2_config_path)

    ld.add_action(slam_node)
    ld.add_action(rviz2_node)

    return ld