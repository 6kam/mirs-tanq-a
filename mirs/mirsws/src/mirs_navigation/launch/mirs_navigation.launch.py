import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # Declare arguments #
    lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='Set lidar usb port.')

    # Launch files and Nodes #
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_s1_launch.py')
        ),
        launch_arguments={'serial_port': LaunchConfiguration('lidar_port')}.items()
    )

    tf2_ros_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        #arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"]
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser'
        ]
    )

    ld = LaunchDescription()
    ld.add_action(lidar_port)

    ld.add_action(sllidar_launch)
    ld.add_action(tf2_ros_node)

    return ld