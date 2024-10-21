import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='127.0.0.1',  # 默认值可以根据需要修改
        description='IP address of the robot'
    )

    # Define the node with the parameter
    jaka_driver_node = Node(
        package='jaka_driver',
        executable='jaka_driver',
        name='jaka_driver',
        output='screen',
        parameters=[{'ip': LaunchConfiguration('ip')}]
    )

    return LaunchDescription([
        ip_arg,
        jaka_driver_node
    ])
