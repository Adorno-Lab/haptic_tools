"""
Starts haptic_navigation_node, and optionally the UDP receiver of sas_operator_side_receiver (udp_receiver_launch.py:
the bridge to the computer of the haptic device).

    ros2 launch haptic_navigation haptic_navigation_launch.py config_file:=<robot>.yaml
    ros2 launch haptic_navigation haptic_navigation_launch.py start_udp_receiver:=true

The node and the receiver can also run on different computers of the same ROS network (e.g. the receiver on the
computer that receives the data of the device, with udp_receiver_launch.py). Without a device, use haptic_simulator
(haptic_tools), which publishes the same messages.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([FindPackageShare('haptic_navigation'), 'config', 'default.yaml']),
            description='Parameters of haptic_navigation_node (see config/default.yaml).'
        ),
        DeclareLaunchArgument(
            'start_udp_receiver',
            default_value='false',
            choices=['true', 'false'],
            description='true: also starts sas_operator_side_receiver_udp_node (udp_receiver_launch.py).'
        ),
        Node(
            package='haptic_navigation',
            executable='haptic_navigation_node',
            name='haptic_navigation_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('config_file')],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('haptic_navigation'), 'launch', 'udp_receiver_launch.py'])),
            condition=IfCondition(LaunchConfiguration('start_udp_receiver'))
        ),
    ])
