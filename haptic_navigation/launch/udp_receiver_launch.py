"""
Starts the UDP receiver of sas_operator_side_receiver: the bridge to the computer of the haptic device (e.g. the Windows
computer of the Touch X). It publishes the data of the device on
/sas_operator_side_receiver/<label>/get/information_from_master, and sends the force back. Run it on the computer that
receives the data of the device; haptic_navigation_node can run on any computer of the same ROS network.

    ros2 launch haptic_navigation udp_receiver_launch.py

The defaults (patient_side_ips ['192.168.8.100'], patient_side_ports [2222], operator_side_ports [2223]) are those of
sas_operator_side_receiver_launch.py of semi_automatic_strategy_teleop/haptic_interface. The receiver binds to the
address in patient_side_ips: the computer must have it.
"""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'patient_side_ips',
            default_value="['192.168.8.100']",
            description='patient_side_ips of the UDP receiver (the address it binds to).'
        ),
        DeclareLaunchArgument(
            'patient_side_ports',
            default_value='[2222]',
            description='patient_side_ports of the UDP receiver.'
        ),
        DeclareLaunchArgument(
            'operator_side_ports',
            default_value='[2223]',
            description='operator_side_ports of the UDP receiver.'
        ),
        Node(
            package='sas_operator_side_receiver',
            executable='sas_operator_side_receiver_udp_node',
            name='sas_operator_side_receiver_udp_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'patient_side_ips': ParameterValue(LaunchConfiguration('patient_side_ips'), value_type=List[str]),
                'patient_side_ports': ParameterValue(LaunchConfiguration('patient_side_ports'), value_type=List[int]),
                'operator_side_ports': ParameterValue(LaunchConfiguration('operator_side_ports'), value_type=List[int]),
            }],
        ),
    ])
