"""
Starts the simulator of a haptic device of sas_operator_side_receiver (e.g. the Touch X). It publishes the same
messages as the UDP receiver with the real device, on /sas_operator_side_receiver/<label>/get/information_from_master,
and shows the force received on /sas_operator_side_receiver/<label>/set/information_to_master.

    ros2 launch haptic_simulator haptic_simulator_launch.py
    ros2 launch haptic_simulator haptic_simulator_launch.py motion_scaling:=10.0

Do not run it with the UDP receiver of the real device, on the same label and ROS_DOMAIN_ID.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'label',
            default_value='m0_0',
            description='Label of the device in sas_operator_side_receiver.'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='250.0',
            description='Rate of the messages of the device (Hz).'
        ),
        DeclareLaunchArgument(
            'translation_range',
            default_value='0.25',
            description='The stylus moves in [-translation_range, translation_range] on each axis (m).'
        ),
        DeclareLaunchArgument(
            'motion_scaling',
            default_value='1.0',
            description='Initial motion scaling (it can be changed in the GUI).'
        ),
        DeclareLaunchArgument(
            'force_compliance',
            default_value='0.05',
            description='With "Apply the force to the stylus", the stylus moves force_compliance*force per second (m/(N s)).'
        ),
        Node(
            package='haptic_simulator',
            executable='haptic_simulator',
            name='haptic_simulator',
            output='screen',
            parameters=[{
                'label': LaunchConfiguration('label'),
                'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=float),
                'translation_range': ParameterValue(LaunchConfiguration('translation_range'), value_type=float),
                'motion_scaling': ParameterValue(LaunchConfiguration('motion_scaling'), value_type=float),
                'force_compliance': ParameterValue(LaunchConfiguration('force_compliance'), value_type=float),
            }],
        ),
    ])
