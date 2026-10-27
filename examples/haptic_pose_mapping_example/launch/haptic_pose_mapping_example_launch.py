"""
Example of the pose mapping with scenes/pose_mapping.ttt: start the scene in CoppeliaSim (and its simulation),
then

    ros2 launch haptic_pose_mapping_example haptic_pose_mapping_example_launch.py                 # with haptic_simulator
    ros2 launch haptic_pose_mapping_example haptic_pose_mapping_example_launch.py device:=udp     # with the Touch X

device:=simulator starts haptic_simulator (press the clutch with the space bar). device:=udp starts the UDP receiver
of the Touch X (udp_receiver_launch.py of haptic_navigation; see its arguments).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='simulator',
            choices=['simulator', 'udp'],
            description='simulator: haptic_simulator. udp: the UDP receiver of the Touch X.'
        ),
        DeclareLaunchArgument(
            'motion_scaling',
            default_value='1.0',
            description='Motion scaling of haptic_simulator (device:=simulator; the example uses motion_scaling of its '
                        'YAML instead).'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('haptic_simulator'), 'launch', 'haptic_simulator_launch.py'])),
            launch_arguments={'motion_scaling': LaunchConfiguration('motion_scaling')}.items(),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'simulator')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('haptic_navigation'), 'launch', 'udp_receiver_launch.py'])),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'udp')),
        ),
        Node(
            package='haptic_pose_mapping_example',
            executable='haptic_pose_mapping_example_node',
            name='haptic_pose_mapping_example_node',
            output='screen',
            emulate_tty=True,
            parameters=[PathJoinSubstitution([FindPackageShare('haptic_pose_mapping_example'), 'config',
                                              'haptic_pose_mapping_example.yaml'])],
        ),
    ])
