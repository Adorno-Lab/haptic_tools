"""
Example of the haptic navigation with scenes/haptic_scene.ttt: start the scene in CoppeliaSim (and its simulation),
then

    ros2 launch haptic_navigation_example haptic_navigation_example_launch.py                 # with haptic_simulator
    ros2 launch haptic_navigation_example haptic_navigation_example_launch.py device:=udp     # with the Touch X

device:=simulator starts haptic_simulator (press the clutch with the space bar). device:=udp starts the UDP receiver
of the Touch X (udp_receiver_launch.py of haptic_navigation; see its arguments). gui:=true (the default) starts
haptic_navigation_example_gui, which shows the dead zone, the stylus, its angle, and the current mode. device:=none
starts neither (the device is started apart, e.g. in another container).
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
            choices=['simulator', 'udp', 'none'],
            description='simulator: haptic_simulator. udp: the UDP receiver of the Touch X. none: started apart '
                        '(e.g. by the compose file of devel/examples/haptic_navigation_example).'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start haptic_navigation_example_gui.'
        ),
        DeclareLaunchArgument(
            'motion_scaling',
            default_value='10.0',
            description='Motion scaling of haptic_simulator (device:=simulator).'
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
            package='haptic_navigation_example',
            executable='haptic_navigation_example_node',
            name='haptic_navigation_example_node',
            output='screen',
            emulate_tty=True,
            parameters=[PathJoinSubstitution([FindPackageShare('haptic_navigation_example'), 'config',
                                              'haptic_navigation_example.yaml'])],
        ),
        Node(
            package='haptic_navigation_example',
            executable='haptic_navigation_example_gui',
            name='haptic_navigation_example_gui',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
