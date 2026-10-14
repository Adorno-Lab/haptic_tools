# haptic_tools

ROS 2 tools for teleoperation with a haptic device (e.g. the Touch X) of `sas_operator_side_receiver`
(SmartArmStack). The strategies are C++ libraries that do not use ROS, with ROS 2 nodes around them.

| Package | |
|---|---|
| [haptic_navigation](haptic_navigation/README.md) | The navigation mode of Aldhafeeri et al. (TAROS 2025): the twist of a command frame (an end effector, or a mobile base) and the restoring force on the device, behind a safety layer (`HapticNavigationStrategy`, `HapticNavigationSafety`, `haptic_navigation_node`). Also `HapticPoseMapping` (not from the paper; an open-source implementation of teleoperation features available in the non-commercial SmartArmStack packages): the desired pose of an end effector set with the pose of the device and its clutch, as seen from a reference camera. |
| [haptic_simulator](haptic_simulator) | A GUI that simulates the device: it publishes the same messages as the UDP receiver of the real device, and shows the force received. |
| [haptic_tools_msgs](haptic_tools_msgs) | Messages (`HapticNavigationStatus`). |

> B. Aldhafeeri, J. Carrasco, and B. V. Adorno, "A New Semi-Automatic Strategy for Teleoperating Mobile
> Manipulators Using a Haptic Device," in *Towards Autonomous Robotic Systems (TAROS 2025)*, LNAI 16045,
> pp. 11–24, Springer, 2026. [doi:10.1007/978-3-032-01486-3_2](https://doi.org/10.1007/978-3-032-01486-3_2)

The paper covers the navigation strategy only. The pose mapping (`HapticPoseMapping`) is not part of it: it is an
open-source implementation of teleoperation features available in the non-commercial SmartArmStack (SAS) packages.

## Dependencies

ROS 2 (Jazzy), Eigen, DQ Robotics, and SmartArmStack (`sas_core`, `sas_common`, `sas_msgs`; and
`sas_operator_side_receiver` for the real device). `haptic_simulator` needs Qt 6.

## Build

Clone the repository in the `src` folder of a colcon workspace:

```shell
cd ~/ros2_ws/src
git clone https://github.com/Adorno-Lab/haptic_tools.git
cd ~/ros2_ws
colcon build --packages-up-to haptic_navigation haptic_simulator
colcon test --packages-select haptic_navigation && colcon test-result --verbose
```

## Try it without a device

In separate terminals:

```shell
ros2 launch haptic_simulator haptic_simulator_launch.py motion_scaling:=10.0
ros2 launch haptic_navigation haptic_navigation_launch.py
ros2 topic pub --once /haptic_navigation/set/enabled sas_msgs/msg/Bool "{data: true}"
ros2 topic echo /haptic_navigation/get/status
```

See [haptic_navigation/README.md](haptic_navigation/README.md) for the equations, the parameters, and the topics.

## License

[LGPL-2.1](LICENSE).
