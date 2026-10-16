# haptic_tools

[![Docs](https://img.shields.io/badge/docs-GitHub_Pages-green)](https://adorno-lab.github.io/haptic_tools/)

ROS 2 tools for teleoperation with a haptic device (e.g. the Touch X) of `sas_operator_side_receiver`
(SmartArmStack). The strategies are C++ libraries that do not use ROS, with ROS 2 nodes around them.

| Package | |
|---|---|
| [haptic_tools_core](haptic_tools_core/README.md) | The C++ libraries, without ROS. `navigation/`: the navigation mode of Aldhafeeri et al. (TAROS 2025), the twist of a command frame (an end effector, or a mobile base) and the restoring force on the device, behind a safety layer (`HapticNavigationStrategy`, `HapticNavigationSafety`). `pose_mapping/`: `HapticPoseMapping` (not from the paper; an open-source implementation of teleoperation features available in the non-commercial SmartArmStack packages), the desired pose of an end effector set with the pose of the device and its clutch, as seen from a reference camera. |
| [haptic_navigation](haptic_navigation/README.md) | `haptic_navigation_node`: the ROS 2 node of the navigation. |
| [haptic_simulator](haptic_simulator) | A GUI that simulates the device: it publishes the same messages as the UDP receiver of the real device, and shows the force received. |
| [haptic_tools_msgs](haptic_tools_msgs) | Messages (`HapticNavigationStatus`). |
| [examples/](examples/README.md) | `haptic_navigation_example`, with the CoppeliaSim scene [scenes/haptic_scene.ttt](scenes/haptic_scene.ttt). |

> B. Aldhafeeri, J. Carrasco, and B. V. Adorno, "A New Semi-Automatic Strategy for Teleoperating Mobile
> Manipulators Using a Haptic Device," in *Towards Autonomous Robotic Systems (TAROS 2025)*, LNAI 16045,
> pp. 11–24, Springer, 2026. [doi:10.1007/978-3-032-01486-3_2](https://doi.org/10.1007/978-3-032-01486-3_2)

The paper covers the navigation strategy only. The pose mapping (`HapticPoseMapping`) is not part of it: it is an
open-source implementation of teleoperation features available in the non-commercial SmartArmStack (SAS) packages.

## Requirements

**To build and run the packages** (tested on Ubuntu 24.04):

- ROS 2 Jazzy.
- [DQ Robotics](https://dqrobotics.github.io/) for C++ (`libdqrobotics`) and Eigen 3.
- The non-commercial packages of [SmartArmStack (SAS)](https://smartarmstack.github.io/): `sas_core`, `sas_common`,
  `sas_msgs`, and `sas_operator_side_receiver` (the UDP receiver of the real device, declared by `haptic_navigation`;
  not needed with `haptic_simulator`).
- Qt 6 (`qt6-base-dev`), for `haptic_simulator` and the GUI of `haptic_navigation_example`.

**To run the example in simulation:**

- CoppeliaSim 4.7 (the scene [scenes/haptic_scene.ttt](scenes/haptic_scene.ttt) was saved with 4.7.0 rev4), with
  the Python interface to ROS 2 (`rclpy`, used by the script of the base of the scene) and `sas_robot_driver_coppeliasim`
  of SAS (used by the `sas_object_script` of the frames of the scene).
- A haptic device: `haptic_simulator` (no hardware needed), or the Touch X (below).

**To use the real haptic device (the Touch X):**

- A Windows computer with the Touch X connected, its drivers, and the SAS program of the operator side that sends
  the data of the device over UDP.
- `sas_operator_side_receiver` of SAS on the Linux computer: it receives the data of the device and publishes it on
  ROS 2 (`udp_receiver_launch.py` of `haptic_navigation`, or `device:=udp` in the examples). Both computers must be
  on the same network; the receiver binds to `patient_side_ips` (default `192.168.8.100`), with the ports
  `patient_side_ports` (`2222`) and `operator_side_ports` (`2223`).

## Build

Clone the repository in the `src` folder of a colcon workspace:

```shell
cd ~/ros2_ws/src
git clone https://github.com/Adorno-Lab/haptic_tools.git
cd ~/ros2_ws
colcon build --packages-up-to haptic_navigation haptic_simulator
colcon test --packages-select haptic_tools_core && colcon test-result --verbose
```

## Try it without a device

In separate terminals:

```shell
ros2 launch haptic_simulator haptic_simulator_launch.py motion_scaling:=10.0
ros2 launch haptic_navigation haptic_navigation_launch.py
ros2 topic pub --once /haptic_navigation/set/enabled sas_msgs/msg/Bool "{data: true}"
ros2 topic echo /haptic_navigation/get/status
```

See [haptic_tools_core](haptic_tools_core/README.md) for the equations and the parameters, and
[haptic_navigation](haptic_navigation/README.md) for the topics of the node.

## Docker

[devel/](devel/README.md) has the images and a compose file to run the haptic navigation example with the scene,
`haptic_simulator` or the Touch X, without installing the requirements:

### Haptic Simulator

```shell
cd devel/examples/haptic_navigation_example
docker compose up --build
```
### Real Touch X

```shell
cd devel/examples/haptic_navigation_example
docker compose --profile touch_x up --build  
```

## License

[LGPL-2.1](LICENSE).
