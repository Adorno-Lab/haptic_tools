# haptic_navigation

ROS 2 node of the navigation with a haptic device (e.g. Touch X): it runs `HapticNavigationStrategy` behind
`HapticNavigationSafety`, the navigation mode of Aldhafeeri et al. (TAROS 2025), with a device of
`sas_operator_side_receiver`. The equations, the parameters of the strategy, and the safety layer are described in
[haptic_tools_core](../haptic_tools_core/README.md#navigation).

## haptic_navigation_node

It reads the device of `sas_operator_side_receiver` (e.g. a Touch X through its UDP receiver, or `haptic_simulator`),
and runs the strategy behind the safety layer. Every cycle (`thread_sampling_time_sec`, 0.004 s by default) it takes
a snapshot of the last message of the device, and publishes the twist, the status, and the force. It does not wait
for the device: until its data is valid, the twist and the force are zero. When it stops (SIGINT, SIGTERM), it sends
a zero twist and a zero force.

| Topic | Type | |
|---|---|---|
| `/sas_operator_side_receiver/<interface_name>/get/information_from_master` | `sas_msgs/InformationFromMaster` | subscribed (best effort) |
| `/sas_operator_side_receiver/<interface_name>/set/information_to_master` | `sas_msgs/InformationToMaster` | the force, every cycle |
| `<topic_prefix>/set/enabled` | `sas_msgs/Bool` | subscribed |
| `<topic_prefix>/get/target_twist` | `geometry_msgs/TwistStamped` | every cycle, in $`\mathcal{F}_E`$ (`twist_frame_id`); zero when not active |
| `<topic_prefix>/get/status` | `haptic_tools_msgs/HapticNavigationStatus` | every cycle |

The parameters are in [config/default.yaml](config/default.yaml), with the defaults of
`semi_automatic_strategy_teleop`. Copy it for each robot, and set the ones that depend on the device and on the
robot (`planar_joint`, `device_to_command_frame_rotation`, `rotation_axis`, `twist_frame_id`).

The node does not convert the twist to the world frame, limit the velocities, or avoid obstacles: that is the job
of the controller of the robot, which receives the twist.

```shell
ros2 launch haptic_navigation haptic_navigation_launch.py config_file:=<robot>.yaml
```

With `start_udp_receiver:=true`, the launch file also starts the UDP receiver of `sas_operator_side_receiver`
(arguments `patient_side_ips`, `patient_side_ports`, `operator_side_ports`). Their defaults, `['192.168.8.100']`,
`[2222]`, and `[2223]`, are those of `sas_operator_side_receiver_launch.py` of
`semi_automatic_strategy_teleop/haptic_interface`. The receiver can also run alone, on the computer that receives the
data of the device (`udp_receiver_launch.py`).

Without a device, use `haptic_simulator` (in `haptic_tools`), which publishes the same messages at the rate of the
device, with the position and the joints of the stylus, the clutch, the buttons, and the motion scaling, and shows the
force:

```shell
ros2 launch haptic_simulator haptic_simulator_launch.py motion_scaling:=10.0
```

To try it without a robot, enable it and watch the twist and the status:

```shell
ros2 topic pub --once /haptic_navigation/set/enabled sas_msgs/msg/Bool "{data: true}"
ros2 topic echo /haptic_navigation/get/status
```
