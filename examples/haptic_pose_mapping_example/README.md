# haptic_pose_mapping_example

Example of the pose mapping (`HapticPoseMapping` of `haptic_tools_core`, not from the paper): a haptic device sets the
pose of the frame `xd` of [scenes/pose_mapping.ttt](../../scenes/pose_mapping.ttt), as seen from the camera of the
main view (`DefaultCamera`). `haptic_pose_mapping_example_node` (`HapticPoseMappingExample::_step()`,
`src/haptic_pose_mapping_example.cpp`) uses the frames `x` (the end effector), `xd` (the desired pose), and
`DefaultCamera` of the scene, published by their `sas_object_script`:

- while the clutch is pressed, the motion of the device since the press (position and orientation) moves `xd`,
  starting from the pose of `x` at the press: moving the stylus to the right moves `xd` to the right in the main view,
  and rotating the stylus rotates `xd` about itself. The poses of `x`, the device, and the camera are latched at the
  press, so moving the main view with the mouse has no effect until the next press;
- the translation of the device is scaled by the motion scaling sent with the device data (`motion_scaling` of
  `sas_msgs/InformationFromMaster`: set in the master, or the `motion_scaling` argument of `haptic_simulator`), also
  latched at the press. If it is not positive, `motion_scaling` of the configuration is used;
- when the clutch is released (or the device data is older than 0.1 s), `xd` is set to `x`.

The example does not move `x`. With a robot, a kinematic controller moves the end effector towards `xd`, and `x` is
its pose (e.g. from the forward kinematics). Here, `x` can be moved with the mouse in CoppeliaSim.

| Topic | Type | |
|---|---|---|
| `/sas_operator_side_receiver/m0_0/get/information_from_master` | `sas_msgs/InformationFromMaster` | subscribed: the device |
| `/sas_robot_driver_coppeliasim/object/x/get/pose` | `geometry_msgs/PoseStamped` | subscribed: `x`, read every cycle |
| `/sas_robot_driver_coppeliasim/object/DefaultCamera/get/pose` | `geometry_msgs/PoseStamped` | subscribed: the camera, read at each press |
| `/sas_robot_driver_coppeliasim/object/xd/set/pose` | `geometry_msgs/PoseStamped` | `xd`, every cycle |

```shell
ros2 launch haptic_pose_mapping_example haptic_pose_mapping_example_launch.py                # with haptic_simulator
ros2 launch haptic_pose_mapping_example haptic_pose_mapping_example_launch.py device:=udp    # with the Touch X
```

The parameters are in [config/haptic_pose_mapping_example.yaml](config/haptic_pose_mapping_example.yaml):
`motion_scaling` (used when the device data has no valid motion scaling), `max_translation` (of `xd` in a press), and
`camera_to_device_rotation` (`[0, 0, 1, 0]` for a CoppeliaSim camera; see the other conventions in
[haptic_tools_core](../../haptic_tools_core/README.md#pose-mapping)).
