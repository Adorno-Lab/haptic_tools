# haptic_pose_mapping_example

Example of the pose mapping (`HapticPoseMapping` of `haptic_tools_core`): a haptic device sets the
pose of the frame `xd` of [scenes/haptic_scene.ttt](../../scenes/haptic_scene.ttt), as seen from the camera of the
main view (`DefaultCamera`). `haptic_pose_mapping_example_node` (`HapticPoseMappingExample::_step()`,
`src/haptic_pose_mapping_example.cpp`) uses the frames `x`, `xd`, and `DefaultCamera` of the scene, published by
their `sas_object_script`:

- while the clutch is pressed, the motion of the device since the press (position and orientation) moves `xd`:
  moving the stylus to the right moves `xd` to the right in the main view, and rotating the stylus rotates `xd`
  about itself. The pose of `x`, the device, and the camera are latched at the press, so moving the main view with
  the mouse has no effect until the next press;
- when the clutch is released (or the device data is older than 0.1 s), `xd` is set to `x`, and held;
- with `x_follows_xd` (the default), `x` moves towards `xd` (along the screw between them, `pow()` of DQ Robotics), as
  a robot controller would do. A large motion is done in several strokes of the clutch.


```shell
ros2 launch haptic_pose_mapping_example haptic_pose_mapping_example_launch.py                # with haptic_simulator
ros2 launch haptic_pose_mapping_example haptic_pose_mapping_example_launch.py device:=udp    # with the Touch X
```

The parameters are in [config/haptic_pose_mapping_example.yaml](config/haptic_pose_mapping_example.yaml):
`motion_scaling` (of the translation of the device), `max_translation` (of `xd` in a press), and
`camera_to_device_rotation` (`[0, 0, 1, 0]` for a CoppeliaSim camera; see the other conventions in
[haptic_tools_core](../../haptic_tools_core/README.md#pose-mapping)).
