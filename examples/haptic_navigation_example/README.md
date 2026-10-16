# haptic_navigation_example

Example of the haptic navigation (the navigation mode of Aldhafeeri et al., TAROS 2025): a haptic device drives the
holonomic base of [scenes/haptic_scene.ttt](../../scenes/haptic_scene.ttt). `haptic_navigation_example_node`
(`HapticNavigationExample::_step()`, `src/haptic_navigation_example.cpp`) uses `HapticNavigationStrategy` and
`HapticNavigationSafety` of `haptic_tools_core` directly: every cycle (4 ms) it reads the last message of the device,
computes the twist of the base and the restoring force, publishes the twist, and sends the force to the device.

| Topic | Type | |
|---|---|---|
| `/sas_operator_side_receiver/m0_0/get/information_from_master` | `sas_msgs/InformationFromMaster` | subscribed: the device |
| `/sas_operator_side_receiver/m0_0/set/information_to_master` | `sas_msgs/InformationToMaster` | the force on the device |
| `/sas_b1/b1_1/set/holonomic_target_twist` | `geometry_msgs/TwistStamped` | the twist of the base (vx, vy, ωz, in the base frame), for the script `holonomic_cmd` of the scene |
| `/haptic_navigation_example/get/status` | `haptic_tools_msgs/HapticNavigationStatus` | the dead zone, the stylus, its angle, and the submode, for the GUI |

```shell
ros2 launch haptic_navigation_example haptic_navigation_example_launch.py                # with haptic_simulator
ros2 launch haptic_navigation_example haptic_navigation_example_launch.py device:=udp    # with the Touch X
```

The launch file also starts `haptic_navigation_example_gui` (`gui:=false` to skip it), a basic GUI
(`NavigationView::paintEvent()`, `src/navigation_view.cpp`) that shows the current mode: a top view of the base with
the dead zone (dashed circle) and the stylus (a dot: green in translation, orange in rotation, blue when stopped in
the dead zone, grey when the clutch is released), and a bar of the stylus angle θ, with the rotation zones
beyond ±θ_limit in orange.

Press the clutch (the space bar in `haptic_simulator`, which starts with a motion scaling of 10), and:

- push the stylus out of the dead zone (4 cm, before the motion scaling) to move the base in that direction: away
  from you is forward, to the right is to the right (`device_to_command_frame_rotation`);
- with the stylus in the dead zone, turn its last joint beyond 1.2 rad to rotate the base (to the left for a positive
  angle, `rotation_axis`).

The device pulls the stylus back to the centre while the base moves. The base stops when the clutch is released, when
the device data is older than 0.1 s, and when the node stops. If the clutch is already pressed when the data
arrives, release it and press it again. The parameters are in
[config/haptic_navigation_example.yaml](config/haptic_navigation_example.yaml); the equations are in
[haptic_tools_core](../../haptic_tools_core/README.md#navigation). For the complete node (enabled on a topic, with a
status message), see [haptic_navigation](../../haptic_navigation/README.md).
