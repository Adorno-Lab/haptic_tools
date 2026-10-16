# Examples

An example with the CoppeliaSim scene [scenes/haptic_scene.ttt](../scenes/haptic_scene.ttt). It uses the libraries
of [haptic_tools_core](../haptic_tools_core/README.md) directly, with a haptic device of `sas_operator_side_receiver`
(the Touch X, or `haptic_simulator`).

| Package | Example |
|---|---|
| [haptic_navigation_example](haptic_navigation_example/README.md) | The haptic navigation: the device drives the holonomic base of the scene, by publishing its twist. |

To run it: open the scene in CoppeliaSim (4.7, with the ROS 2 interface and `sas_robot_driver_coppeliasim`), start
its simulation, and launch the example (with `haptic_simulator` by default, or `device:=udp` for the Touch X).
