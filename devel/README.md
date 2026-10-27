# devel

Docker images and compose files to run the examples.

| Folder | |
|---|---|
| [haptic_tools_jazzy](haptic_tools_jazzy/Dockerfile) | Image with the packages of this repository built (`murilomarinho/sas:jazzy`, plus `sas_operator_side_receiver` and Qt 6). |
| [haptic_tools_jazzy_coppeliasim](haptic_tools_jazzy_coppeliasim/Dockerfile) | Image with CoppeliaSim 4.7 to run the scenes of [scenes/](../scenes) (`murilomarinho/coppeliasim:noble_470rev4`); the scenes are mounted, not copied. |
| [examples/haptic_navigation_example](examples/haptic_navigation_example/compose.yaml) | The haptic navigation example: the scene, the example node, its GUI, and the haptic device (`haptic_simulator`, or the Touch X). |
| [examples/haptic_pose_mapping_example](examples/haptic_pose_mapping_example/compose.yaml) | The pose mapping example: the scene, the example node, and the haptic device (`haptic_simulator`, or the Touch X). |

Run an example (the images are built the first time), then start the simulation in CoppeliaSim:

```shell
cd devel/examples/haptic_navigation_example   # or devel/examples/haptic_pose_mapping_example
xhost +local:root
docker compose up --build                     # with haptic_simulator (press the clutch with the space bar)
docker compose --profile touch_x up --build   # with the real Touch X
```

The containers use the host network, and `ROS_DOMAIN_ID=77` (set in the images). Do not run another source of the
same device (e.g. another UDP receiver of the Touch X) at the same time, on the same `ROS_DOMAIN_ID`, nor both
examples at the same time. With the Touch X, this computer must have the address `192.168.8.100`, on the network of
the Windows computer (see the comments in the `compose.yaml` of each example).
