# haptic_navigation

Navigation with a haptic device (e.g. Touch X). It implements the navigation mode ($`\zeta = 0`$) of

> B. Aldhafeeri, J. Carrasco, and B. V. Adorno, "A New Semi-Automatic Strategy for Teleoperating Mobile
> Manipulators Using a Haptic Device," in *Towards Autonomous Robotic Systems (TAROS 2025)*, LNAI 16045,
> pp. 11–24, Springer, 2026. [doi:10.1007/978-3-032-01486-3_2](https://doi.org/10.1007/978-3-032-01486-3_2)

as implemented in `semi_automatic_strategy_teleop` (project-clerice). The equation numbers below are those of the
paper. The package also has a pose mapping (`HapticPoseMapping`, see [Pose mapping](#pose-mapping-hapticposemapping)),
which is not from the paper.

The strategy (`HapticNavigationStrategy`) gives the twist of a **command frame** $`\mathcal{F}_E`$ and the force to
render on the device. It is a C++ library that does not use ROS. $`\mathcal{F}_E`$ is the frame whose motion is
commanded:

- **An end effector** (`planar_joint: false`): it moves in any direction, and the full twist
  is used.
- **A holonomic mobile base** (`planar_joint: true`, e.g. the Unitree G1): only $`[v_x, v_y, \omega_z]`$ are used.

## Navigation mode

Let $`s`$ be the motion scaling of the device (`motion_scaling` of `sas_msgs/InformationFromMaster`). As in
`semi_automatic_strategy_teleop`, the position of the stylus and the radius of the dead zone are scaled by $`s`$:

```math
\boldsymbol{p}_h^H = s\,\boldsymbol{p}_{\text{device}}, \qquad r = s\,r_0,
```

with $`\boldsymbol{p}_h^H`$ the position of the stylus in the device frame $`\mathcal{F}_H`$, and $`r_0`$ the parameter
`dead_zone_radius`. Scaling both leaves the dead zone unchanged ($`\|\boldsymbol{p}_h^H\| > r \iff
\|\boldsymbol{p}_{\text{device}}\| > r_0`$), and $`s`$ acts as the gain of the translation.

**Restoring force (7)**, rendered on the device, in $`\mathcal{F}_H`$, while the robot moves (translation or rotation):

```math
\boldsymbol{f}_r^H = -k_r\,\boldsymbol{p}_h^H.
```

**Submodes (8)**, with $`\theta`$ the joint angle of the stylus controlled by the user:

```math
\mathcal{K} = \begin{cases} 0 \;(\text{translation}), & \text{if } \|\boldsymbol{p}_h^H\| > r,\\
1 \;(\text{rotation}), & \text{if } \|\boldsymbol{p}_h^H\| \le r \text{ and } |\theta| > \theta_{\text{limit}}.\end{cases}
```

When $`\|\boldsymbol{p}_h^H\| \le r`$ and $`|\theta| \le \theta_{\text{limit}}`$, the robot stops completely, and the force is
zero ($`\boldsymbol{f}_r^H = \boldsymbol{0}`$), as in `semi_automatic_strategy_teleop`.

**Translation command (9)**: the desired linear velocity of $`\mathcal{F}_E`$ in the world frame is

```math
\dot{\boldsymbol{t}}_d^W = \mathrm{Ad}(\boldsymbol{r}_E^W)\,\boldsymbol{p}_d^E, \qquad
\boldsymbol{p}_d^E = \left(\|\boldsymbol{p}_h^H\| - r\right)\mathrm{Ad}(\boldsymbol{r}_H^E)\,\bar{\boldsymbol{p}}_h^H,
\qquad \bar{\boldsymbol{p}}_h^H = \boldsymbol{p}_h^H/\|\boldsymbol{p}_h^H\|,
```

with $`\boldsymbol{r}_H^E`$ the constant rotation of $`\mathcal{F}_H`$ relative to $`\mathcal{F}_E`$, determined by
inspection. The strategy gives $`\boldsymbol{p}_d^E`$, the velocity expressed in $`\mathcal{F}_E`$. The robot controller
completes Eq. (9) with the current rotation $`\boldsymbol{r}_E^W`$ of $`\mathcal{F}_E`$ (e.g. from its forward
kinematics).

- `planar_joint: false`: the full $`\boldsymbol{p}_d^E`$, including up and down.
- `planar_joint: true`: $`[v_x,\, v_y]`$ of $`\boldsymbol{p}_d^E`$, and $`v_z = 0`$.

The dead zone is the sphere of the paper in both cases ($`\|\boldsymbol{p}_h^H\|`$ includes the vertical axis of the
device).

**Yaw command (11)**: the desired time derivative of the rotation of $`\mathcal{F}_E`$ is

```math
\dot{\boldsymbol{r}}_d^W = \tfrac{1}{2}\,\boldsymbol{r}_E^W\,\boldsymbol{\omega}_{W,E}^E, \qquad
\boldsymbol{\omega}_{W,E}^E = \mathrm{sign}(\theta)\,\dot{\phi}_d\,\boldsymbol{n}_d^E,
```

the angular velocity expressed in $`\mathcal{F}_E`$, with the constant rate $`\dot{\phi}_d`$ about the axis
$`\boldsymbol{n}_d^E`$, and the sign of $`\theta`$ (as in `semi_automatic_strategy_teleop`; the paper uses
$`\dot{\phi}_d\,\boldsymbol{n}_d^E`$). The strategy gives $`\boldsymbol{\omega}_{W,E}^E`$; the robot controller completes
Eq. (11) with $`\boldsymbol{r}_E^W`$. With
`planar_joint: true`, $`\boldsymbol{n}_d^E`$ must be $`\hat{k}`$ (a positive $`\theta`$ turns left) or $`-\hat{k}`$, so only
$`\omega_z`$ is used.

### Not included

These parts of the paper belong to the controller of the robot, which receives the twist:

- the expression of the twist in the world frame (Eq. (9) and (11), with $`\boldsymbol{r}_E^W`$);
- keeping the orientation fixed during the translation (first term of Eq. (10));
- aligning the camera with the ground (Eq. (12));
- the velocity limits and the obstacle avoidance (VFIs, Eq. (1) and (6)).

## Parameters of the strategy

As in the paper, rotations are unit quaternions and vectors are pure quaternions (`DQ_robotics::DQ`): the
configuration, the position of the stylus given to `compute()`, and its outputs (the velocities, the force, and the
stylus offset).

| Field of `HapticNavigationStrategyConfiguration` | Symbol | Default |
|---|---|---|
| `planar_joint` | — | `false` (the end effector of the paper) |
| `device_to_command_frame_rotation` | $`\boldsymbol{r}_H^E`$ (unit quaternion), in $`\boldsymbol{p}_d^E`$ of Eq. (9) | $`1`$ |
| `dead_zone_radius` | $`r_0`$ (m), Eq. (8) and $`\boldsymbol{p}_d^E`$ of Eq. (9) | 0.04 |
| `stiffness` | $`k_r`$ (N/m), Eq. (7) | 10.0 |
| `rotation_threshold` | $`\theta_{\text{limit}}`$ (rad), Eq. (8) | 1.2 |
| `yaw_rate` | $`\dot{\phi}_d`$ (rad/s), in $`\boldsymbol{\omega}_{W,E}^E`$ of Eq. (11) | 0.2 |
| `rotation_axis` | $`\boldsymbol{n}_d^E`$ (unit pure quaternion, in $`\mathcal{F}_E`$), in $`\boldsymbol{\omega}_{W,E}^E`$ of Eq. (11) | $`\hat{k}`$ |

The default values of `device_to_command_frame_rotation` and `rotation_axis` depend on the device and on the robot, 
and must be determined by inspection. In the parameters of `haptic_navigation_node`, they are written as `[w, x, y, z]` and `[x, y, z]`.


## Safety

The safety layer (`HapticNavigationSafety`, also without ROS) is not part of the strategy, so that it never changes
its equations. It decides when the strategy is active, and limits the force. The strategy is active only if all
of these hold; otherwise, the twist and the force are zero:

- the navigation is enabled (by the supervisor of the robot, on `<prefix>/set/enabled`);
- the device data is valid: received, younger than `data_timeout`, finite, and with a positive motion scaling;
- the clutch is pressed, as a deadman;
- the clutch was released since the navigation was enabled or the data became valid, so that the robot does not
  start moving while the clutch is already pressed with the stylus outside the dead zone.

The force sent to the device is limited to `max_force` (the Touch X sustains about 1.75 N), and it is sent every
cycle, also when it is zero, so that a stale force does not stay on the device.

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

## Pose mapping (`HapticPoseMapping`)

> **Not from the paper.** The pose mapping is not part of Aldhafeeri et al. (TAROS 2025): it is an open-source
> implementation of teleoperation features available in the non-commercial SmartArmStack (SAS) packages.

The library also has `HapticPoseMapping` ([haptic_pose_mapping.hpp](include/haptic_navigation/haptic_pose_mapping.hpp),
without ROS): it sets the **desired pose** of an end effector (position and orientation) with the pose of the device
and its clutch, as seen from a reference camera. It is used by the lever tasks of the G1 (`g1_lever_demo`,
`xd_source: haptic`): pressing the clutch moves `xd` with the Touch X, and releasing it puts `xd` at the palm.

The device frame $`\mathcal{F}_H`$ has $`x`$ to the right, $`y`$ up, and $`z`$ towards the user. A CoppeliaSim camera looks
along its $`+z`$ axis with $`+y`$ up (its $`+x`$ points to the left of the image), so the camera rotated by
$`\boldsymbol{r}_{\text{off}}`$, 180° about $`y`$, has the axes of $`\mathcal{F}_H`$ as seen by the user in front of the
image.

At the press of the clutch, with $`\boldsymbol{\underline{x}}_{h1}`$ the pose of the device, $`\boldsymbol{\underline{x}}_{e1}`$ the
pose of the end effector, $`\boldsymbol{r}_{c1}`$ the rotation of the camera, and $`s`$ the motion scaling, everything is
latched:

```math
\boldsymbol{r}_{\text{ref}} = \boldsymbol{r}_{c1}\boldsymbol{r}_{\text{off}}, \qquad
\boldsymbol{\underline{x}}_{\text{ref}} = \boldsymbol{r}_{\text{ref}} + \tfrac{1}{2}\varepsilon\,\boldsymbol{t}_{e1}\boldsymbol{r}_{\text{ref}},
```

a frame at the end effector with the axes of the device as seen in the image. While the clutch is pressed, with
$`\boldsymbol{\underline{x}}_h = \boldsymbol{r}_h + \frac{1}{2}\varepsilon\,\boldsymbol{p}_h\boldsymbol{r}_h`$ the pose of the device:

```math
\boldsymbol{r}_\Delta = \boldsymbol{r}_h\boldsymbol{r}_{h1}^*, \qquad
\boldsymbol{p}_\Delta = s\,(\boldsymbol{p}_h - \boldsymbol{p}_{h1}), \qquad
\boldsymbol{\underline{x}}_\Delta = \boldsymbol{r}_\Delta + \tfrac{1}{2}\varepsilon\,\boldsymbol{p}_\Delta\boldsymbol{r}_\Delta,
```

```math
\boldsymbol{\underline{x}}_d = \boldsymbol{\underline{x}}_{\text{ref}}\,\boldsymbol{\underline{x}}_\Delta\,
\boldsymbol{\underline{x}}_{\text{ref}}^*\,\boldsymbol{\underline{x}}_{e1},
```

that is, $`\boldsymbol{t}_d = \boldsymbol{t}_{e1} + \mathrm{Ad}(\boldsymbol{r}_{\text{ref}})\,\boldsymbol{p}_\Delta`$ and
$`\boldsymbol{r}_d = (\boldsymbol{r}_{\text{ref}}\boldsymbol{r}_\Delta\boldsymbol{r}_{\text{ref}}^*)\,\boldsymbol{r}_{e1}`$:
the motion of the device since the press, applied to the end effector in the frame of the camera, rotating about the
end effector. Compared with the formula of `semi_automatic_strategy_teleop/haptic_interface`:

- **Position control.** The pose of the end effector is latched at the press. The original placed
  $`\boldsymbol{\underline{x}}_{\text{ref}}`$ at the *current* end effector in every cycle, so $`\boldsymbol{t}_d`$ stayed a constant offset ahead of the
  hand, which kept moving while the stylus was held still (rate control).
- **Increments in the device frame.** $`\boldsymbol{r}_h\boldsymbol{r}_{h1}^*`$ and $`\boldsymbol{p}_h - \boldsymbol{p}_{h1}`$, not
  $`\boldsymbol{\underline{x}}_{h1}^*\boldsymbol{\underline{x}}_h`$ (the motion in the frame of the stylus at the press): moving the stylus to the right
  moves the end effector to the right in the image, however the stylus is held.
- **The camera and the scaling are latched too**, so moving the camera (e.g. the main view with the mouse) or
  changing the scaling has no effect until the next press.

Optionally, $`\|\boldsymbol{p}_\Delta\|`$ is limited (`max_translation`). The class does not handle the clutch: its user
calls `engage()` at the press, `desired_pose()` while it is pressed, and `disengage()` at the release. The tests are in
[test/test_haptic_pose_mapping.cpp](test/test_haptic_pose_mapping.cpp).
