# haptic_tools_core

The haptic tools without ROS: C++ libraries based on DQ Robotics. The ROS 2 node of the navigation is in [haptic_navigation](../haptic_navigation/README.md).

| Folder | Content |
|---|---|
| [navigation/](include/haptic_tools_core/navigation) | `HapticNavigationStrategy` and `HapticNavigationSafety`: the navigation mode of Aldhafeeri et al. (TAROS 2025), and its safety layer. See the section Navigation below. |
| [pose_mapping/](include/haptic_tools_core/pose_mapping) | `HapticPoseMapping`: the desired pose of an end effector set with the pose of the device and its clutch. Not from the paper. See the section Pose mapping below. |
| [dq_validation.hpp](include/haptic_tools_core/dq_validation.hpp) | The checks of the unit quaternions, unit dual quaternions, and pure quaternions given to the libraries. |

Use it from another package:

```cmake
find_package(haptic_tools_core REQUIRED)
target_link_libraries(<target> haptic_tools_core::haptic_tools_core)
```

```cpp
#include <haptic_tools_core/navigation/haptic_navigation_strategy.hpp>
#include <haptic_tools_core/pose_mapping/haptic_pose_mapping.hpp>
```

and `<depend>haptic_tools_core</depend>` in its `package.xml`. The classes are in the namespace `haptic_tools`.

## Navigation

The navigation mode ($`\zeta = 0`$) of

> B. Aldhafeeri, J. Carrasco, and B. V. Adorno, "A New Semi-Automatic Strategy for Teleoperating Mobile
> Manipulators Using a Haptic Device," in *Towards Autonomous Robotic Systems (TAROS 2025)*, LNAI 16045,
> pp. 11–24, Springer, 2026. [doi:10.1007/978-3-032-01486-3_2](https://doi.org/10.1007/978-3-032-01486-3_2)

as implemented in `semi_automatic_strategy_teleop` (project-clerice). The equation numbers below are those of the
paper.

`HapticNavigationStrategy` gives the twist of a **command frame** $`\mathcal{F}_E`$ and the force to
render on the device. $`\mathcal{F}_E`$ is the frame whose motion is
commanded:

- **An end effector** (`planar_joint: false`): it moves in any direction, and the full twist
  is used.
- **A holonomic mobile base** (`planar_joint: true`, e.g. the Unitree G1): only $`[v_x, v_y, \omega_z]`$ are used.

### Navigation mode

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

#### Not included

These parts of the paper belong to the controller of the robot, which receives the twist:

- the expression of the twist in the world frame (Eq. (9) and (11), with $`\boldsymbol{r}_E^W`$);
- keeping the orientation fixed during the translation (first term of Eq. (10));
- aligning the camera with the ground (Eq. (12));
- the velocity limits and the obstacle avoidance (VFIs, Eq. (1) and (6)).

### Parameters of the strategy

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

### Safety

The safety layer (`HapticNavigationSafety`, also without ROS) is not part of the strategy, so that it never changes
its equations. It decides when the strategy is active, and limits the force. The strategy is active only if all
of these hold; otherwise, the twist and the force are zero:

- the navigation is enabled (e.g. by the supervisor of the robot, on `<prefix>/set/enabled` of
  `haptic_navigation_node`);
- the device data is valid: received, younger than `data_timeout`, finite, and with a positive motion scaling;
- the clutch is pressed, as a deadman;
- the clutch was released since the navigation was enabled or the data became valid, so that the robot does not
  start moving while the clutch is already pressed with the stylus outside the dead zone.

The force sent to the device is limited to `max_force` (the Touch X sustains about 1.75 N), and it is sent every
cycle, also when it is zero, so that a stale force does not stay on the device.
## Pose mapping

> **Not from the paper.** The pose mapping is not part of Aldhafeeri et al. (TAROS 2025): it is an open-source
> implementation of teleoperation features available in the non-commercial SmartArmStack (SAS) packages.

`HapticPoseMapping` ([haptic_pose_mapping.hpp](include/haptic_tools_core/pose_mapping/haptic_pose_mapping.hpp),
without ROS): it sets the **desired pose** of an end effector (position and orientation) with the pose of the device
and its clutch, as seen from a reference camera.

The device frame $`\mathcal{F}_H`$ has $`x`$ to the right, $`y`$ up, and $`z`$ towards the user. The camera rotated by
$`\boldsymbol{r}_{\text{off}}`$ has the axes of $`\mathcal{F}_H`$ as seen by the user in front of the image.
$`\boldsymbol{r}_{\text{off}}`$ depends on the convention of the camera frame of the simulator, and it is the second
argument of the constructor (`camera_to_device_rotation`, a unit quaternion):

| Camera frame | $`\boldsymbol{r}_{\text{off}}`$ |
|---|---|
| CoppeliaSim: looks along $`+z`$, $`+y`$ up ($`+x`$ to the left of the image) | $`\hat{\jmath}`$, 180° about $`y`$ (the default, `coppeliasim_camera_to_device_rotation()`) |
| ROS optical frame: looks along $`+z`$, $`+y`$ down ($`+x`$ to the right) | $`\hat{\imath}`$, 180° about $`x`$ |
| ROS body frame: looks along $`+x`$, $`+y`$ to the left, $`+z`$ up | $`\tfrac{1}{2}(1 + \hat{\imath} - \hat{\jmath} - \hat{k})`$, 120° about $`(\hat{\imath} - \hat{\jmath} - \hat{k})/\sqrt{3}`$ |
