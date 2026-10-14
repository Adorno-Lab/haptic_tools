# Status of haptic_navigation_node. Published every cycle on <prefix>/get/status.
std_msgs/Header header

# The submode of the navigation strategy, Eq. (8). NONE when the robot stops (stylus in the dead zone and its angle
# below the threshold), and whenever the strategy is not active (see below).
uint8 SUBMODE_NONE=0
uint8 SUBMODE_TRANSLATION=1
uint8 SUBMODE_ROTATION=2
uint8 submode

# The strategy is active (it commands a twist and renders a force) only if enabled, device_data_valid, and clutch,
# and the clutch was released since the navigation was enabled or the data became valid.
bool enabled                           # Last value received on <prefix>/set/enabled (sas_msgs/Bool).
bool device_data_valid                 # The device is connected, and its data is fresher than data_timeout.
bool clutch                            # The clutch is pressed (false whenever device_data_valid is false).
bool waiting_for_clutch_release        # Enabled with the clutch pressed: release it, and press it again to move.
bool active                            # The strategy commands the twist and renders the force.

# To draw the strategy (e.g. a top view of the command frame, with the dead zone as a circle and the stylus as a
# point, and a gauge of the stylus angle against its threshold). The symbols are those of the navigation mode of
# Aldhafeeri et al. (TAROS 2025), Eq. (7)-(11); the position and the radius are scaled by the motion scaling s.
# The command frame F_E is the frame whose twist is commanded (an end effector, or the base of a mobile robot).
geometry_msgs/Vector3 stylus_offset    # p_h^H expressed in F_E, Ad(r_H^E)*p_h^H (m). The dead zone is a sphere.
                                       # Not in the paper: for display (not used to compute the twist).
float64 dead_zone_radius               # r, scaled: s*dead_zone_radius (m).
float64 gimbal_roll                    # theta, the joint angle of the stylus used for the rotation (rad).
float64 rotation_threshold             # theta_limit: ROTATION when ||p_h^H|| <= r and |theta| > theta_limit (rad).

geometry_msgs/Twist twist              # Twist published on <prefix>/get/target_twist (in F_E): p_d^E of Eq. (9) and w^E of Eq. (11).
geometry_msgs/Wrench wrench            # Wrench sent to the device (device frame).
