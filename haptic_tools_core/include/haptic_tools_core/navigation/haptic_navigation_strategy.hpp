/*
#    Copyright (c) 2026 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License.
#    If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#
# ################################################################
*/

/**
 * @file haptic_navigation_strategy.hpp
 * @brief Navigation mode of Aldhafeeri et al. (TAROS 2025): the twist of a command frame set with a haptic device.
 */

#pragma once
#include <dqrobotics/DQ.h>

namespace haptic_tools
{

/**
 * @brief Configuration of the HapticNavigationStrategy.
 *
 * The command frame F_E is the frame whose twist is commanded: an end effector, or the base of a mobile robot.
 * Rotations are unit quaternions, and vectors are pure quaternions (DQ Robotics), as in the paper.
 */
struct HapticNavigationStrategyConfiguration
{
    /// True: F_E moves on a plane (e.g. a holonomic base), and only [vx, vy, wz] are commanded.
    /// False: F_E moves in any direction (e.g. an end effector).
    bool planar_joint{false};
    /// r_H^E in p_d^E of Eq. (9): rotation of the device frame F_H relative to F_E (unit quaternion),
    /// p^E = Ad(r_H^E, p^H).
    DQ_robotics::DQ device_to_command_frame_rotation{1};
    /// r in Eq. (8) and in p_d^E of Eq. (9), before the motion scaling (m, positive).
    double dead_zone_radius{0.04};
    double stiffness{10.0};           ///< k_r in Eq. (7) (N/m, non-negative).
    double rotation_threshold{1.2};   ///< theta_limit in Eq. (8) (rad, positive).
    double yaw_rate{0.2};             ///< phi_dot_d in w^E of Eq. (11) (rad/s, non-negative).
    /// n_d^E in w^E of Eq. (11): rotation axis in F_E (unit pure quaternion). With planar_joint, k_ or -k_.
    DQ_robotics::DQ rotation_axis{DQ_robotics::k_};
};

/**
 * @brief Navigation mode (zeta = 0) of Aldhafeeri et al. (TAROS 2025): the twist of the command frame F_E, and the
 *        force rendered on the device. It does not use ROS.
 *
 * @details With p_h^H = s*p_device the position of the stylus (pure quaternion) scaled by the motion scaling s, and
 *          r_s = s*r the scaled dead zone:
 * @verbatim
   Eq. (7)   f_r^H = -k_r*p_h^H                                    (zero in NONE)
   Eq. (8)   TRANSLATION if ||p_h^H|| > r_s
             ROTATION    if ||p_h^H|| <= r_s and |theta| > theta_limit
             NONE        otherwise (the robot stops)
   Eq. (9)   t_dot_d^W = Ad(r_E^W, p_d^E),     p_d^E = (||p_h^H|| - r_s)*Ad(r_H^E, p_h^H/||p_h^H||)
   Eq. (11)  r_dot_d^W = 0.5*r_E^W*w^E,       w^E   = sign(theta)*phi_dot_d*n_d^E
   @endverbatim
 *
 * The strategy gives p_d^E and w^E, the twist of F_E expressed in F_E. The robot controller completes Eq. (9) and
 * (11) with the current rotation r_E^W of F_E (e.g. from its forward kinematics), and implements Eq. (10) and (12),
 * the velocity limits, and the obstacle avoidance. The clutch and the validity of the data are handled by
 * HapticNavigationSafety.
 *
 * B. Aldhafeeri, J. Carrasco, and B. V. Adorno, "A New Semi-Automatic Strategy for Teleoperating Mobile Manipulators
 * Using a Haptic Device," TAROS 2025, LNAI 16045, pp. 11-24, Springer, 2026. doi: 10.1007/978-3-032-01486-3_2
 */
class HapticNavigationStrategy
{
public:
    /// Submodes of Eq. (8).
    enum class SUBMODE
    {
        NONE,          ///< The robot stops.
        TRANSLATION,   ///< The stylus is outside the dead zone: p_d^E of Eq. (9).
        ROTATION       ///< The stylus is inside the dead zone, and |theta| > theta_limit: w^E of Eq. (11).
    };

    /// Result of compute().
    struct Output
    {
        SUBMODE submode{SUBMODE::NONE};      ///< Submode, Eq. (8).
        /// p_d^E of Eq. (9), in F_E (m/s, pure quaternion). With planar_joint, no k_ part.
        /// Eq. (9) itself is t_dot_d^W = Ad(r_E^W, p_d^E), computed by the robot controller.
        DQ_robotics::DQ linear_velocity{0};
        /// w^E of Eq. (11), in F_E (rad/s, pure quaternion). With planar_joint, only k_.
        /// Eq. (11) itself is r_dot_d^W = 0.5*r_E^W*w^E, computed by the robot controller.
        DQ_robotics::DQ angular_velocity{0};
        DQ_robotics::DQ force{0};            ///< f_r^H, in F_H (N, pure quaternion), Eq. (7). Zero in NONE.
        /// Ad(r_H^E, p_h^H): the scaled stylus position, in F_E (m, pure quaternion). Not in the paper: for display in
        /// the status (it is not used to compute the twist).
        DQ_robotics::DQ stylus_offset{0};
        double dead_zone_radius{0.0};        ///< r_s, the scaled dead zone radius (m).
    };

private:
    HapticNavigationStrategyConfiguration configuration_;   ///< Parameters of the strategy.

public:
    HapticNavigationStrategy() = delete;

    /**
     * @brief Creates the strategy.
     * @param configuration The parameters of the strategy.
     * @throws std::invalid_argument if the configuration is not valid.
     */
    explicit HapticNavigationStrategy(const HapticNavigationStrategyConfiguration& configuration);

    /**
     * @brief Evaluates Eq. (7) and (8), p_d^E of Eq. (9), and w^E of Eq. (11) for the current state of the device.
     * @param device_position Position of the stylus in the device frame, before the motion scaling (m, pure
     *        quaternion).
     * @param motion_scaling Motion scaling s of the device (positive).
     * @param theta Angle of the stylus joint used for the rotation (rad).
     * @return The submode, the twist of F_E, and the force on the device.
     * @throws std::invalid_argument if device_position is not a pure quaternion.
     * @pre The arguments are finite, and motion_scaling is positive.
     */
    Output compute(const DQ_robotics::DQ& device_position, const double& motion_scaling, const double& theta) const;

    /// @brief Returns the configuration of the strategy.
    /// @return The configuration.
    const HapticNavigationStrategyConfiguration& get_configuration() const;
};

}
