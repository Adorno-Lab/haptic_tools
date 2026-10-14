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
 * @file haptic_navigation_strategy.cpp
 * @brief Implementation of HapticNavigationStrategy.
 */

#include <haptic_tools_core/navigation/haptic_navigation_strategy.hpp>
#include <haptic_tools_core/dq_validation.hpp>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace DQ_robotics;

namespace haptic_tools
{

namespace
{

/// @brief Throws std::invalid_argument with @p message if @p condition is false.
void _check(const bool& condition, const std::string& message)
{
    if (!condition)
        throw std::invalid_argument("HapticNavigationStrategy: " + message);
}

/**
 * @brief Validates the configuration of the strategy, and normalizes its rotation and its axis.
 * @param c The configuration.
 * @return c, with device_to_command_frame_rotation and rotation_axis normalized.
 * @throws std::invalid_argument if it is not valid.
 */
HapticNavigationStrategyConfiguration _validate(HapticNavigationStrategyConfiguration c)
{
    c.device_to_command_frame_rotation = to_unit_quaternion(c.device_to_command_frame_rotation,
                                                            "HapticNavigationStrategy: device_to_command_frame_rotation");
    c.rotation_axis = to_unit_pure_quaternion(c.rotation_axis, "HapticNavigationStrategy: rotation_axis");
    _check(std::isfinite(c.dead_zone_radius) && c.dead_zone_radius > 0.0, "dead_zone_radius must be positive.");
    _check(std::isfinite(c.stiffness) && c.stiffness >= 0.0, "stiffness must be non-negative.");
    _check(std::isfinite(c.rotation_threshold) && c.rotation_threshold > 0.0, "rotation_threshold must be positive.");
    _check(std::isfinite(c.yaw_rate) && c.yaw_rate >= 0.0, "yaw_rate must be non-negative.");
    _check(!c.planar_joint || c.rotation_axis.vec3().head<2>().norm() <= kUnitTolerance,
           "with planar_joint, rotation_axis must be k_ or -k_.");
    return c;
}

}

HapticNavigationStrategy::HapticNavigationStrategy(const HapticNavigationStrategyConfiguration& configuration):
    configuration_(_validate(configuration))
{
    // With planar_joint, the axis is exactly +-k_.
    if (configuration_.planar_joint)
        configuration_.rotation_axis = std::copysign(1.0, configuration_.rotation_axis.q(3))*k_;
}

HapticNavigationStrategy::Output HapticNavigationStrategy::compute(const DQ& device_position,
                                                                   const double& motion_scaling,
                                                                   const double& theta) const
{
    if (!is_finite_pure_quaternion(device_position))
        throw std::invalid_argument("HapticNavigationStrategy::compute: device_position must be a pure quaternion.");
    const HapticNavigationStrategyConfiguration& c = configuration_;
    const DQ& r_H_E = c.device_to_command_frame_rotation;
    Output output;

    // p_h^H and r, scaled by the motion scaling (as in semi_automatic_strategy_teleop).
    const DQ p_h = motion_scaling*device_position;
    const double r = motion_scaling*c.dead_zone_radius;
    const double norm_p_h = p_h.vec3().norm();
    output.stylus_offset = Ad(r_H_E, p_h);   // not in the paper: for display in the status
    output.dead_zone_radius = r;

    // Eq. (8): submodes.
    if (norm_p_h > r)
    {
        // p_d^E of Eq. (9): (||p_h^H|| - r)*Ad(r_H^E, p_bar_h^H). A planar joint does not move vertically.
        output.submode = SUBMODE::TRANSLATION;
        output.linear_velocity = (norm_p_h - r)*Ad(r_H_E, (1.0/norm_p_h)*p_h);
        if (c.planar_joint)
            output.linear_velocity = output.linear_velocity.q(1)*i_ + output.linear_velocity.q(2)*j_;
    }
    else if (std::abs(theta) > c.rotation_threshold)
    {
        // omega_{W,E}^E of Eq. (11): phi_dot_d*n_d^E, with the sign of theta. With a planar joint, n_d^E = +-k_.
        output.submode = SUBMODE::ROTATION;
        output.angular_velocity = (std::copysign(1.0, theta)*c.yaw_rate)*c.rotation_axis;
        if (c.planar_joint)
            output.angular_velocity = output.angular_velocity.q(3)*k_;
    }
    else
    {
        // The robot stops completely, and the force is zero (as in semi_automatic_strategy_teleop).
        output.submode = SUBMODE::NONE;
        return output;
    }

    // Eq. (7): restoring force, while the robot moves.
    output.force = -c.stiffness*p_h;
    return output;
}

const HapticNavigationStrategyConfiguration& HapticNavigationStrategy::get_configuration() const
{
    return configuration_;
}

}
