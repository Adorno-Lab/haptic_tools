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
 * @file haptic_navigation_safety.cpp
 * @brief Implementation of HapticNavigationSafety.
 */

#include <haptic_navigation/haptic_navigation_safety.hpp>
#include <haptic_navigation/dq_validation.hpp>
#include <cmath>
#include <stdexcept>

namespace haptic_tools
{

HapticNavigationSafety::HapticNavigationSafety(const HapticNavigationSafetyConfiguration& configuration):
    configuration_(configuration)
{
    if (!(std::isfinite(configuration_.data_timeout) && configuration_.data_timeout > 0.0))
        throw std::invalid_argument("HapticNavigationSafety: data_timeout must be positive.");
    if (!(std::isfinite(configuration_.max_force) && configuration_.max_force >= 0.0))
        throw std::invalid_argument("HapticNavigationSafety: max_force must be non-negative.");
}

HapticNavigationSafety::Output HapticNavigationSafety::update(const Input& input)
{
    Output output;
    output.device_data_valid = input.data_received
                               && std::isfinite(input.data_age) && input.data_age <= configuration_.data_timeout
                               && input.data_finite
                               && std::isfinite(input.motion_scaling) && input.motion_scaling > 0.0;
    output.clutch = output.device_data_valid && input.clutch;

    // The clutch must be released after the navigation is disabled or the data is not valid.
    if (!input.enabled || !output.device_data_valid)
        clutch_released_since_last_stop_ = false;
    else if (!output.clutch)
        clutch_released_since_last_stop_ = true;

    output.active = input.enabled && output.clutch && clutch_released_since_last_stop_;
    output.waiting_for_clutch_release = input.enabled && output.clutch && !clutch_released_since_last_stop_;
    return output;
}

DQ_robotics::DQ HapticNavigationSafety::limit_force(const DQ_robotics::DQ& force) const
{
    // Only a finite pure quaternion is a force: anything else is not sent to the device.
    if (!is_finite_pure_quaternion(force))
        return DQ_robotics::DQ(0);
    const double norm = force.vec3().norm();
    if (norm > configuration_.max_force)
        return (configuration_.max_force/norm)*force;
    return force;
}

const HapticNavigationSafetyConfiguration& HapticNavigationSafety::get_configuration() const
{
    return configuration_;
}

}
