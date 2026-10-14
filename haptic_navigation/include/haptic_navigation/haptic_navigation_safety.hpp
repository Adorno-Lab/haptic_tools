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
 * @file haptic_navigation_safety.hpp
 * @brief Safety layer of the haptic navigation: when the strategy is active, and the force limit.
 */

#pragma once
#include <dqrobotics/DQ.h>

namespace haptic_tools
{

/// @brief Configuration of the HapticNavigationSafety.
struct HapticNavigationSafetyConfiguration
{
    double data_timeout{0.1};   ///< The device data is valid while its last message is younger than this (s, positive).
    double max_force{1.5};      ///< Maximum norm of the force sent to the device (N, non-negative). The Touch X sustains about 1.75 N.
};

/**
 * @brief Safety layer of the haptic navigation. It decides when the HapticNavigationStrategy is active, and limits
 *        the force sent to the device. It does not use ROS.
 *
 * @details The strategy is active only if:
 *  - the navigation is enabled;
 *  - the device data is valid: received, younger than data_timeout, finite, and with a positive motion scaling;
 *  - the clutch is pressed (deadman);
 *  - the clutch was released since the navigation was enabled or the data became valid, so that the robot does not
 *    start moving with the clutch already pressed.
 *
 * When it is not active, the user of this class sends a zero twist and a zero force.
 */
class HapticNavigationSafety
{
public:
    /// State of the navigation and of the device in this cycle.
    struct Input
    {
        bool enabled{false};         ///< The navigation is enabled.
        bool data_received{false};   ///< At least one message of the device was received.
        double data_age{0.0};        ///< Time since the last message of the device (s).
        bool data_finite{false};     ///< The position and the angle of the stylus are finite.
        double motion_scaling{0.0};  ///< Motion scaling of the device.
        bool clutch{false};          ///< The clutch of the device is pressed.
    };

    /// Decision of update().
    struct Output
    {
        bool device_data_valid{false};            ///< The device data is valid.
        bool clutch{false};                       ///< The clutch is pressed (false whenever the data is not valid).
        bool waiting_for_clutch_release{false};   ///< Enabled with valid data, but the clutch must be released first.
        bool active{false};                       ///< The strategy commands the robot and renders the force.
    };

private:
    HapticNavigationSafetyConfiguration configuration_;   ///< Parameters of the safety layer.
    bool clutch_released_since_last_stop_{false};         ///< The clutch was released since the last stop.

public:
    HapticNavigationSafety() = delete;

    /**
     * @brief Creates the safety layer.
     * @param configuration The parameters of the safety layer.
     * @throws std::invalid_argument if the configuration is not valid.
     */
    explicit HapticNavigationSafety(const HapticNavigationSafetyConfiguration& configuration);

    /**
     * @brief Decides if the strategy is active in this cycle. Call it once per cycle.
     * @param input The state of the navigation and of the device.
     * @return The validity of the data, the clutch, and whether the strategy is active.
     */
    Output update(const Input& input);

    /**
     * @brief Limits the norm of a force to max_force, keeping its direction.
     * @param force The force (N, pure quaternion).
     * @return The limited force, or zero if the force is not a finite pure quaternion.
     */
    DQ_robotics::DQ limit_force(const DQ_robotics::DQ& force) const;

    /// @brief Returns the configuration of the safety layer.
    /// @return The configuration.
    const HapticNavigationSafetyConfiguration& get_configuration() const;
};

}
