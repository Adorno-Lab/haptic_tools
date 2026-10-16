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
 * @file navigation_view.hpp
 * @brief A widget that draws the status of the haptic navigation (used by haptic_navigation_example_gui).
 */

#pragma once
#include <optional>
#include <QWidget>
#include <haptic_tools_msgs/msg/haptic_navigation_status.hpp>

namespace haptic_tools
{

/**
 * @brief Draws haptic_tools_msgs/HapticNavigationStatus, to see the current mode of the navigation.
 *
 * @details
 *  - A top view of the base (x forward, up; y to the left): the dead zone (a dashed circle of radius
 *    dead_zone_radius), and the stylus (a dot at stylus_offset). The dot is green in translation, orange in rotation,
 *    blue when the clutch is pressed and the base is stopped, and grey otherwise.
 *  - A bar of theta (gimbal_roll) from -pi to pi, with the rotation zones beyond +-rotation_threshold in orange.
 *  - A line of text with the mode.
 */
class NavigationView : public QWidget
{
private:
    std::optional<haptic_tools_msgs::msg::HapticNavigationStatus> status_;   ///< Last status; none: no data.

protected:
    /// @brief Draws the top view, the bar of theta, and the mode.
    /// @param event The paint event (not used: the whole widget is drawn).
    void paintEvent(QPaintEvent* event) override;

public:
    /// @brief Creates the widget, without a status.
    /// @param parent The parent widget.
    explicit NavigationView(QWidget* parent = nullptr);

    /// @brief Shows a status.
    /// @param status The status of the navigation.
    void set_status(const haptic_tools_msgs::msg::HapticNavigationStatus& status);

    /// @brief Shows that there is no status (e.g. the example node is not running).
    void clear();
};

}
