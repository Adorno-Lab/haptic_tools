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
 * @file haptic_navigation_example.hpp
 * @brief Example of the haptic navigation: a haptic device drives the holonomic base of scenes/haptic_scene.ttt
 *        (used by haptic_navigation_example_node).
 */

#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_msgs/msg/information_from_master.hpp>
#include <sas_msgs/msg/information_to_master.hpp>
#include <haptic_tools_msgs/msg/haptic_navigation_status.hpp>
#include <haptic_tools_core/navigation/haptic_navigation_safety.hpp>
#include <haptic_tools_core/navigation/haptic_navigation_strategy.hpp>

namespace haptic_tools
{

/// @brief Configuration of the HapticNavigationExample.
struct HapticNavigationExampleConfiguration
{
    std::string interface_name;          ///< Label of the device in sas_operator_side_receiver, e.g. "m0_0".
    std::string twist_topic;             ///< Topic of the base of the scene (geometry_msgs/TwistStamped).
    std::string status_topic;            ///< Status of the navigation, e.g. for haptic_navigation_example_gui.
    double thread_sampling_time_sec;     ///< Period of the loop (s), e.g. 0.004.
    HapticNavigationStrategyConfiguration strategy;   ///< Parameters of the strategy.
    HapticNavigationSafetyConfiguration safety;       ///< Parameters of the safety layer.
};

/**
 * @brief Drives a holonomic base with a haptic device: the navigation mode of Aldhafeeri et al. (TAROS 2025), with
 *        HapticNavigationStrategy and HapticNavigationSafety of haptic_tools_core.
 *
 * @details While the clutch is pressed, pushing the stylus out of the dead zone moves the base in that direction (in
 * the frame of the base), and turning the stylus inside the dead zone rotates it. The device renders a restoring force
 * towards the centre. For the complete node (enabled on a topic, with a status message), see haptic_navigation_node.
 *
 * Topics:
 * | Topic | Type | |
 * |---|---|---|
 * | /sas_operator_side_receiver/\<interface_name\>/get/information_from_master | sas_msgs/InformationFromMaster | subscribed |
 * | /sas_operator_side_receiver/\<interface_name\>/set/information_to_master | sas_msgs/InformationToMaster | the force, every cycle |
 * | \<twist_topic\> | geometry_msgs/TwistStamped | the twist of the base (vx, vy, wz, in the base frame), every cycle |
 * | \<status_topic\> | haptic_tools_msgs/HapticNavigationStatus | the dead zone, the stylus, theta, and the submode, every cycle |
 */
class HapticNavigationExample
{
private:
    std::shared_ptr<rclcpp::Node> node_;                         ///< The ROS 2 node.
    HapticNavigationExampleConfiguration configuration_;         ///< The parameters of the example.
    std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler_;   ///< Ends control_loop().
    sas::Clock clock_;                                           ///< Period of the loop.
    HapticNavigationStrategy strategy_;                          ///< The navigation strategy.
    HapticNavigationSafety safety_;                              ///< The safety layer.

    rclcpp::Subscription<sas_msgs::msg::InformationFromMaster>::SharedPtr subscriber_device_;   ///< The device.
    rclcpp::Publisher<sas_msgs::msg::InformationToMaster>::SharedPtr publisher_force_;          ///< The force.
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_twist_;            ///< The base.
    rclcpp::Publisher<haptic_tools_msgs::msg::HapticNavigationStatus>::SharedPtr publisher_status_;   ///< The status.
    sas_msgs::msg::InformationFromMaster device_;                 ///< Last message of the device.
    bool device_received_{false};                                 ///< At least one message was received.
    std::chrono::steady_clock::time_point device_time_;           ///< Arrival time of the last message.
    std::string state_;                                           ///< Last state written to the log.

    /// @brief Stores the last message of the device and its arrival time.
    /// @param msg The message of the device.
    void _callback_device(const sas_msgs::msg::InformationFromMaster& msg);

    /// @brief One cycle: the safety layer, the strategy, the twist, and the force.
    void _step();

    /**
     * @brief Publishes the twist of the base and the force on the device.
     * @param linear_velocity Linear velocity of the base, in the base frame (m/s, pure quaternion).
     * @param angular_velocity Angular velocity of the base, in the base frame (rad/s, pure quaternion).
     * @param force Force on the device, in the device frame (N, pure quaternion).
     */
    void _send(const DQ_robotics::DQ& linear_velocity, const DQ_robotics::DQ& angular_velocity,
               const DQ_robotics::DQ& force);

public:
    HapticNavigationExample() = delete;
    HapticNavigationExample(const HapticNavigationExample&) = delete;

    /**
     * @brief Creates the strategy, the safety layer, the subscriber of the device, and the publishers.
     * @param node The ROS 2 node.
     * @param configuration The parameters of the example.
     * @param shutdown_signaler Signals the end of control_loop().
     * @throws std::invalid_argument if the configuration of the strategy or of the safety layer is not valid.
     */
    HapticNavigationExample(const std::shared_ptr<rclcpp::Node>& node,
                            const HapticNavigationExampleConfiguration& configuration,
                            const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler);

    /**
     * @brief Runs a cycle every thread_sampling_time_sec until the shutdown is signaled, then sends a zero twist and
     *        a zero force.
     */
    void control_loop();
};

}
