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
 * @file haptic_navigation.hpp
 * @brief ROS 2 wrapper of the haptic navigation (used by haptic_navigation_node).
 */

#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_msgs/msg/bool.hpp>
#include <sas_msgs/msg/information_from_master.hpp>
#include <sas_msgs/msg/information_to_master.hpp>
#include <haptic_tools_msgs/msg/haptic_navigation_status.hpp>
#include <haptic_tools_core/navigation/haptic_navigation_safety.hpp>
#include <haptic_tools_core/navigation/haptic_navigation_strategy.hpp>

namespace haptic_tools
{

/// @brief Configuration of the HapticNavigation node.
struct HapticNavigationConfiguration
{
    std::string interface_name;              ///< Label of the device in sas_operator_side_receiver, e.g. "m0_0".
    std::string topic_prefix;                ///< Prefix of the topics of the node, e.g. "haptic_navigation".
    double thread_sampling_time_sec;         ///< Period of the loop (s), e.g. 0.004.
    std::string twist_frame_id;              ///< frame_id of the twist: the command frame F_E, e.g. "pelvis".
    /// Index of theta in joint_values of the device; negative counts from the end (-1: the last one).
    int theta_index;
    bool start_enabled;                      ///< Initial value of enabled, before a message on \<prefix\>/set/enabled.
    HapticNavigationStrategyConfiguration strategy;   ///< Parameters of the strategy.
    HapticNavigationSafetyConfiguration safety;       ///< Parameters of the safety layer.
};

/**
 * @brief Runs the HapticNavigationStrategy, behind the HapticNavigationSafety, with a haptic device of
 *        sas_operator_side_receiver (e.g. a Touch X, or haptic_simulator).
 *
 * @details Topics:
 * | Topic | Type | |
 * |---|---|---|
 * | /sas_operator_side_receiver/\<interface_name\>/get/information_from_master | sas_msgs/InformationFromMaster | subscribed |
 * | /sas_operator_side_receiver/\<interface_name\>/set/information_to_master | sas_msgs/InformationToMaster | the force, every cycle |
 * | \<topic_prefix\>/set/enabled | sas_msgs/Bool | subscribed |
 * | \<topic_prefix\>/get/target_twist | geometry_msgs/TwistStamped | every cycle, in F_E; zero when not active |
 * | \<topic_prefix\>/get/status | haptic_tools_msgs/HapticNavigationStatus | every cycle |
 *
 * It does not wait for the device: until its data is valid, the twist and the force are zero.
 */
class HapticNavigation
{
private:
    std::shared_ptr<rclcpp::Node> node_;                         ///< The ROS 2 node.
    HapticNavigationConfiguration configuration_;                ///< Parameters of the node.
    std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler_;   ///< Ends control_loop().
    sas::Clock clock_;                                           ///< Period of the loop.
    HapticNavigationStrategy strategy_;                          ///< The navigation strategy.
    HapticNavigationSafety safety_;                              ///< The safety layer.

    /// Device data (best effort).
    rclcpp::Subscription<sas_msgs::msg::InformationFromMaster>::SharedPtr subscriber_information_from_master_;
    sas_msgs::msg::InformationFromMaster information_from_master_;      ///< Last message of the device.
    bool information_from_master_received_{false};                      ///< At least one message was received.
    std::chrono::steady_clock::time_point information_from_master_time_;   ///< Arrival time of the last message.
    /// @brief Stores the last message of the device and its arrival time.
    /// @param msg The message of the device.
    void _callback_information_from_master(const sas_msgs::msg::InformationFromMaster& msg);

    rclcpp::Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_enabled_;   ///< Enables the navigation.
    bool enabled_{false};                                               ///< The navigation is enabled.
    /// @brief Enables or disables the navigation.
    /// @param msg True to enable it.
    void _callback_enabled(const sas_msgs::msg::Bool& msg);

    /// Force on the device.
    rclcpp::Publisher<sas_msgs::msg::InformationToMaster>::SharedPtr publisher_information_to_master_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_target_twist_;   ///< Twist of F_E.
    /// Status of the navigation.
    rclcpp::Publisher<haptic_tools_msgs::msg::HapticNavigationStatus>::SharedPtr publisher_status_;

    /**
     * @brief Reads theta from joint_values of the last message, at theta_index.
     * @param[out] theta The angle (rad).
     * @return False if theta_index is not in joint_values.
     */
    bool _get_theta(double& theta) const;

    /// @brief One cycle: safety layer, strategy, and the twist, the force, and the status.
    void _step();

    /**
     * @brief Publishes the twist and the force.
     * @param linear_velocity Linear velocity of F_E, in F_E (m/s, pure quaternion).
     * @param angular_velocity Angular velocity of F_E, in F_E (rad/s, pure quaternion).
     * @param force Force on the device, in the device frame (N, pure quaternion).
     */
    void _send(const DQ_robotics::DQ& linear_velocity, const DQ_robotics::DQ& angular_velocity,
               const DQ_robotics::DQ& force);

public:
    HapticNavigation() = delete;
    HapticNavigation(const HapticNavigation&) = delete;

    /**
     * @brief Creates the publishers and the subscribers.
     * @param node The ROS 2 node.
     * @param configuration The parameters of the node, the strategy, and the safety layer.
     * @param shutdown_signaler Signals the end of control_loop().
     * @throws std::invalid_argument if the configuration of the strategy or of the safety layer is not valid.
     */
    HapticNavigation(const std::shared_ptr<rclcpp::Node>& node,
                     const HapticNavigationConfiguration& configuration,
                     const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler);

    /**
     * @brief Runs a cycle every thread_sampling_time_sec until the shutdown is signaled, then sends a zero twist and
     *        a zero force.
     */
    void control_loop();
};

}
