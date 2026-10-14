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
 * @file haptic_navigation_node.cpp
 * @brief haptic_navigation_node: reads the parameters and runs HapticNavigation.
 */

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include "haptic_navigation.hpp"

#include <signal.h>
/// Signals the end of the control loop (SIGINT, SIGTERM).
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
/// @brief Signal handler: requests the shutdown of the control loop.
void sig_handler(int)
{
    shutdown_signaler->shutdown();
}

/**
 * @brief Reads a vector parameter [x, y, z] as a pure quaternion.
 * @param node The ROS 2 node.
 * @param name The name of the parameter.
 * @return The pure quaternion x*i_ + y*j_ + z*k_.
 * @throws std::runtime_error if the parameter is missing, or if it does not have 3 values.
 */
static DQ_robotics::DQ get_pure_quaternion_parameter(std::shared_ptr<rclcpp::Node>& node, const std::string& name)
{
    std::vector<double> v;
    sas::get_ros_parameter(node, name, v);
    if (v.size() != 3)
        throw std::runtime_error("get_pure_quaternion_parameter: " + name + " must be [x, y, z], it has "
                                 + std::to_string(v.size()) + " values.");
    return v.at(0)*DQ_robotics::i_ + v.at(1)*DQ_robotics::j_ + v.at(2)*DQ_robotics::k_;
}

/**
 * @brief Reads a quaternion parameter [w, x, y, z]. It is validated (unit) and normalized by its user.
 * @param node The ROS 2 node.
 * @param name The name of the parameter.
 * @return The quaternion w + x*i_ + y*j_ + z*k_.
 * @throws std::runtime_error if the parameter is missing, or if it does not have 4 values.
 */
static DQ_robotics::DQ get_rotation_parameter(std::shared_ptr<rclcpp::Node>& node, const std::string& name)
{
    std::vector<double> q;
    sas::get_ros_parameter(node, name, q);
    if (q.size() != 4)
        throw std::runtime_error("get_rotation_parameter: " + name + " must be a quaternion [w, x, y, z], it has "
                                 + std::to_string(q.size()) + " values.");
    return DQ_robotics::DQ(q.at(0), q.at(1), q.at(2), q.at(3));
}

/**
 * @brief Entry point of haptic_navigation_node.
 *
 * @details Parameters (all mandatory; see config/default.yaml):
 *   - interface_name (string): label of the device in sas_operator_side_receiver, e.g. "m0_0".
 *   - topic_prefix (string): prefix of the topics of the node, e.g. "haptic_navigation".
 *   - thread_sampling_time_sec (double): period of the loop, e.g. 0.004.
 *   - twist_frame_id (string): frame_id of the twist, the command frame F_E.
 *   - theta_index (int): index of theta in joint_values of the device; negative counts from the end (-1: the last).
 *   - start_enabled (bool): initial value of enabled, before a message on \<topic_prefix\>/set/enabled.
 *   - Strategy (see HapticNavigationStrategyConfiguration): planar_joint (bool), device_to_command_frame_rotation
 *     (unit quaternion [w, x, y, z]), dead_zone_radius, stiffness, rotation_threshold, yaw_rate (double), and
 *     rotation_axis ([x, y, z], read as a pure quaternion).
 *   - Safety (see HapticNavigationSafetyConfiguration): data_timeout, max_force (double).
 *
 * @param argc Number of arguments.
 * @param argv Arguments (ROS 2 arguments and parameters).
 * @return 0, or 1 after an exception.
 */
int main(int argc, char** argv)
{
    if (signal(SIGINT, sig_handler) == SIG_ERR || signal(SIGTERM, sig_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal handlers.");

    // The signals are handled here (not by rclcpp), so that the zero twist and force can still be published
    // when the loop ends.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("haptic_navigation_node");

    int exit_code = 0;
    try
    {
        haptic_tools::HapticNavigationConfiguration configuration;
        int64_t theta_index;

        sas::get_ros_parameter(node, "interface_name", configuration.interface_name);
        sas::get_ros_parameter(node, "topic_prefix", configuration.topic_prefix);
        sas::get_ros_parameter(node, "thread_sampling_time_sec", configuration.thread_sampling_time_sec);
        sas::get_ros_parameter(node, "twist_frame_id", configuration.twist_frame_id);
        sas::get_ros_parameter(node, "theta_index", theta_index);
        configuration.theta_index = static_cast<int>(theta_index);
        sas::get_ros_parameter(node, "start_enabled", configuration.start_enabled);

        auto& strategy = configuration.strategy;
        sas::get_ros_parameter(node, "planar_joint", strategy.planar_joint);
        strategy.device_to_command_frame_rotation = get_rotation_parameter(node, "device_to_command_frame_rotation");
        sas::get_ros_parameter(node, "dead_zone_radius", strategy.dead_zone_radius);
        sas::get_ros_parameter(node, "stiffness", strategy.stiffness);
        sas::get_ros_parameter(node, "rotation_threshold", strategy.rotation_threshold);
        sas::get_ros_parameter(node, "yaw_rate", strategy.yaw_rate);
        strategy.rotation_axis = get_pure_quaternion_parameter(node, "rotation_axis");

        auto& safety = configuration.safety;
        sas::get_ros_parameter(node, "data_timeout", safety.data_timeout);
        sas::get_ros_parameter(node, "max_force", safety.max_force);

        RCLCPP_INFO_STREAM(node->get_logger(), "::Parameters OK: interface_name " << configuration.interface_name
                           << ", topic_prefix " << configuration.topic_prefix
                           << ", thread_sampling_time_sec " << configuration.thread_sampling_time_sec
                           << ", twist_frame_id " << configuration.twist_frame_id
                           << ", theta_index " << configuration.theta_index
                           << ", start_enabled " << (configuration.start_enabled ? "true" : "false")
                           << ", planar_joint " << (strategy.planar_joint ? "true" : "false")
                           << ", dead_zone_radius " << strategy.dead_zone_radius
                           << ", stiffness " << strategy.stiffness
                           << ", rotation_threshold " << strategy.rotation_threshold
                           << ", yaw_rate " << strategy.yaw_rate
                           << ", device_to_command_frame_rotation " << strategy.device_to_command_frame_rotation
                           << ", rotation_axis " << strategy.rotation_axis
                           << ", data_timeout " << safety.data_timeout
                           << ", max_force " << safety.max_force);

        haptic_tools::HapticNavigation haptic_navigation(node, configuration, shutdown_signaler);
        haptic_navigation.control_loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), std::string("::Exception::") + e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
