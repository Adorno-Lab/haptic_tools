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
 * @file haptic_navigation_example_node.cpp
 * @brief haptic_navigation_example_node: reads the parameters and runs HapticNavigationExample.
 */

#include <stdexcept>
#include <string>
#include <vector>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include "haptic_navigation_example.hpp"

using namespace DQ_robotics;

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
/// Signals the end of the control loop (SIGINT, SIGTERM).
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
/// @brief Signal handler: requests the shutdown of the control loop.
void sig_handler(int)
{
    shutdown_signaler->shutdown();
}

/**
 * @brief Reads a double array parameter of a given size.
 * @param node The ROS 2 node.
 * @param name The name of the parameter.
 * @param size The expected number of values.
 * @return The values.
 * @throws std::runtime_error if the parameter is missing, or if it does not have @p size values.
 */
static std::vector<double> get_array_parameter(std::shared_ptr<rclcpp::Node>& node, const std::string& name,
                                               const std::size_t& size)
{
    std::vector<double> v;
    sas::get_ros_parameter(node, name, v);
    if (v.size() != size)
        throw std::runtime_error(name + " must have " + std::to_string(size) + " values, it has "
                                 + std::to_string(v.size()) + ".");
    return v;
}

/**
 * @brief Entry point of haptic_navigation_example_node.
 *
 * @details Parameters (all mandatory; see config/haptic_navigation_example.yaml): interface_name, twist_topic,
 * status_topic (string), thread_sampling_time_sec, dead_zone_radius, stiffness, rotation_threshold, yaw_rate, data_timeout,
 * max_force (double), device_to_command_frame_rotation (unit quaternion [w, x, y, z]), and rotation_axis ([x, y, z]).
 *
 * @param argc Number of arguments.
 * @param argv Arguments (ROS 2 arguments and parameters).
 * @return 0, or 1 after an exception.
 */
int main(int argc, char** argv)
{
    if (signal(SIGINT, sig_handler) == SIG_ERR || signal(SIGTERM, sig_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal handlers.");
    // The signals are handled here (not by rclcpp), so that the zero twist and force can still be published.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("haptic_navigation_example_node");

    int exit_code = 0;
    try
    {
        haptic_tools::HapticNavigationExampleConfiguration configuration;
        sas::get_ros_parameter(node, "interface_name", configuration.interface_name);
        sas::get_ros_parameter(node, "twist_topic", configuration.twist_topic);
        sas::get_ros_parameter(node, "status_topic", configuration.status_topic);
        sas::get_ros_parameter(node, "thread_sampling_time_sec", configuration.thread_sampling_time_sec);

        // The base of the scene moves on the floor: only [vx, vy, wz].
        auto& strategy = configuration.strategy;
        strategy.planar_joint = true;
        const std::vector<double> r = get_array_parameter(node, "device_to_command_frame_rotation", 4);
        strategy.device_to_command_frame_rotation = DQ(r.at(0), r.at(1), r.at(2), r.at(3));
        const std::vector<double> n = get_array_parameter(node, "rotation_axis", 3);
        strategy.rotation_axis = n.at(0)*i_ + n.at(1)*j_ + n.at(2)*k_;
        sas::get_ros_parameter(node, "dead_zone_radius", strategy.dead_zone_radius);
        sas::get_ros_parameter(node, "stiffness", strategy.stiffness);
        sas::get_ros_parameter(node, "rotation_threshold", strategy.rotation_threshold);
        sas::get_ros_parameter(node, "yaw_rate", strategy.yaw_rate);
        sas::get_ros_parameter(node, "data_timeout", configuration.safety.data_timeout);
        sas::get_ros_parameter(node, "max_force", configuration.safety.max_force);

        haptic_tools::HapticNavigationExample example(node, configuration, shutdown_signaler);
        example.control_loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), std::string("::Exception::") + e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
