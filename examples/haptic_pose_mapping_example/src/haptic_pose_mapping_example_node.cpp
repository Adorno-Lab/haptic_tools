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
 * @file haptic_pose_mapping_example_node.cpp
 * @brief haptic_pose_mapping_example_node: reads the parameters and runs HapticPoseMappingExample.
 */

#include <stdexcept>
#include <string>
#include <vector>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include "haptic_pose_mapping_example.hpp"

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
 * @brief Entry point of haptic_pose_mapping_example_node.
 *
 * @details Parameters (all mandatory; see config/haptic_pose_mapping_example.yaml): interface_name, x_prefix,
 * xd_prefix, camera_prefix (string), thread_sampling_time_sec, motion_scaling, max_translation, data_timeout
 * (double), and camera_to_device_rotation (unit quaternion [w, x, y, z]).
 *
 * @param argc Number of arguments.
 * @param argv Arguments (ROS 2 arguments and parameters).
 * @return 0, or 1 after an exception.
 */
int main(int argc, char** argv)
{
    if (signal(SIGINT, sig_handler) == SIG_ERR || signal(SIGTERM, sig_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal handlers.");
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("haptic_pose_mapping_example_node");

    int exit_code = 0;
    try
    {
        haptic_tools::HapticPoseMappingExampleConfiguration configuration;
        sas::get_ros_parameter(node, "interface_name", configuration.interface_name);
        sas::get_ros_parameter(node, "x_prefix", configuration.x_prefix);
        sas::get_ros_parameter(node, "xd_prefix", configuration.xd_prefix);
        sas::get_ros_parameter(node, "camera_prefix", configuration.camera_prefix);
        sas::get_ros_parameter(node, "thread_sampling_time_sec", configuration.thread_sampling_time_sec);
        sas::get_ros_parameter(node, "motion_scaling", configuration.motion_scaling);
        sas::get_ros_parameter(node, "max_translation", configuration.max_translation);
        std::vector<double> r;
        sas::get_ros_parameter(node, "camera_to_device_rotation", r);
        if (r.size() != 4)
            throw std::runtime_error("camera_to_device_rotation must be a quaternion [w, x, y, z].");
        configuration.camera_to_device_rotation = DQ_robotics::DQ(r.at(0), r.at(1), r.at(2), r.at(3));
        sas::get_ros_parameter(node, "data_timeout", configuration.data_timeout);

        haptic_tools::HapticPoseMappingExample example(node, configuration, shutdown_signaler);
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
