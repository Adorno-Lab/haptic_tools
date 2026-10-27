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
 * @file haptic_pose_mapping_example.cpp
 * @brief Implementation of HapticPoseMappingExample.
 */

#include "haptic_pose_mapping_example.hpp"
#include <cmath>
#include <stdexcept>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>
#include <haptic_tools_core/dq_validation.hpp>

using namespace DQ_robotics;

namespace haptic_tools
{

HapticPoseMappingExample::HapticPoseMappingExample(const std::shared_ptr<rclcpp::Node>& node,
                                                   const HapticPoseMappingExampleConfiguration& configuration,
                                                   const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler):
    node_(node),
    configuration_(configuration),
    shutdown_signaler_(shutdown_signaler),
    clock_(configuration.thread_sampling_time_sec),
    mapping_(configuration.max_translation, configuration.camera_to_device_rotation)
{
    if (!(std::isfinite(configuration_.motion_scaling) && configuration_.motion_scaling > 0.0))
        throw std::invalid_argument("motion_scaling must be positive.");
    x_client_ = std::make_unique<sas::ObjectClient>(node_, configuration_.x_prefix);
    xd_client_ = std::make_unique<sas::ObjectClient>(node_, configuration_.xd_prefix);
    camera_client_ = std::make_unique<sas::ObjectClient>(node_, configuration_.camera_prefix);
    // The device data arrives at a high rate: best effort, keep the last one.
    subscriber_device_ = node_->create_subscription<sas_msgs::msg::InformationFromMaster>(
        "/sas_operator_side_receiver/" + configuration_.interface_name + "/get/information_from_master",
        rclcpp::SensorDataQoS(),
        std::bind(&HapticPoseMappingExample::_callback_device, this, std::placeholders::_1));
}

void HapticPoseMappingExample::_callback_device(const sas_msgs::msg::InformationFromMaster& msg)
{
    device_ = msg;
    device_received_ = true;
    device_time_ = std::chrono::steady_clock::now();
}

bool HapticPoseMappingExample::_device_pose(DQ& device_pose) const
{
    if (!device_received_ || std::chrono::duration<double>(std::chrono::steady_clock::now() - device_time_).count()
                                 > configuration_.data_timeout)
        return false;
    try
    {
        device_pose = to_unit_dual_quaternion(sas::geometry_msgs_pose_to_dq(device_.pose), "the pose of the device");
        return true;
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
}

void HapticPoseMappingExample::_log(const std::string& state)
{
    if (state != state_)
        RCLCPP_INFO_STREAM(node_->get_logger(), "::" << state);
    state_ = state;
}

void HapticPoseMappingExample::_step()
{
    if (!x_client_->is_enabled() || !camera_client_->is_enabled())
        return _log("waiting for the poses of x (" + configuration_.x_prefix + ") and of the camera ("
                    + configuration_.camera_prefix + "); is the simulation running?");
    // The end effector. With a robot, its pose from the forward kinematics.
    const DQ x = to_unit_dual_quaternion(x_client_->get_pose(), "the pose of x");

    DQ xd = x;
    DQ device_pose{1};
    const bool device_valid = _device_pose(device_pose);
    if (device_valid && device_.clutch)
    {
        // At the press, the poses are latched: the device, x, and the camera. So is the motion scaling: the one set in
        // the master (or haptic_simulator), or motion_scaling of the configuration if the master sends no valid one.
        if (!mapping_.is_engaged())
        {
            const bool from_master = std::isfinite(device_.motion_scaling) && device_.motion_scaling > 0.0;
            const double motion_scaling = from_master ? device_.motion_scaling : configuration_.motion_scaling;
            mapping_.engage(device_pose, x, camera_client_->get_pose(), motion_scaling);
            RCLCPP_INFO_STREAM(node_->get_logger(), "::Clutch pressed: motion scaling " << motion_scaling
                               << (from_master ? " (from the master)." : " (motion_scaling: the master sent "
                                   + std::to_string(device_.motion_scaling) + ")."));
        }
        xd = mapping_.desired_pose(device_pose);
        _log("xd follows the device (as seen from the camera)");
    }
    else
    {
        // Released (or no valid data): xd is x.
        mapping_.disengage();
        _log(device_valid ? "press the clutch to move xd"
                          : "no valid data of the device /sas_operator_side_receiver/" + configuration_.interface_name);
    }
    // With a robot, a kinematic controller moves the end effector towards xd.
    xd_client_->send_pose(xd);
}

void HapticPoseMappingExample::control_loop()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "::Device /sas_operator_side_receiver/" << configuration_.interface_name
                       << ", x " << configuration_.x_prefix << ", xd " << configuration_.xd_prefix
                       << ", camera " << configuration_.camera_prefix << ".");
    clock_.init();
    while (!shutdown_signaler_->should_shutdown())
    {
        rclcpp::spin_some(node_);
        _step();
        clock_.update_and_sleep();
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "::Leaving.");
}

}
