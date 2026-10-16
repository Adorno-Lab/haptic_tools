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
 * @file haptic_navigation_example.cpp
 * @brief Implementation of HapticNavigationExample.
 */

#include "haptic_navigation_example.hpp"
#include <cmath>
#include <thread>

using namespace DQ_robotics;

namespace haptic_tools
{

namespace
{

/// @brief Converts a pure quaternion to a geometry_msgs/Vector3.
geometry_msgs::msg::Vector3 _to_vector3(const DQ& v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.q(1);
    msg.y = v.q(2);
    msg.z = v.q(3);
    return msg;
}

/// @brief Returns the name of a submode, for the log.
std::string _to_string(const HapticNavigationStrategy::SUBMODE& submode)
{
    switch (submode)
    {
    case HapticNavigationStrategy::SUBMODE::TRANSLATION: return "translation";
    case HapticNavigationStrategy::SUBMODE::ROTATION:    return "rotation";
    case HapticNavigationStrategy::SUBMODE::NONE:        break;
    }
    return "stopped";
}

}

HapticNavigationExample::HapticNavigationExample(const std::shared_ptr<rclcpp::Node>& node,
                                                 const HapticNavigationExampleConfiguration& configuration,
                                                 const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler):
    node_(node),
    configuration_(configuration),
    shutdown_signaler_(shutdown_signaler),
    clock_(configuration.thread_sampling_time_sec),
    strategy_(configuration.strategy),
    safety_(configuration.safety)
{
    const std::string device = "/sas_operator_side_receiver/" + configuration_.interface_name;
    // The device data arrives at a high rate: best effort, keep the last one.
    subscriber_device_ = node_->create_subscription<sas_msgs::msg::InformationFromMaster>(
        device + "/get/information_from_master", rclcpp::SensorDataQoS(),
        std::bind(&HapticNavigationExample::_callback_device, this, std::placeholders::_1));
    publisher_force_ = node_->create_publisher<sas_msgs::msg::InformationToMaster>(device + "/set/information_to_master", 1);
    publisher_twist_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(configuration_.twist_topic, 1);
    publisher_status_ = node_->create_publisher<haptic_tools_msgs::msg::HapticNavigationStatus>(configuration_.status_topic, 1);
}

void HapticNavigationExample::_callback_device(const sas_msgs::msg::InformationFromMaster& msg)
{
    device_ = msg;
    device_received_ = true;
    device_time_ = std::chrono::steady_clock::now();
}

void HapticNavigationExample::_step()
{
    const auto& p = device_.pose.position;
    const bool position_finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    // theta is the last joint of the stylus (the gimbal of the Touch X, or the last slider of haptic_simulator).
    const double theta = device_.joint_values.empty() ? 0.0 : device_.joint_values.back();

    HapticNavigationSafety::Input input;
    input.enabled = true;
    input.data_received = device_received_;
    input.data_age = device_received_
        ? std::chrono::duration<double>(std::chrono::steady_clock::now() - device_time_).count() : 0.0;
    input.data_finite = position_finite && !device_.joint_values.empty() && std::isfinite(theta);
    input.motion_scaling = device_.motion_scaling;
    input.clutch = device_.clutch;
    const HapticNavigationSafety::Output safety = safety_.update(input);

    // Zero twist and force when the strategy is not active; the stylus is still shown while the data is valid.
    HapticNavigationStrategy::Output output;
    if (safety.device_data_valid)
        output = strategy_.compute(p.x*i_ + p.y*j_ + p.z*k_, device_.motion_scaling, theta);
    if (!safety.active)
    {
        output.submode = HapticNavigationStrategy::SUBMODE::NONE;
        output.linear_velocity = DQ(0);
        output.angular_velocity = DQ(0);
        output.force = DQ(0);
    }
    output.force = safety_.limit_force(output.force);
    _send(output.linear_velocity, output.angular_velocity, output.force);

    using Status = haptic_tools_msgs::msg::HapticNavigationStatus;
    Status status;
    status.header.stamp = node_->now();
    status.header.frame_id = "trunk_respondable";
    status.submode = output.submode == HapticNavigationStrategy::SUBMODE::TRANSLATION ? Status::SUBMODE_TRANSLATION
                   : output.submode == HapticNavigationStrategy::SUBMODE::ROTATION ? Status::SUBMODE_ROTATION
                   : Status::SUBMODE_NONE;
    status.enabled = true;
    status.device_data_valid = safety.device_data_valid;
    status.clutch = safety.clutch;
    status.waiting_for_clutch_release = safety.waiting_for_clutch_release;
    status.active = safety.active;
    status.stylus_offset = _to_vector3(output.stylus_offset);
    // Without valid data, the radius before the motion scaling (to draw the dead zone).
    status.dead_zone_radius = safety.device_data_valid ? output.dead_zone_radius
                                                       : strategy_.get_configuration().dead_zone_radius;
    status.gimbal_roll = safety.device_data_valid ? theta : 0.0;
    status.rotation_threshold = strategy_.get_configuration().rotation_threshold;
    status.twist.linear = _to_vector3(output.linear_velocity);
    status.twist.angular = _to_vector3(output.angular_velocity);
    status.wrench.force = _to_vector3(output.force);
    publisher_status_->publish(status);

    std::string state = "moving: " + _to_string(output.submode);
    if (!safety.device_data_valid)
        state = "no valid data of the device /sas_operator_side_receiver/" + configuration_.interface_name;
    else if (safety.waiting_for_clutch_release)
        state = "release the clutch, and press it again";
    else if (!safety.clutch)
        state = "press the clutch to move the base";
    if (state != state_)
        RCLCPP_INFO_STREAM(node_->get_logger(), "::" << state);
    state_ = state;
}

void HapticNavigationExample::_send(const DQ& linear_velocity, const DQ& angular_velocity, const DQ& force)
{
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = node_->now();
    twist.header.frame_id = "trunk_respondable";
    twist.twist.linear.x = linear_velocity.q(1);
    twist.twist.linear.y = linear_velocity.q(2);
    twist.twist.angular.z = angular_velocity.q(3);
    publisher_twist_->publish(twist);

    sas_msgs::msg::InformationToMaster to_device;
    to_device.header.stamp = node_->now();
    to_device.wrench.force.x = force.q(1);
    to_device.wrench.force.y = force.q(2);
    to_device.wrench.force.z = force.q(3);
    publisher_force_->publish(to_device);
}

void HapticNavigationExample::control_loop()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "::Device /sas_operator_side_receiver/" << configuration_.interface_name
                       << ", base " << configuration_.twist_topic << ".");
    clock_.init();
    while (!shutdown_signaler_->should_shutdown())
    {
        rclcpp::spin_some(node_);
        _step();
        clock_.update_and_sleep();
    }
    // Zero twist and force before leaving, several times in case a message is lost.
    for (int n = 0; n < 5; n++)
    {
        _send(DQ(0), DQ(0), DQ(0));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "::Zero twist and force sent. Leaving.");
}

}
