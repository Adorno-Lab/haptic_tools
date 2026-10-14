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
 * @file haptic_navigation.cpp
 * @brief Implementation of HapticNavigation.
 */

#include "haptic_navigation.hpp"
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

/// @brief Converts a submode of the strategy to the constant of HapticNavigationStatus.
uint8_t _to_status_submode(const HapticNavigationStrategy::SUBMODE& submode)
{
    using Status = haptic_tools_msgs::msg::HapticNavigationStatus;
    switch (submode)
    {
    case HapticNavigationStrategy::SUBMODE::TRANSLATION:
        return Status::SUBMODE_TRANSLATION;
    case HapticNavigationStrategy::SUBMODE::ROTATION:
        return Status::SUBMODE_ROTATION;
    case HapticNavigationStrategy::SUBMODE::NONE:
        break;
    }
    return Status::SUBMODE_NONE;
}

}

HapticNavigation::HapticNavigation(const std::shared_ptr<rclcpp::Node>& node,
                                   const HapticNavigationConfiguration& configuration,
                                   const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler):
    node_(node),
    configuration_(configuration),
    shutdown_signaler_(shutdown_signaler),
    clock_(configuration.thread_sampling_time_sec),
    strategy_(configuration.strategy),
    safety_(configuration.safety),
    enabled_(configuration.start_enabled)
{
    const std::string device_prefix = "/sas_operator_side_receiver/" + configuration_.interface_name;
    const std::string& p = configuration_.topic_prefix;

    // The device data arrives at a high rate: best effort, keep the last one.
    subscriber_information_from_master_ = node_->create_subscription<sas_msgs::msg::InformationFromMaster>(
        device_prefix + "/get/information_from_master", rclcpp::SensorDataQoS(),
        std::bind(&HapticNavigation::_callback_information_from_master, this, std::placeholders::_1));
    publisher_information_to_master_ = node_->create_publisher<sas_msgs::msg::InformationToMaster>(
        device_prefix + "/set/information_to_master", 1);

    subscriber_enabled_ = node_->create_subscription<sas_msgs::msg::Bool>(
        p + "/set/enabled", 1, std::bind(&HapticNavigation::_callback_enabled, this, std::placeholders::_1));
    publisher_target_twist_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(p + "/get/target_twist", 1);
    publisher_status_ = node_->create_publisher<haptic_tools_msgs::msg::HapticNavigationStatus>(p + "/get/status", 1);
}

void HapticNavigation::_callback_information_from_master(const sas_msgs::msg::InformationFromMaster& msg)
{
    // The age of the data is measured from its arrival, so that it does not depend on the clock of the sender.
    information_from_master_ = msg;
    information_from_master_received_ = true;
    information_from_master_time_ = std::chrono::steady_clock::now();
}

void HapticNavigation::_callback_enabled(const sas_msgs::msg::Bool& msg)
{
    if (msg.data != enabled_)
        RCLCPP_INFO_STREAM(node_->get_logger(), "::Navigation " << (msg.data ? "enabled." : "disabled."));
    enabled_ = msg.data;
}

bool HapticNavigation::_get_theta(double& theta) const
{
    const auto& joint_values = information_from_master_.joint_values;
    const int size = static_cast<int>(joint_values.size());
    const int index = configuration_.theta_index >= 0 ? configuration_.theta_index : size + configuration_.theta_index;
    if (index < 0 || index >= size)
        return false;
    theta = joint_values.at(static_cast<std::size_t>(index));
    return true;
}

void HapticNavigation::_step()
{
    // Snapshot of the last message of the device.
    const auto& msg = information_from_master_;
    const bool position_finite = std::isfinite(msg.pose.position.x) && std::isfinite(msg.pose.position.y)
                                 && std::isfinite(msg.pose.position.z);
    const DQ device_position = position_finite
        ? msg.pose.position.x*i_ + msg.pose.position.y*j_ + msg.pose.position.z*k_
        : DQ(0);
    double theta = 0.0;
    const bool theta_available = _get_theta(theta);
    if (information_from_master_received_ && !theta_available)
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                    "::theta_index " << configuration_.theta_index << " is not in joint_values (size "
                                    << msg.joint_values.size() << "). The device data is not valid.");

    // Safety layer.
    HapticNavigationSafety::Input safety_input;
    safety_input.enabled = enabled_;
    safety_input.data_received = information_from_master_received_;
    safety_input.data_age = information_from_master_received_
        ? std::chrono::duration<double>(std::chrono::steady_clock::now() - information_from_master_time_).count()
        : 0.0;
    safety_input.data_finite = theta_available && position_finite && std::isfinite(theta);
    safety_input.motion_scaling = msg.motion_scaling;
    safety_input.clutch = msg.clutch;
    const HapticNavigationSafety::Output safety_output = safety_.update(safety_input);

    // Strategy. When not active: zero twist and force.
    HapticNavigationStrategy::Output output;
    if (safety_output.device_data_valid)
    {
        output = strategy_.compute(device_position, msg.motion_scaling, theta);
        if (!safety_output.active)
        {
            output.submode = HapticNavigationStrategy::SUBMODE::NONE;
            output.linear_velocity = DQ(0);
            output.angular_velocity = DQ(0);
            output.force = DQ(0);
        }
    }
    output.force = safety_.limit_force(output.force);

    _send(output.linear_velocity, output.angular_velocity, output.force);

    haptic_tools_msgs::msg::HapticNavigationStatus status;
    status.header.stamp = node_->now();
    status.header.frame_id = configuration_.twist_frame_id;
    status.submode = _to_status_submode(output.submode);
    status.enabled = enabled_;
    status.device_data_valid = safety_output.device_data_valid;
    status.clutch = safety_output.clutch;
    status.waiting_for_clutch_release = safety_output.waiting_for_clutch_release;
    status.active = safety_output.active;
    status.stylus_offset = _to_vector3(output.stylus_offset);
    status.dead_zone_radius = output.dead_zone_radius;
    status.gimbal_roll = safety_output.device_data_valid ? theta : 0.0;
    status.rotation_threshold = strategy_.get_configuration().rotation_threshold;
    status.twist.linear = _to_vector3(output.linear_velocity);
    status.twist.angular = _to_vector3(output.angular_velocity);
    status.wrench.force = _to_vector3(output.force);
    publisher_status_->publish(status);
}

void HapticNavigation::_send(const DQ& linear_velocity, const DQ& angular_velocity, const DQ& force)
{
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = node_->now();
    twist.header.frame_id = configuration_.twist_frame_id;
    twist.twist.linear = _to_vector3(linear_velocity);
    twist.twist.angular = _to_vector3(angular_velocity);
    publisher_target_twist_->publish(twist);

    sas_msgs::msg::InformationToMaster to_master;
    to_master.header.stamp = node_->now();
    to_master.wrench.force = _to_vector3(force);
    publisher_information_to_master_->publish(to_master);
}

void HapticNavigation::control_loop()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "::Device /sas_operator_side_receiver/" << configuration_.interface_name
                       << ", topics " << configuration_.topic_prefix << ", "
                       << (enabled_ ? "enabled." : "disabled (enable it on " + configuration_.topic_prefix + "/set/enabled)."));
    clock_.init();
    bool data_valid_logged = false;
    while (!shutdown_signaler_->should_shutdown())
    {
        rclcpp::spin_some(node_);
        _step();
        if (information_from_master_received_ && !data_valid_logged)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "::Receiving data from the device.");
            data_valid_logged = true;
        }
        clock_.update_and_sleep();
    }

    // Zero twist and force before leaving, several times in case a message is lost.
    for (int i = 0; i < 5; i++)
    {
        _send(DQ(0), DQ(0), DQ(0));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "::Zero twist and force sent. Leaving.");
}

}
