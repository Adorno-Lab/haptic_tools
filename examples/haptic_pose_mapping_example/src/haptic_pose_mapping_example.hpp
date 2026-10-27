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
 * @file haptic_pose_mapping_example.hpp
 * @brief Example of the pose mapping: a haptic device sets the pose of the frame xd of scenes/pose_mapping.ttt
 *        (used by haptic_pose_mapping_example_node).
 */

#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_object_client.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_msgs/msg/information_from_master.hpp>
#include <haptic_tools_core/pose_mapping/haptic_pose_mapping.hpp>

namespace haptic_tools
{

/// @brief Configuration of the HapticPoseMappingExample.
struct HapticPoseMappingExampleConfiguration
{
    std::string interface_name;          ///< Label of the device in sas_operator_side_receiver, e.g. "m0_0".
    std::string x_prefix;                ///< The end effector, e.g. "sas_robot_driver_coppeliasim/object/x".
    std::string xd_prefix;               ///< The desired pose, e.g. "sas_robot_driver_coppeliasim/object/xd".
    std::string camera_prefix;           ///< The reference camera, e.g. "sas_robot_driver_coppeliasim/object/DefaultCamera".
    double thread_sampling_time_sec;     ///< Period of the loop (s), e.g. 0.01.
    double motion_scaling;               ///< Scaling of the translation (positive) if the master sends no valid one.
    double max_translation;              ///< Maximum displacement of xd in a press of the clutch (m); zero: no limit.
    DQ_robotics::DQ camera_to_device_rotation;   ///< r_offset of HapticPoseMapping (unit quaternion): j_ for CoppeliaSim.
    double data_timeout;                 ///< The device data is valid while younger than this (s).
};

/**
 * @brief Sets the pose of xd with a haptic device and its clutch, with HapticPoseMapping of haptic_tools_core.
 *
 * @details The frames of the scene are published by their sas_object_script: x (the end effector), xd (the desired
 * pose), and the reference camera (e.g. DefaultCamera, the camera of the main view). While the clutch is pressed, the
 * motion of the device since the press moves xd, starting from the pose of x at the press: moving the stylus to the
 * right moves xd to the right in the image of the camera. When the clutch is released (or the device data is not
 * valid), xd is set to x. The example does not move x: with a robot, a kinematic controller moves the end effector
 * towards xd, and x is its pose (e.g. from the forward kinematics).
 */
class HapticPoseMappingExample
{
private:
    std::shared_ptr<rclcpp::Node> node_;                         ///< The ROS 2 node.
    HapticPoseMappingExampleConfiguration configuration_;        ///< The parameters of the example.
    std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler_;   ///< Ends control_loop().
    sas::Clock clock_;                                           ///< Period of the loop.
    HapticPoseMapping mapping_;                                  ///< The pose mapping.

    std::unique_ptr<sas::ObjectClient> x_client_;        ///< The end effector (read every cycle).
    std::unique_ptr<sas::ObjectClient> xd_client_;       ///< The desired pose (sent every cycle).
    std::unique_ptr<sas::ObjectClient> camera_client_;   ///< The reference camera (read at each press).
    rclcpp::Subscription<sas_msgs::msg::InformationFromMaster>::SharedPtr subscriber_device_;   ///< The device.
    sas_msgs::msg::InformationFromMaster device_;                 ///< Last message of the device.
    bool device_received_{false};                                 ///< At least one message was received.
    std::chrono::steady_clock::time_point device_time_;           ///< Arrival time of the last message.

    std::string state_;            ///< Last state written to the log.

    /// @brief Stores the last message of the device and its arrival time.
    /// @param msg The message of the device.
    void _callback_device(const sas_msgs::msg::InformationFromMaster& msg);

    /**
     * @brief Returns the pose of the device, if its data is valid.
     * @param[out] device_pose The pose of the stylus in the device frame (unit dual quaternion).
     * @return True if the data was received, is younger than data_timeout, and the pose is a unit dual quaternion.
     */
    bool _device_pose(DQ_robotics::DQ& device_pose) const;

    /// @brief Writes @p state to the log when it changes.
    /// @param state What the example is doing.
    void _log(const std::string& state);

    /// @brief One cycle: reads x, and sets xd with the clutch and the mapping.
    void _step();

public:
    HapticPoseMappingExample() = delete;
    HapticPoseMappingExample(const HapticPoseMappingExample&) = delete;

    /**
     * @brief Creates the mapping, the clients of the frames of the scene, and the subscriber of the device.
     * @param node The ROS 2 node.
     * @param configuration The parameters of the example.
     * @param shutdown_signaler Signals the end of control_loop().
     * @throws std::invalid_argument if a parameter is not valid.
     */
    HapticPoseMappingExample(const std::shared_ptr<rclcpp::Node>& node,
                             const HapticPoseMappingExampleConfiguration& configuration,
                             const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler);

    /// @brief Runs a cycle every thread_sampling_time_sec until the shutdown is signaled.
    void control_loop();
};

}
