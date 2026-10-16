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
 * @file haptic_simulator_window.hpp
 * @brief GUI that simulates a haptic device of sas_operator_side_receiver.
 */

#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <dqrobotics/DQ.h>
#include <rclcpp/rclcpp.hpp>
#include <sas_msgs/msg/information_from_master.hpp>
#include <sas_msgs/msg/information_to_master.hpp>

namespace haptic_tools
{

/// @brief Configuration of the HapticSimulatorWindow.
struct HapticSimulatorConfiguration
{
    std::string label;            ///< Label of the device in sas_operator_side_receiver, e.g. "m0_0".
    double publish_rate;          ///< Rate of the messages of the device (Hz), e.g. 250.
    double translation_range;     ///< The stylus moves in [-translation_range, translation_range] on each axis (m).
    double joint_range;           ///< The joints of the stylus move in [-joint_range, joint_range] (rad).
    double motion_scaling;        ///< Initial motion scaling.
    double force_compliance;      ///< With "Apply the force", the stylus moves force_compliance*force per second (m/(N s)).
};

/**
 * @brief Simulates a haptic device of sas_operator_side_receiver (e.g. the Touch X).
 *
 * @details It publishes sas_msgs/InformationFromMaster on /sas_operator_side_receiver/\<label\>/get/information_from_master
 * (at publish_rate), as the UDP receiver does with the real device, and shows the force received on
 * /sas_operator_side_receiver/\<label\>/set/information_to_master.
 *
 *  - Position: sliders x, y, z in the device frame (x right, y up, z towards the operator).
 *  - Joints: three sliders for the gimbal, published as joint_values = [0, 0, 0, j4, j5, j6]. The last one is theta
 *    of haptic_navigation. The orientation is rx(j4)*ry(j5)*rz(j6).
 *  - The clutch (also the space bar), the button, two additional keys, the motion scaling, and the gripper.
 *  - "Apply the force to the stylus": the stylus moves with the force received, as a hand that follows the device.
 */
class HapticSimulatorWindow : public QMainWindow
{
    Q_OBJECT
private:
    /// A slider and a spin box that show the same value.
    struct Axis
    {
        QSlider* slider;   ///< The slider.
        QDoubleSpinBox* spin_box;   ///< The spin box.
    };

    std::shared_ptr<rclcpp::Node> node_;   ///< The ROS 2 node.
    HapticSimulatorConfiguration configuration_;   ///< Parameters of the simulator.
    const std::atomic_bool& quit_requested_;   ///< Set by the signal handler: closes the window.

    rclcpp::Publisher<sas_msgs::msg::InformationFromMaster>::SharedPtr publisher_information_from_master_;   ///< State of the device.
    rclcpp::Subscription<sas_msgs::msg::InformationToMaster>::SharedPtr subscriber_information_to_master_;   ///< Force on the device.
    DQ_robotics::DQ force_{0};                                      ///< Last force received (N, pure quaternion, device frame).
    std::chrono::steady_clock::time_point force_time_;              ///< Arrival time of the last force.
    bool force_received_{false};   ///< At least one force was received.
    DQ_robotics::DQ previous_position_{0};                          ///< Last position published (m, pure quaternion), for the velocity.
    std::chrono::steady_clock::time_point previous_publish_time_{std::chrono::steady_clock::now()};   ///< To compute the velocity and the force motion.

    std::array<Axis, 3> position_axes_;   ///< Position of the stylus (x, y, z).
    std::array<Axis, 3> joint_axes_;   ///< Joints of the gimbal (4, 5, 6).
    QPushButton* clutch_button_;   ///< The clutch.
    QPushButton* button_;   ///< The button.
    std::array<QPushButton*, 2> additional_keys_;   ///< The two additional keys.
    QDoubleSpinBox* motion_scaling_;   ///< Motion scaling.
    QDoubleSpinBox* gripper_;   ///< Gripper value.
    QCheckBox* apply_force_;   ///< Moves the stylus with the force received.
    QLabel* force_label_;   ///< Shows the force received.
    QLabel* status_label_;   ///< Shows the topic and the rate.

    static constexpr int kSliderSteps = 2000;   ///< Resolution of the sliders.

    /**
     * @brief Creates a slider and a spin box in [-range, range], linked to each other.
     * @param range The limit of the values.
     * @param decimals The decimals of the spin box.
     * @param suffix The unit shown in the spin box.
     * @return The axis.
     */
    Axis _make_axis(const double& range, const int& decimals, const QString& suffix);
    /// @brief Returns the value of an axis.
    /// @param axis The axis.
    /// @return The value.
    double _value(const Axis& axis) const;
    /// @brief Sets the value of an axis (the spin box and the slider).
    /// @param axis The axis.
    /// @param value The value.
    void _set_value(Axis& axis, const double& value);
    /// @brief Builds the widgets of the window.
    void _build_ui();
    /// @brief Returns the position of the stylus.
    /// @return The position, in the device frame (m, pure quaternion).
    DQ_robotics::DQ _position() const;
    /// @brief Sets the position of the stylus, each axis limited to translation_range.
    /// @param position The position, in the device frame (m, pure quaternion).
    void _set_position(const DQ_robotics::DQ& position);
    /// @brief Returns the orientation of the stylus.
    /// @return rx(joint 4)*ry(joint 5)*rz(joint 6) (unit quaternion).
    DQ_robotics::DQ _orientation() const;
    /// @brief Moves the stylus and its joints back to zero.
    void _centre();
    /// @brief Publishes the state of the device (called at publish_rate).
    void _publish();
    /// @brief Updates the force and status labels.
    void _update_labels();

public:
    /**
     * @brief Creates the window, the publisher, and the subscriber.
     * @param node The ROS 2 node.
     * @param configuration The parameters of the simulator.
     * @param quit_requested When true, the window closes.
     * @param parent The parent widget.
     */
    HapticSimulatorWindow(const std::shared_ptr<rclcpp::Node>& node, const HapticSimulatorConfiguration& configuration,
                          const std::atomic_bool& quit_requested, QWidget* parent = nullptr);
};

}
