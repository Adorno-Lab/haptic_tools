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
 * @file haptic_simulator_window.cpp
 * @brief Implementation of HapticSimulatorWindow.
 */

#include "haptic_simulator_window.hpp"
#include <algorithm>
#include <cmath>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QShortcut>
#include <QTimer>
#include <QVBoxLayout>

using namespace DQ_robotics;

namespace haptic_tools
{

namespace
{

/// @brief Returns the rotation of @p angle about @p axis (a unit pure quaternion, e.g. i_).
DQ _rotation(const DQ& axis, const double& angle)
{
    return cos(angle/2.0) + sin(angle/2.0)*axis;
}

/// Style of the checked toggle buttons.
const QString kToggleStyle = "QPushButton:checked { background-color: #28a046; color: white; font-weight: bold; }";

}

HapticSimulatorWindow::HapticSimulatorWindow(const std::shared_ptr<rclcpp::Node>& node,
                                             const HapticSimulatorConfiguration& configuration,
                                             const std::atomic_bool& quit_requested, QWidget* parent):
    QMainWindow(parent),
    node_(node),
    configuration_(configuration),
    quit_requested_(quit_requested)
{
    _build_ui();

    const std::string prefix = "/sas_operator_side_receiver/" + configuration_.label;
    publisher_information_from_master_ = node_->create_publisher<sas_msgs::msg::InformationFromMaster>(
        prefix + "/get/information_from_master", 1);
    subscriber_information_to_master_ = node_->create_subscription<sas_msgs::msg::InformationToMaster>(
        prefix + "/set/information_to_master", 1, [this](const sas_msgs::msg::InformationToMaster& msg) {
            force_ = msg.wrench.force.x*i_ + msg.wrench.force.y*j_ + msg.wrench.force.z*k_;
            force_time_ = std::chrono::steady_clock::now();
            force_received_ = true;
        });

    auto* timer = new QTimer(this);
    timer->setTimerType(Qt::PreciseTimer);
    connect(timer, &QTimer::timeout, this, &HapticSimulatorWindow::_publish);
    timer->start(std::max(1, static_cast<int>(std::lround(1000.0/configuration_.publish_rate))));

    auto* label_timer = new QTimer(this);
    connect(label_timer, &QTimer::timeout, this, &HapticSimulatorWindow::_update_labels);
    label_timer->start(100);
}

HapticSimulatorWindow::Axis HapticSimulatorWindow::_make_axis(const double& range, const int& decimals,
                                                              const QString& suffix)
{
    Axis axis;
    axis.slider = new QSlider(Qt::Horizontal, this);
    axis.slider->setRange(-kSliderSteps/2, kSliderSteps/2);
    axis.slider->setMinimumWidth(260);
    axis.spin_box = new QDoubleSpinBox(this);
    axis.spin_box->setRange(-range, range);
    axis.spin_box->setDecimals(decimals);
    axis.spin_box->setSingleStep(std::pow(10.0, -decimals + 1));
    axis.spin_box->setSuffix(suffix);
    axis.spin_box->setMinimumWidth(110);

    QSlider* slider = axis.slider;
    QDoubleSpinBox* spin_box = axis.spin_box;
    connect(slider, &QSlider::valueChanged, this, [slider, spin_box, range](int value) {
        const QSignalBlocker blocker(spin_box);
        spin_box->setValue(range*value/(kSliderSteps/2.0));
        (void)slider;
    });
    connect(spin_box, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [slider, range](double value) {
        const QSignalBlocker blocker(slider);
        slider->setValue(static_cast<int>(std::lround(value/range*(kSliderSteps/2.0))));
    });
    return axis;
}

double HapticSimulatorWindow::_value(const Axis& axis) const
{
    return axis.spin_box->value();
}

void HapticSimulatorWindow::_set_value(Axis& axis, const double& value)
{
    axis.spin_box->setValue(value);  // also moves the slider
}

void HapticSimulatorWindow::_build_ui()
{
    setWindowTitle(QString::fromStdString("Haptic simulator (" + configuration_.label + ")"));
    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);

    auto add_axes = [this](QGroupBox* box, std::array<Axis, 3>& axes, const std::array<QString, 3>& names,
                           const double& range, const int& decimals, const QString& suffix) {
        auto* grid = new QGridLayout(box);
        for (std::size_t i = 0; i < 3; i++)
        {
            axes.at(i) = _make_axis(range, decimals, suffix);
            grid->addWidget(new QLabel(names.at(i), box), static_cast<int>(i), 0);
            grid->addWidget(axes.at(i).slider, static_cast<int>(i), 1);
            grid->addWidget(axes.at(i).spin_box, static_cast<int>(i), 2);
        }
    };

    // The frame of the Touch X (the sticker on the device): x to the right of the operator, y up, z towards the operator.
    auto* position_box = new QGroupBox("Stylus position (frame of the Touch X)", central);
    add_axes(position_box, position_axes_, {"x  (+ right)", "y  (+ up)", "z  (+ towards the operator)"},
             configuration_.translation_range, 4, " m");
    layout->addWidget(position_box);

    auto* joint_box = new QGroupBox("Stylus joints (the last one is theta of haptic_navigation)", central);
    add_axes(joint_box, joint_axes_, {"joint 4", "joint 5", "joint 6"}, configuration_.joint_range, 3, " rad");
    layout->addWidget(joint_box);

    // Buttons. They do not take the focus, so that the space bar is the shortcut of the clutch.
    auto* buttons_box = new QGroupBox("Buttons", central);
    auto* buttons = new QHBoxLayout(buttons_box);
    auto make_toggle = [buttons_box](const QString& text) {
        auto* button = new QPushButton(text, buttons_box);
        button->setCheckable(true);
        button->setFocusPolicy(Qt::NoFocus);
        button->setStyleSheet(kToggleStyle);
        button->setMinimumHeight(36);
        return button;
    };
    clutch_button_ = make_toggle("Clutch (space)");
    QFont bold = clutch_button_->font();
    bold.setBold(true);
    clutch_button_->setFont(bold);
    button_ = make_toggle("Button");
    additional_keys_ = {make_toggle("Key 1"), make_toggle("Key 2")};
    buttons->addWidget(clutch_button_, 2);
    buttons->addWidget(button_, 1);
    buttons->addWidget(additional_keys_.at(0), 1);
    buttons->addWidget(additional_keys_.at(1), 1);
    layout->addWidget(buttons_box);
    auto* clutch_shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
    clutch_shortcut->setContext(Qt::WindowShortcut);
    connect(clutch_shortcut, &QShortcut::activated, clutch_button_, &QPushButton::toggle);

    // Parameters.
    auto* parameters_box = new QGroupBox("Parameters", central);
    auto* parameters = new QGridLayout(parameters_box);
    motion_scaling_ = new QDoubleSpinBox(parameters_box);
    motion_scaling_->setRange(0.0, 100.0);
    motion_scaling_->setDecimals(2);
    motion_scaling_->setSingleStep(0.5);
    motion_scaling_->setValue(configuration_.motion_scaling);
    gripper_ = new QDoubleSpinBox(parameters_box);
    gripper_->setRange(-1.0, 1.0);
    gripper_->setDecimals(2);
    gripper_->setSingleStep(0.05);
    parameters->addWidget(new QLabel("Motion scaling", parameters_box), 0, 0);
    parameters->addWidget(motion_scaling_, 0, 1);
    parameters->addWidget(new QLabel("Gripper", parameters_box), 0, 2);
    parameters->addWidget(gripper_, 0, 3);
    layout->addWidget(parameters_box);

    // Force feedback.
    auto* force_box = new QGroupBox("Force feedback (set/information_to_master)", central);
    auto* force_layout = new QVBoxLayout(force_box);
    force_label_ = new QLabel("No force received", force_box);
    apply_force_ = new QCheckBox(QString("Apply the force to the stylus (%1 m/(N s))")
                                     .arg(configuration_.force_compliance), force_box);
    apply_force_->setFocusPolicy(Qt::NoFocus);
    force_layout->addWidget(force_label_);
    force_layout->addWidget(apply_force_);
    layout->addWidget(force_box);

    auto* bottom = new QHBoxLayout();
    auto* centre = new QPushButton("Centre", central);
    centre->setFocusPolicy(Qt::NoFocus);
    connect(centre, &QPushButton::clicked, this, &HapticSimulatorWindow::_centre);
    status_label_ = new QLabel(central);
    bottom->addWidget(status_label_, 1);
    bottom->addWidget(centre);
    layout->addLayout(bottom);

    setCentralWidget(central);
}

DQ HapticSimulatorWindow::_position() const
{
    return _value(position_axes_.at(0))*i_ + _value(position_axes_.at(1))*j_ + _value(position_axes_.at(2))*k_;
}

void HapticSimulatorWindow::_set_position(const DQ& position)
{
    const double& range = configuration_.translation_range;
    for (std::size_t n = 0; n < 3; n++)
        _set_value(position_axes_.at(n), std::clamp(position.q(static_cast<Eigen::Index>(n) + 1), -range, range));
}

DQ HapticSimulatorWindow::_orientation() const
{
    return normalize(_rotation(i_, _value(joint_axes_.at(0)))*_rotation(j_, _value(joint_axes_.at(1)))
                     *_rotation(k_, _value(joint_axes_.at(2))));
}

void HapticSimulatorWindow::_centre()
{
    for (auto& axis : position_axes_)
        _set_value(axis, 0.0);
    for (auto& axis : joint_axes_)
        _set_value(axis, 0.0);
}

void HapticSimulatorWindow::_publish()
{
    if (quit_requested_)
    {
        close();
        return;
    }
    rclcpp::spin_some(node_);

    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - previous_publish_time_).count();
    previous_publish_time_ = now;

    // The hand follows the force received (only while it is recent).
    const bool force_fresh = force_received_
                             && std::chrono::duration<double>(now - force_time_).count() < 0.5;
    if (apply_force_->isChecked() && force_fresh && dt > 0.0 && dt < 0.5)
        _set_position(_position() + (configuration_.force_compliance*dt)*force_);

    sas_msgs::msg::InformationFromMaster msg;
    msg.header.stamp = node_->now();
    msg.label = configuration_.label;
    msg.sender_ip = "127.0.0.1";
    const DQ p = _position();
    msg.pose.position.x = p.q(1);
    msg.pose.position.y = p.q(2);
    msg.pose.position.z = p.q(3);
    const DQ r = _orientation();
    msg.pose.orientation.w = r.q(0);
    msg.pose.orientation.x = r.q(1);
    msg.pose.orientation.y = r.q(2);
    msg.pose.orientation.z = r.q(3);
    if (dt > 0.0 && dt < 0.5)
    {
        const DQ v = (1.0/dt)*(p - previous_position_);
        msg.twist.linear.x = v.q(1);
        msg.twist.linear.y = v.q(2);
        msg.twist.linear.z = v.q(3);
    }
    previous_position_ = p;
    msg.clutch = clutch_button_->isChecked();
    msg.additional_keys = {additional_keys_.at(0)->isChecked(), additional_keys_.at(1)->isChecked()};
    msg.motion_scaling = motion_scaling_->value();
    msg.button = button_->isChecked() ? 1.0 : 0.0;
    msg.gripper = gripper_->value();
    msg.joint_values = {0.0, 0.0, 0.0, _value(joint_axes_.at(0)), _value(joint_axes_.at(1)), _value(joint_axes_.at(2))};
    publisher_information_from_master_->publish(msg);
}

void HapticSimulatorWindow::_update_labels()
{
    const bool force_fresh = force_received_
                             && std::chrono::duration<double>(std::chrono::steady_clock::now() - force_time_).count() < 0.5;
    if (force_fresh)
        force_label_->setText(QString("Force: (%1, %2, %3) N   |F| = %4 N")
                                  .arg(force_.q(1), 0, 'f', 2).arg(force_.q(2), 0, 'f', 2).arg(force_.q(3), 0, 'f', 2)
                                  .arg(force_.vec3().norm(), 0, 'f', 2));
    else
        force_label_->setText("No force received");
    status_label_->setText(QString::fromStdString("/sas_operator_side_receiver/" + configuration_.label)
                           + QString("   %1 Hz").arg(configuration_.publish_rate, 0, 'f', 0));
}

}
