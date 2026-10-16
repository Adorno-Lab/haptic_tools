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
 * @file main.cpp
 * @brief haptic_simulator: reads the parameters and shows the HapticSimulatorWindow.
 */

#include <atomic>
#include <cmath>
#include <signal.h>
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "haptic_simulator_window.hpp"

/// Set by the signal handler: closes the window.
static std::atomic_bool quit_requested(false);
/// @brief Signal handler (SIGINT, SIGTERM): requests to close the window.
void sig_handler(int)
{
    quit_requested = true;
}

/**
 * @brief Entry point of haptic_simulator.
 *
 * @details Parameters (with defaults): label ("m0_0"), publish_rate (250.0 Hz), translation_range (0.25 m),
 * joint_range (pi rad), motion_scaling (1.0), force_compliance (0.05 m/(N s)).
 *
 * @param argc Number of arguments.
 * @param argv Arguments (ROS 2 and Qt arguments, and parameters).
 * @return The exit code of the application.
 */
int main(int argc, char** argv)
{
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("haptic_simulator");

    haptic_tools::HapticSimulatorConfiguration configuration;
    configuration.label = node->declare_parameter<std::string>("label", "m0_0");
    configuration.publish_rate = node->declare_parameter<double>("publish_rate", 250.0);
    configuration.translation_range = node->declare_parameter<double>("translation_range", 0.25);
    configuration.joint_range = node->declare_parameter<double>("joint_range", M_PI);
    configuration.motion_scaling = node->declare_parameter<double>("motion_scaling", 1.0);
    configuration.force_compliance = node->declare_parameter<double>("force_compliance", 0.05);

    int exit_code = 0;
    {
        QApplication application(argc, argv);
        haptic_tools::HapticSimulatorWindow window(node, configuration, quit_requested);
        window.show();
        exit_code = application.exec();
    }
    rclcpp::shutdown();
    return exit_code;
}
