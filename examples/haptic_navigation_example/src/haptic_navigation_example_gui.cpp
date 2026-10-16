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
 * @file haptic_navigation_example_gui.cpp
 * @brief haptic_navigation_example_gui: shows the status of haptic_navigation_example_node in a NavigationView.
 */

#include <atomic>
#include <chrono>
#include <signal.h>
#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "navigation_view.hpp"

/// Set by the signal handler: closes the window.
static std::atomic_bool quit_requested{false};
/// @brief Signal handler (SIGINT, SIGTERM): requests to close the window.
void sig_handler(int)
{
    quit_requested = true;
}

/**
 * @brief Entry point of haptic_navigation_example_gui.
 *
 * @details Parameters (with defaults): status_topic ("/haptic_navigation_example/get/status"). The view shows "no
 * status" when the last status is older than 0.5 s.
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
    auto node = std::make_shared<rclcpp::Node>("haptic_navigation_example_gui");
    const std::string status_topic = node->declare_parameter<std::string>("status_topic",
                                                                          "/haptic_navigation_example/get/status");
    int exit_code = 0;
    {
        QApplication application(argc, argv);
        haptic_tools::NavigationView view;
        view.setWindowTitle("Haptic navigation example");
        view.show();

        std::chrono::steady_clock::time_point last_status{};
        bool received = false;
        auto subscriber = node->create_subscription<haptic_tools_msgs::msg::HapticNavigationStatus>(
            status_topic, 1, [&](const haptic_tools_msgs::msg::HapticNavigationStatus& msg) {
                view.set_status(msg);
                last_status = std::chrono::steady_clock::now();
                received = true;
            });

        // The ROS callbacks run in the event loop of Qt, every 10 ms (the window is the GUI thread).
        QTimer timer;
        QObject::connect(&timer, &QTimer::timeout, [&]() {
            if (quit_requested)
            {
                view.close();
                return;
            }
            rclcpp::spin_some(node);
            if (received && std::chrono::duration<double>(std::chrono::steady_clock::now() - last_status).count() > 0.5)
            {
                view.clear();
                received = false;
            }
        });
        timer.start(10);
        exit_code = application.exec();
    }
    rclcpp::shutdown();
    return exit_code;
}
