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
 * @file test_haptic_navigation_safety.cpp
 * @brief Tests of HapticNavigationSafety.
 */

#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>
#include <haptic_tools_core/navigation/haptic_navigation_safety.hpp>

using haptic_tools::HapticNavigationSafety;
using haptic_tools::HapticNavigationSafetyConfiguration;

namespace
{

HapticNavigationSafety make_safety()
{
    HapticNavigationSafetyConfiguration c;
    c.data_timeout = 0.1;
    c.max_force = 1.5;
    return HapticNavigationSafety(c);
}

// Enabled, valid data, clutch as given.
HapticNavigationSafety::Input valid_input(const bool& clutch)
{
    HapticNavigationSafety::Input input;
    input.enabled = true;
    input.data_received = true;
    input.data_age = 0.01;
    input.data_finite = true;
    input.motion_scaling = 1.0;
    input.clutch = clutch;
    return input;
}

}

TEST(HapticNavigationSafety, ActiveAfterReleasingAndPressingTheClutch)
{
    auto safety = make_safety();
    EXPECT_FALSE(safety.update(valid_input(false)).active);
    const auto output = safety.update(valid_input(true));
    EXPECT_TRUE(output.active);
    EXPECT_TRUE(output.device_data_valid);
    EXPECT_TRUE(output.clutch);
    EXPECT_FALSE(output.waiting_for_clutch_release);
    // The clutch is a deadman.
    EXPECT_FALSE(safety.update(valid_input(false)).active);
    EXPECT_TRUE(safety.update(valid_input(true)).active);
}

TEST(HapticNavigationSafety, ClutchAlreadyPressedWhenEnabledMustBeReleasedFirst)
{
    auto safety = make_safety();
    auto input = valid_input(true);
    input.enabled = false;
    EXPECT_FALSE(safety.update(input).active);

    input.enabled = true;  // enabled with the clutch pressed
    auto output = safety.update(input);
    EXPECT_FALSE(output.active);
    EXPECT_TRUE(output.waiting_for_clutch_release);
    EXPECT_FALSE(safety.update(input).active);

    EXPECT_FALSE(safety.update(valid_input(false)).active);
    output = safety.update(valid_input(true));
    EXPECT_TRUE(output.active);
    EXPECT_FALSE(output.waiting_for_clutch_release);

    // Disabled and enabled again with the clutch still pressed: release it again.
    input.enabled = false;
    EXPECT_FALSE(safety.update(input).active);
    EXPECT_TRUE(safety.update(valid_input(true)).waiting_for_clutch_release);
}

TEST(HapticNavigationSafety, ClutchMustBeReleasedAfterTheDataComesBack)
{
    auto safety = make_safety();
    safety.update(valid_input(false));
    EXPECT_TRUE(safety.update(valid_input(true)).active);

    auto stale = valid_input(true);
    stale.data_age = 0.2;
    const auto output = safety.update(stale);
    EXPECT_FALSE(output.active);
    EXPECT_FALSE(output.device_data_valid);
    EXPECT_FALSE(output.clutch);

    EXPECT_FALSE(safety.update(valid_input(true)).active);
    safety.update(valid_input(false));
    EXPECT_TRUE(safety.update(valid_input(true)).active);
}

TEST(HapticNavigationSafety, InvalidData)
{
    auto safety = make_safety();
    safety.update(valid_input(false));
    ASSERT_TRUE(safety.update(valid_input(true)).active);

    auto input = valid_input(true);
    input.data_received = false;
    EXPECT_FALSE(make_safety().update(input).device_data_valid);

    input = valid_input(true);
    input.data_finite = false;
    EXPECT_FALSE(make_safety().update(input).device_data_valid);

    input = valid_input(true);
    input.motion_scaling = 0.0;
    EXPECT_FALSE(make_safety().update(input).device_data_valid);

    input = valid_input(true);
    input.motion_scaling = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(make_safety().update(input).device_data_valid);

    input = valid_input(true);
    input.data_age = 0.1;  // at the timeout: still valid
    EXPECT_TRUE(make_safety().update(input).device_data_valid);
}

TEST(HapticNavigationSafety, ForceLimit)
{
    const auto safety = make_safety();
    using namespace DQ_robotics;
    EXPECT_LT((safety.limit_force(0.3*i_ - 0.4*k_) - (0.3*i_ - 0.4*k_)).vec8().norm(), 1e-12);
    EXPECT_LT((safety.limit_force(3.0*i_ + 4.0*k_) - (0.9*i_ + 1.2*k_)).vec8().norm(), 1e-12);
    EXPECT_TRUE(safety.limit_force(std::numeric_limits<double>::infinity()*i_).vec8().isZero());
    EXPECT_TRUE(safety.limit_force(1.0 + i_).vec8().isZero());   // not a pure quaternion
}

TEST(HapticNavigationSafety, InvalidConfigurationThrows)
{
    HapticNavigationSafetyConfiguration c;
    c.data_timeout = 0.0;
    EXPECT_THROW(HapticNavigationSafety{c}, std::invalid_argument);
    c = HapticNavigationSafetyConfiguration{};
    c.max_force = -1.0;
    EXPECT_THROW(HapticNavigationSafety{c}, std::invalid_argument);
}
