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
 * @file test_haptic_navigation_strategy.cpp
 * @brief Tests of HapticNavigationStrategy.
 */

// Tests of the navigation mode of Aldhafeeri et al. (TAROS 2025): Eq. (7) and (8), p_d^E of Eq. (9), and w^E of
// Eq. (11).

#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>
#include <haptic_tools_core/navigation/haptic_navigation_strategy.hpp>

using haptic_tools::HapticNavigationStrategy;
using haptic_tools::HapticNavigationStrategyConfiguration;
using SUBMODE = HapticNavigationStrategy::SUBMODE;
using namespace DQ_robotics;

namespace
{

constexpr double kTolerance = 1e-12;

/// @brief The pure quaternion x*i_ + y*j_ + z*k_.
DQ p(const double& x, const double& y, const double& z)
{
    return x*i_ + y*j_ + z*k_;
}

/// @brief Expects a == b, up to rounding.
void expect_equal(const DQ& a, const DQ& b)
{
    EXPECT_LT((a - b).vec8().norm(), 1e-9) << "a = " << a << ", b = " << b;
}

/// @brief Expects a == 0.
void expect_zero(const DQ& a)
{
    EXPECT_TRUE(a.vec8().isZero()) << "a = " << a;
}

// The end effector of the paper (planar_joint false): r = 0.04 m, k_r = 10 N/m, theta_limit = 1.2 rad,
// phi_dot_d = 0.2 rad/s, n_d^E = k_, r_H^E = 1.
HapticNavigationStrategyConfiguration spatial_configuration()
{
    HapticNavigationStrategyConfiguration c;
    c.planar_joint = false;
    c.dead_zone_radius = 0.04;
    c.stiffness = 10.0;
    c.rotation_threshold = 1.2;
    c.yaw_rate = 0.2;
    c.rotation_axis = k_;
    return c;
}

// A holonomic mobile base: the same, with planar_joint.
HapticNavigationStrategyConfiguration planar_configuration()
{
    auto c = spatial_configuration();
    c.planar_joint = true;
    return c;
}

}

TEST(HapticNavigationStrategy, Eq7RestoringForceWhileTheRobotMovesAndZeroWhenItStops)
{
    const HapticNavigationStrategy strategy(spatial_configuration());
    // The robot stops (NONE): zero force, also with the stylus away from the origin.
    const auto stopped = strategy.compute(p(0.03, 0.0, 0.01), 1.0, 0.0);
    EXPECT_EQ(stopped.submode, SUBMODE::NONE);
    expect_zero(stopped.force);
    // TRANSLATION, with s = 2: f = -k_r*(2*p).
    const auto translation = strategy.compute(p(0.05, 0.0, 0.0), 2.0, 0.0);
    EXPECT_EQ(translation.submode, SUBMODE::TRANSLATION);
    expect_equal(translation.force, p(-1.0, 0.0, 0.0));
    // ROTATION.
    const auto rotation = strategy.compute(p(0.0, 0.02, 0.0), 1.0, 1.5);
    EXPECT_EQ(rotation.submode, SUBMODE::ROTATION);
    expect_equal(rotation.force, p(0.0, -0.2, 0.0));
}

TEST(HapticNavigationStrategy, Eq8Submodes)
{
    const HapticNavigationStrategy strategy(spatial_configuration());
    EXPECT_EQ(strategy.compute(p(0.05, 0.0, 0.0), 1.0, 0.0).submode, SUBMODE::TRANSLATION);  // ||p|| > r
    EXPECT_EQ(strategy.compute(p(0.04, 0.0, 0.0), 1.0, 0.0).submode, SUBMODE::NONE);         // ||p|| = r
    EXPECT_EQ(strategy.compute(p(0.03, 0.0, 0.0), 1.0, 1.3).submode, SUBMODE::ROTATION);     // ||p|| <= r, |theta| > limit
    EXPECT_EQ(strategy.compute(p(0.03, 0.0, 0.0), 1.0, -1.3).submode, SUBMODE::ROTATION);
    EXPECT_EQ(strategy.compute(p(0.03, 0.0, 0.0), 1.0, 1.2).submode, SUBMODE::NONE);         // |theta| = limit
    EXPECT_EQ(strategy.compute(p(0.05, 0.0, 0.0), 1.0, 1.5).submode, SUBMODE::TRANSLATION);  // translation first

    // NONE: the robot stops completely.
    const auto stopped = strategy.compute(p(0.03, 0.01, 0.0), 1.0, 0.5);
    expect_zero(stopped.linear_velocity);
    expect_zero(stopped.angular_velocity);
}

TEST(HapticNavigationStrategy, Eq8MotionScalingScalesThePositionAndTheRadius)
{
    // ||s*p|| > s*r if and only if ||p|| > r: the dead zone does not change with s.
    const HapticNavigationStrategy strategy(spatial_configuration());
    EXPECT_EQ(strategy.compute(p(0.039, 0.0, 0.0), 5.0, 0.0).submode, SUBMODE::NONE);
    EXPECT_EQ(strategy.compute(p(0.041, 0.0, 0.0), 0.2, 0.0).submode, SUBMODE::TRANSLATION);
    EXPECT_NEAR(strategy.compute(p(0.0, 0.0, 0.0), 3.0, 0.0).dead_zone_radius, 0.12, kTolerance);
}

TEST(HapticNavigationStrategy, Eq9TranslationVelocity)
{
    const HapticNavigationStrategy strategy(spatial_configuration());

    // Straight: p_d^E = (||s*p|| - s*r)*p_bar.
    auto output = strategy.compute(p(0.05, 0.0, 0.0), 1.0, 0.0);
    expect_equal(output.linear_velocity, p(0.01, 0.0, 0.0));
    expect_zero(output.angular_velocity);

    expect_equal(strategy.compute(p(0.05, 0.0, 0.0), 10.0, 0.0).linear_velocity, p(0.1, 0.0, 0.0));
    expect_equal(strategy.compute(p(0.0, -0.06, 0.0), 10.0, 0.0).linear_velocity, p(0.0, -0.2, 0.0));

    // Diagonal: the direction of p.
    const double d = 0.06/std::sqrt(2.0);
    const double v = 0.2/std::sqrt(2.0);
    expect_equal(strategy.compute(p(d, d, 0.0), 10.0, 0.0).linear_velocity, p(v, v, 0.0));
}

TEST(HapticNavigationStrategy, Eq9VerticalVelocityOnlyWithoutPlanarJoint)
{
    const DQ position = p(0.03, 0.0, 0.03);  // ||p|| includes the vertical axis: outside the dead zone
    const double norm = position.vec3().norm();
    const double k = (norm - 0.04)/norm;

    // End effector: up/down is commanded.
    const HapticNavigationStrategy spatial(spatial_configuration());
    auto output = spatial.compute(position, 1.0, 0.0);
    EXPECT_EQ(output.submode, SUBMODE::TRANSLATION);
    expect_equal(output.linear_velocity, k*position);
    expect_equal(spatial.compute(p(0.0, 0.0, 0.05), 1.0, 0.0).linear_velocity, p(0.0, 0.0, 0.01));

    // Planar joint: the same sphere, but no vertical velocity.
    const HapticNavigationStrategy planar(planar_configuration());
    output = planar.compute(position, 1.0, 0.0);
    EXPECT_EQ(output.submode, SUBMODE::TRANSLATION);
    expect_equal(output.linear_velocity, p(k*0.03, 0.0, 0.0));
    output = planar.compute(p(0.0, 0.0, 0.05), 1.0, 0.0);  // straight up
    EXPECT_EQ(output.submode, SUBMODE::TRANSLATION);
    expect_zero(output.linear_velocity);
}

TEST(HapticNavigationStrategy, Eq9DeviceToCommandFrameRotation)
{
    // A device frame with x to the right, y up, and z towards the operator (e.g. OpenHaptics), and a command
    // frame with x forward, y left, z up: device x = -y, device y = z, device z = -x. As a unit quaternion,
    // r_H^E = 0.5*(1 + i_ - j_ - k_) (120 deg about (i_ - j_ - k_)/sqrt(3)).
    auto c = planar_configuration();
    c.device_to_command_frame_rotation = 0.5*(1.0 + i_ - j_ - k_);
    const HapticNavigationStrategy strategy(c);

    // Pushing away from the operator is forward.
    const auto forward = strategy.compute(p(0.0, 0.0, -0.05), 10.0, 0.0);
    expect_equal(forward.stylus_offset, p(0.5, 0.0, 0.0));
    expect_equal(forward.linear_velocity, p(0.1, 0.0, 0.0));
    // The force stays in the device frame.
    expect_equal(forward.force, p(0.0, 0.0, 5.0));

    // To the right of the operator is to the right of the robot (negative y).
    expect_equal(strategy.compute(p(0.05, 0.0, 0.0), 10.0, 0.0).linear_velocity, p(0.0, -0.1, 0.0));
}

TEST(HapticNavigationStrategy, Eq11YawCommand)
{
    // Planar joint: about k_, with the sign of theta, at a constant rate.
    const HapticNavigationStrategy planar(planar_configuration());
    expect_equal(planar.compute(p(0.0, 0.0, 0.0), 1.0, 1.3).angular_velocity, p(0.0, 0.0, 0.2));
    expect_equal(planar.compute(p(0.0, 0.0, 0.0), 1.0, 2.5).angular_velocity, p(0.0, 0.0, 0.2));
    expect_equal(planar.compute(p(0.0, 0.0, 0.0), 1.0, -1.3).angular_velocity, p(0.0, 0.0, -0.2));
    expect_zero(planar.compute(p(0.0, 0.0, 0.0), 1.0, 1.3).linear_velocity);

    // Planar joint about -k_: the opposite direction.
    auto c = planar_configuration();
    c.rotation_axis = -1.0*k_;
    c.yaw_rate = 0.3;
    expect_equal(HapticNavigationStrategy(c).compute(p(0.0, 0.0, 0.0), 1.0, 1.3).angular_velocity, p(0.0, 0.0, -0.3));

    // End effector about n_d^E = i_ (e.g. the optical axis of the camera, as in semi_automatic_strategy_teleop).
    c = spatial_configuration();
    c.rotation_axis = i_;
    expect_equal(HapticNavigationStrategy(c).compute(p(0.0, 0.0, 0.0), 1.0, -1.3).angular_velocity, p(-0.2, 0.0, 0.0));
}

TEST(HapticNavigationStrategy, InvalidArgumentsThrow)
{
    auto c = spatial_configuration();
    c.dead_zone_radius = 0.0;
    EXPECT_THROW(HapticNavigationStrategy{c}, std::invalid_argument);

    c = spatial_configuration();
    c.stiffness = -1.0;
    EXPECT_THROW(HapticNavigationStrategy{c}, std::invalid_argument);

    c = spatial_configuration();
    c.rotation_threshold = 0.0;
    EXPECT_THROW(HapticNavigationStrategy{c}, std::invalid_argument);

    c = spatial_configuration();
    c.rotation_axis = 2.0*k_;  // not unit
    EXPECT_THROW(HapticNavigationStrategy{c}, std::invalid_argument);

    c = spatial_configuration();
    c.rotation_axis = normalize(1.0 + k_);  // not pure
    EXPECT_THROW(HapticNavigationStrategy{c}, std::invalid_argument);

    c = planar_configuration();
    c.rotation_axis = i_;  // a planar joint only rotates about k_
    EXPECT_THROW(HapticNavigationStrategy{c}, std::invalid_argument);

    c = spatial_configuration();
    c.device_to_command_frame_rotation = DQ(2.0);  // not unit
    EXPECT_THROW(HapticNavigationStrategy{c}, std::invalid_argument);

    c = spatial_configuration();
    c.device_to_command_frame_rotation = 1.0 + 0.5*E_*i_;  // a unit dual quaternion, not a rotation
    EXPECT_THROW(HapticNavigationStrategy{c}, std::invalid_argument);

    // The position of the stylus must be a pure quaternion.
    const HapticNavigationStrategy strategy(spatial_configuration());
    EXPECT_THROW(strategy.compute(1.0 + p(0.05, 0.0, 0.0), 1.0, 0.0), std::invalid_argument);
}

TEST(HapticNavigationStrategy, ParametersWithRoundingErrorsAreNormalized)
{
    // e.g. a quaternion written with 8 decimals in a YAML file.
    auto c = spatial_configuration();
    c.device_to_command_frame_rotation = DQ(0.0, 0.70710678, -0.70710678, 0.0);
    c.rotation_axis = (1.0 + 1e-8)*i_;
    const HapticNavigationStrategy strategy(c);
    EXPECT_NEAR(strategy.get_configuration().device_to_command_frame_rotation.vec4().norm(), 1.0, kTolerance);
    EXPECT_NEAR(strategy.get_configuration().rotation_axis.vec3().norm(), 1.0, kTolerance);
}
