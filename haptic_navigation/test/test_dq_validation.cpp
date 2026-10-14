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
 * @file test_dq_validation.cpp
 * @brief Tests of the checks of dq_validation.hpp.
 */

#include <cmath>
#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>
#include <haptic_navigation/dq_validation.hpp>

using namespace haptic_tools;
using namespace DQ_robotics;

namespace
{

const double kNaN = std::numeric_limits<double>::quiet_NaN();

/// @brief A pose with an arbitrary rotation and translation (unit up to machine precision).
DQ pose()
{
    const DQ r = normalize(cos(0.3) + sin(0.3)*normalize(i_ + 2.0*j_ - k_));
    return r + 0.5*E_*(0.4*i_ - 1.2*j_ + 0.9*k_)*r;
}

/// @brief x with each coefficient rounded to single precision (e.g. data sent as float).
DQ to_float(const DQ& x)
{
    Eigen::VectorXd v = x.vec8();
    for (int i = 0; i < 8; i++)
        v(i) = static_cast<float>(v(i));
    return DQ(v);
}

}

TEST(DQValidation, RoundingErrorsAreAcceptedAndNormalized)
{
    // These fail is_unit() of DQ Robotics (DQ_threshold = 1e-12). After the check, they pass it.
    const DQ yaml_8_decimals(0.0, 0.70710678, -0.70710678, 0.0);
    const DQ printed_6_decimals(0.707107, 0.707107, 0.0, 0.0);
    EXPECT_FALSE(is_unit(yaml_8_decimals));
    EXPECT_FALSE(is_unit(printed_6_decimals));
    EXPECT_TRUE(is_unit(to_unit_quaternion(yaml_8_decimals, "r")));
    EXPECT_TRUE(is_unit(to_unit_quaternion(printed_6_decimals, "r")));

    const DQ float_pose = to_float(pose());
    EXPECT_FALSE(is_unit(float_pose));
    EXPECT_TRUE(is_unit(to_unit_dual_quaternion(float_pose, "x")));
    EXPECT_LT((to_unit_dual_quaternion(float_pose, "x") - pose()).vec8().norm(), 1e-6);

    const DQ axis = to_unit_pure_quaternion((1.0 + 1e-7)*j_, "n");
    EXPECT_TRUE(is_unit(axis));
    EXPECT_TRUE(is_pure_quaternion(axis));
}

TEST(DQValidation, MistakesAreRejected)
{
    // Rotations.
    EXPECT_THROW(to_unit_quaternion(DQ(0.0, 0.7, -0.7, 0.0), "r"), std::invalid_argument);   // |r| - 1 = 1e-2
    EXPECT_THROW(to_unit_quaternion(DQ(0), "r"), std::invalid_argument);
    EXPECT_THROW(to_unit_quaternion(DQ(kNaN), "r"), std::invalid_argument);
    EXPECT_THROW(to_unit_quaternion(pose(), "r"), std::invalid_argument);                    // a pose, not a rotation

    // Axes.
    EXPECT_THROW(to_unit_pure_quaternion(2.0*k_, "n"), std::invalid_argument);
    EXPECT_THROW(to_unit_pure_quaternion(normalize(1.0 + k_), "n"), std::invalid_argument);  // not pure
    EXPECT_THROW(to_unit_pure_quaternion(k_ + E_*i_, "n"), std::invalid_argument);           // not a quaternion
    EXPECT_THROW(to_unit_pure_quaternion(kNaN*k_, "n"), std::invalid_argument);

    // Poses.
    EXPECT_THROW(to_unit_dual_quaternion(2.0*pose(), "x"), std::invalid_argument);
    EXPECT_THROW(to_unit_dual_quaternion(DQ(0), "x"), std::invalid_argument);
    EXPECT_THROW(to_unit_dual_quaternion(pose() + E_*0.1, "x"), std::invalid_argument);      // dual part of norm(x) != 0
    EXPECT_THROW(to_unit_dual_quaternion(kNaN*pose(), "x"), std::invalid_argument);
}

TEST(DQValidation, TheMessageNamesTheValue)
{
    try
    {
        to_unit_quaternion(DQ(0), "HapticNavigationStrategy: device_to_command_frame_rotation");
        FAIL();
    }
    catch (const std::invalid_argument& e)
    {
        EXPECT_EQ(std::string(e.what()),
                  "HapticNavigationStrategy: device_to_command_frame_rotation must be a unit quaternion.");
    }
}

TEST(DQValidation, PureQuaternionsAreExact)
{
    EXPECT_TRUE(is_finite_pure_quaternion(0.1*i_ - 0.2*j_ + 0.3*k_));
    EXPECT_TRUE(is_finite_pure_quaternion(DQ(0)));
    EXPECT_FALSE(is_finite_pure_quaternion(1e-6 + i_));       // not pure
    EXPECT_FALSE(is_finite_pure_quaternion(i_ + E_*j_));      // not a quaternion
    // is_pure_quaternion() of DQ Robotics accepts NaN; this does not.
    EXPECT_TRUE(is_pure_quaternion(kNaN*i_));
    EXPECT_FALSE(is_finite_pure_quaternion(kNaN*i_));
}
