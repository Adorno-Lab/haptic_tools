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
 * @file dq_validation.hpp
 * @brief Checks of the unit quaternions, unit dual quaternions, and pure quaternions given to the haptic tools.
 */

#pragma once
#include <string>
#include <dqrobotics/DQ.h>

namespace haptic_tools
{

/**
 * @brief Tolerance of the unit norm of the rotations, axes, and poses given to the haptic tools.
 *
 * @details is_unit() of DQ Robotics compares with DQ_threshold (1e-12), which only values computed in double precision
 * meet. Values from outside (a YAML file, a ROS message, the device) have larger rounding errors, e.g. 2e-9 for a
 * quaternion written with 8 decimals, 2e-8 for single precision, and 3e-7 for 6 decimals. They are accepted within
 * this tolerance, and normalized: inside the haptic tools, they are unit up to machine precision. A larger error
 * (e.g. [0, 0.7, -0.7, 0], 2e-2) is a mistake, and it is rejected.
 */
constexpr double kUnitTolerance = 1e-3;

/**
 * @brief Checks if a DQ is finite.
 * @param x The DQ.
 * @return True if all its coefficients are finite (not NaN, not infinite).
 */
bool is_finite(const DQ_robotics::DQ& x);

/**
 * @brief Checks if a DQ is a finite pure quaternion, i.e. a vector x*i_ + y*j_ + z*k_.
 * @details For vectors built from their components (e.g. a position or a force), which are pure exactly: it uses
 *          is_pure_quaternion() of DQ Robotics (DQ_threshold), and also rejects NaN, which is_pure_quaternion()
 *          accepts.
 * @param v The DQ.
 * @return True if v is finite, and its real part and its dual part are zero.
 */
bool is_finite_pure_quaternion(const DQ_robotics::DQ& v);

/**
 * @brief Validates and normalizes a rotation.
 * @param r The rotation (unit quaternion, up to tolerance).
 * @param name The name of r, for the error message, e.g. "HapticNavigationStrategy: device_to_command_frame_rotation".
 * @param tolerance The tolerance of the norm, and of the dual part.
 * @return r, normalized.
 * @throws std::invalid_argument if r is not finite, its norm is not 1, or its dual part is not zero.
 */
DQ_robotics::DQ to_unit_quaternion(const DQ_robotics::DQ& r, const std::string& name,
                                   const double& tolerance = kUnitTolerance);

/**
 * @brief Validates and normalizes an axis.
 * @param n The axis (unit pure quaternion, up to tolerance).
 * @param name The name of n, for the error message.
 * @param tolerance The tolerance of the norm, of the real part, and of the dual part.
 * @return n, normalized.
 * @throws std::invalid_argument if n is not finite, its norm is not 1, or it is not pure.
 */
DQ_robotics::DQ to_unit_pure_quaternion(const DQ_robotics::DQ& n, const std::string& name,
                                        const double& tolerance = kUnitTolerance);

/**
 * @brief Validates and normalizes a pose.
 * @param x The pose (unit dual quaternion, up to tolerance).
 * @param name The name of x, for the error message.
 * @param tolerance The tolerance of the norm of the rotation, and of the dual part of norm(x).
 * @return x, normalized.
 * @throws std::invalid_argument if x is not finite, or norm(x) is not 1.
 */
DQ_robotics::DQ to_unit_dual_quaternion(const DQ_robotics::DQ& x, const std::string& name,
                                        const double& tolerance = kUnitTolerance);

}
