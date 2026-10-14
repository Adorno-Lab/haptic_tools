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
 * @file dq_validation.cpp
 * @brief Implementation of the checks of dq_validation.hpp.
 */

#include <haptic_navigation/dq_validation.hpp>
#include <cmath>
#include <stdexcept>

namespace haptic_tools
{

namespace
{

/// @brief Throws std::invalid_argument with "<name> must be <what>." if @p condition is false.
void _require(const bool& condition, const std::string& name, const std::string& what)
{
    if (!condition)
        throw std::invalid_argument(name + " must be " + what + ".");
}

}

bool is_finite(const DQ_robotics::DQ& x)
{
    return x.vec8().allFinite();
}

bool is_finite_pure_quaternion(const DQ_robotics::DQ& v)
{
    return is_finite(v) && DQ_robotics::is_pure_quaternion(v);
}

DQ_robotics::DQ to_unit_quaternion(const DQ_robotics::DQ& r, const std::string& name, const double& tolerance)
{
    _require(is_finite(r) && std::abs(r.P().vec4().norm() - 1.0) <= tolerance
             && r.D().vec4().norm() <= tolerance, name, "a unit quaternion");
    return DQ_robotics::normalize(r.P());
}

DQ_robotics::DQ to_unit_pure_quaternion(const DQ_robotics::DQ& n, const std::string& name, const double& tolerance)
{
    _require(is_finite(n) && std::abs(n.Im().vec4().norm() - 1.0) <= tolerance
             && std::abs(n.q(0)) <= tolerance && n.D().vec4().norm() <= tolerance, name, "a unit pure quaternion");
    return DQ_robotics::normalize(n.P().Im());
}

DQ_robotics::DQ to_unit_dual_quaternion(const DQ_robotics::DQ& x, const std::string& name, const double& tolerance)
{
    _require(is_finite(x) && std::abs(x.P().vec4().norm() - 1.0) <= tolerance
             && std::abs(DQ_robotics::norm(x).q(4)) <= tolerance, name, "a unit dual quaternion");
    return DQ_robotics::normalize(x);
}

}
