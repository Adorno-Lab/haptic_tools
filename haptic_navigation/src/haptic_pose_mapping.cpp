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
 * @file haptic_pose_mapping.cpp
 * @brief Implementation of HapticPoseMapping.
 */

#include <haptic_navigation/haptic_pose_mapping.hpp>
#include <haptic_navigation/dq_validation.hpp>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace DQ_robotics;

namespace haptic_tools
{

namespace
{

/// @brief to_unit_dual_quaternion() with the name of the pose in the error message.
DQ _unit(const DQ& x, const std::string& name)
{
    return to_unit_dual_quaternion(x, "HapticPoseMapping: " + name);
}

}

HapticPoseMapping::HapticPoseMapping(const double& max_translation, const DQ& camera_to_device_rotation):
    max_translation_(max_translation),
    camera_to_device_rotation_(to_unit_quaternion(camera_to_device_rotation,
                                                  "HapticPoseMapping: camera_to_device_rotation"))
{
    if (!std::isfinite(max_translation_))
        throw std::invalid_argument("HapticPoseMapping: max_translation must be finite.");
}

DQ HapticPoseMapping::coppeliasim_camera_to_device_rotation()
{
    // cos(phi/2) + j_*sin(phi/2), phi = pi: 180 deg about y.
    return cos(M_PI/2.0) + j_*sin(M_PI/2.0);
}

DQ HapticPoseMapping::get_camera_to_device_rotation() const
{
    return camera_to_device_rotation_;
}

void HapticPoseMapping::engage(const DQ& device_pose, const DQ& end_effector_pose, const DQ& camera_pose,
                               const double& motion_scaling)
{
    if (!std::isfinite(motion_scaling) || motion_scaling <= 0.0)
        throw std::invalid_argument("HapticPoseMapping: the motion scaling must be positive, but it is "
                                    + std::to_string(motion_scaling) + ".");
    x_h1_ = _unit(device_pose, "the pose of the device");
    x_e1_ = _unit(end_effector_pose, "the pose of the end effector");
    const DQ r_ref = normalize(_unit(camera_pose, "the pose of the camera").rotation()*camera_to_device_rotation_);
    const DQ t_e1 = x_e1_.translation();
    x_ref_ = r_ref + 0.5*E_*t_e1*r_ref;
    motion_scaling_ = motion_scaling;
    engaged_ = true;
}

void HapticPoseMapping::disengage()
{
    engaged_ = false;
}

bool HapticPoseMapping::is_engaged() const
{
    return engaged_;
}

DQ HapticPoseMapping::desired_pose(const DQ& device_pose) const
{
    if (!engaged_)
        throw std::runtime_error("HapticPoseMapping::desired_pose: not engaged (call engage() at the press of the clutch).");
    const DQ x_h = _unit(device_pose, "the pose of the device");

    // The increments since the press, in the device frame.
    const DQ r_delta = normalize(x_h.rotation()*x_h1_.rotation().conj());
    DQ p_delta = motion_scaling_*(x_h.translation() - x_h1_.translation());
    const double norm = p_delta.vec3().norm();
    if (max_translation_ > 0.0 && norm > max_translation_)
        p_delta = (max_translation_/norm)*p_delta;
    const DQ x_delta = r_delta + 0.5*E_*p_delta*r_delta;

    return normalize(x_ref_*x_delta*x_ref_.conj()*x_e1_);
}

}
