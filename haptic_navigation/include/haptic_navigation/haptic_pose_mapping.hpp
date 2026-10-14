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
 * @file haptic_pose_mapping.hpp
 * @brief Desired pose of an end effector set with the pose of a haptic device and its clutch.
 */

#pragma once
#include <dqrobotics/DQ.h>

namespace haptic_tools
{

/**
 * @brief Sets the desired pose of an end effector with the pose of a haptic device, as seen from a reference
 *        camera. It does not use ROS.
 *
 * @details While the clutch is pressed, the motion of the device since the press moves the desired pose: moving the
 * stylus to the right moves the end effector to the right in the image of the camera. The user handles the clutch:
 * call engage() at the press, desired_pose() while it is pressed, and disengage() at the release.
 *
 * The pose of the device is the pose of the stylus of the haptic device (e.g. the Touch X), expressed in the base
 * frame of the device F_H, as reported by the device: e.g. the pose of sas_msgs/InformationFromMaster, from
 * sas_operator_side_receiver or haptic_simulator. It is not scaled (the motion scaling s is applied here), and it is
 * not expressed in the frame of the robot or of the world. F_H has x to the right, y up, and z towards the user. A
 * CoppeliaSim camera looks along +z with +y up, so r_offset (180 deg about y) gives it the axes of F_H.
 *
 * At the press, with x_h1 the pose of the device, x_e1 the pose of the end effector, r_c1 the rotation of the camera,
 * and s the motion scaling:
 * @verbatim
   r_ref = r_c1*r_offset,   x_ref = r_ref + 0.5*E*t_e1*r_ref
   @endverbatim
 * While pressed, with x_h the pose of the device:
 * @verbatim
   r_delta = r_h*r_h1^*,   p_delta = s*(p_h - p_h1),   x_delta = r_delta + 0.5*E*p_delta*r_delta
   x_d     = x_ref*x_delta*x_ref^* * x_e1
   @endverbatim
 * that is, t_d = t_e1 + Ad(r_ref)*p_delta and r_d = (r_ref*r_delta*r_ref^*)*r_e1.
 *
 * Everything is latched at the press, so this is position control: holding the stylus still holds x_d still, and
 * moving the camera or changing the scaling has no effect until the next press. Optionally, ||p_delta|| is limited
 * to max_translation.
 */
class HapticPoseMapping
{
private:
    bool engaged_{false};          ///< Between engage() and disengage().
    DQ_robotics::DQ x_h1_{1};      ///< Pose of the stylus, in the device frame F_H, at the press.
    DQ_robotics::DQ x_e1_{1};      ///< Pose of the end effector at the press.
    DQ_robotics::DQ x_ref_{1};     ///< Frame with the axes of the camera, at the end effector, at the press.
    double motion_scaling_{1.0};   ///< Motion scaling s, at the press.
    double max_translation_{0.0};  ///< Maximum ||p_delta|| (m); non-positive: no limit.

public:
    /**
     * @brief Creates the mapping. It starts not engaged: call engage() before desired_pose().
     * @param max_translation Safety limit (m) on how far x_d moves from the end effector position at the press. The
     *        scaled displacement of the stylus, p_delta = s*(p_h - p_h1), is cut to this length, keeping its
     *        direction (e.g. 0.5: with s = 10, moving the stylus 10 cm moves x_d 0.5 m, not 1 m). The rotation is
     *        not limited. To move further, release the clutch and press it again. Zero or negative: no limit.
     * @throws std::invalid_argument if max_translation is NaN or infinite.
     */
    explicit HapticPoseMapping(const double& max_translation = 0.0);

    /**
     * @brief Returns r_offset, the rotation from a CoppeliaSim camera to the axes of the device.
     * @return The rotation of 180 deg about the y axis.
     */
    static DQ_robotics::DQ camera_to_device_rotation();

    /**
     * @brief Latches the poses at the press of the clutch.
     * @param device_pose x_h1, the pose of the stylus at the press, in the device frame F_H, unscaled (unit dual
     *        quaternion), e.g. the pose of sas_msgs/InformationFromMaster.
     * @param end_effector_pose x_e1, the pose of the end effector (unit dual quaternion), e.g. in the world frame.
     * @param camera_pose Pose of the reference camera, in the same frame as end_effector_pose (unit dual quaternion;
     *        only its rotation is used).
     * @param motion_scaling Motion scaling s (positive, finite).
     * @throws std::invalid_argument if a pose is not a unit dual quaternion (up to rounding), or if motion_scaling is
     *         not valid.
     */
    void engage(const DQ_robotics::DQ& device_pose, const DQ_robotics::DQ& end_effector_pose,
                const DQ_robotics::DQ& camera_pose, const double& motion_scaling);

    /// @brief Leaves the engaged state (at the release of the clutch).
    void disengage();

    /**
     * @brief Checks if the mapping is engaged.
     * @return True between engage() and disengage().
     */
    bool is_engaged() const;

    /**
     * @brief Computes the desired pose x_d for the current pose of the device.
     * @param device_pose x_h, the current pose of the stylus, in the device frame F_H, unscaled (unit dual
     *        quaternion), e.g. the pose of sas_msgs/InformationFromMaster. The same kind of pose as in engage(): only
     *        its change since the press moves x_d.
     * @return x_d, in the frame of the end effector pose given to engage() (e.g. the world frame). It equals x_e1
     *         when the stylus is at its pose of the press.
     * @throws std::runtime_error if not engaged.
     * @throws std::invalid_argument if device_pose is not a unit dual quaternion.
     */
    DQ_robotics::DQ desired_pose(const DQ_robotics::DQ& device_pose) const;
};

}
