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
 * @file test_haptic_pose_mapping.cpp
 * @brief Tests of HapticPoseMapping.
 */

// Tests of the HapticPoseMapping: the desired pose of an end effector set with the pose of a haptic device.

#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>
#include <haptic_navigation/haptic_pose_mapping.hpp>

using haptic_tools::HapticPoseMapping;
using namespace DQ_robotics;

namespace
{

constexpr double kTolerance = 1e-9;

DQ rotation(const DQ& axis, const double& angle)
{
    return cos(angle/2.0) + sin(angle/2.0)*axis;
}

DQ pose(const DQ& r, const DQ& t)
{
    return r + 0.5*E_*t*r;
}

DQ point(const double& x, const double& y, const double& z)
{
    return x*i_ + y*j_ + z*k_;
}

/// x and -x are the same pose.
void expect_same_pose(const DQ& a, const DQ& b)
{
    const double distance = std::min((a.vec8() - b.vec8()).norm(), (a.vec8() + b.vec8()).norm());
    EXPECT_LT(distance, kTolerance) << "a = " << a << ", b = " << b;
}

void expect_same_point(const DQ& a, const DQ& b)
{
    EXPECT_LT((a.vec3() - b.vec3()).norm(), kTolerance) << "a = " << a << ", b = " << b;
}

// An end effector, a device, and a camera in arbitrary poses.
const DQ kEndEffector = pose(normalize(rotation(i_, 0.4)*rotation(k_, -1.1)), point(0.3, -0.2, 0.9));
const DQ kDevice = pose(normalize(rotation(j_, 0.2)*rotation(i_, 0.7)), point(0.01, -0.02, 0.05));
const DQ kCamera = pose(normalize(rotation(k_, 2.0)*rotation(i_, -0.5)), point(1.5, 0.5, 1.2));

}

TEST(HapticPoseMapping, AtThePressTheDesiredPoseIsTheEndEffector)
{
    HapticPoseMapping mapping;
    EXPECT_FALSE(mapping.is_engaged());
    mapping.engage(kDevice, kEndEffector, kCamera, 2.0);
    EXPECT_TRUE(mapping.is_engaged());
    expect_same_pose(mapping.desired_pose(kDevice), kEndEffector);
}

TEST(HapticPoseMapping, PositionControl)
{
    // Holding the stylus still holds the desired pose still, and moving it back to where it was pressed brings the
    // desired pose back to the end effector (the original formula of haptic_interface drifted instead: the frame of
    // the camera was placed at the current end effector in every cycle).
    HapticPoseMapping mapping;
    mapping.engage(kDevice, kEndEffector, kCamera, 1.5);
    const DQ moved = pose(kDevice.rotation(), kDevice.translation() + point(0.02, 0.0, -0.01));
    const DQ xd = mapping.desired_pose(moved);
    expect_same_pose(mapping.desired_pose(moved), xd);
    expect_same_pose(mapping.desired_pose(kDevice), kEndEffector);
}

TEST(HapticPoseMapping, AxesOfTheCamera)
{
    // A camera with the identity rotation looks along +z with +y up, so its +x points to the left of the image:
    // moving the stylus to the right (+x of the device) moves the end effector along -x, up (+y) along +y, and
    // towards the user (+z) along -z.
    const DQ camera = pose(DQ(1), point(0.0, 0.0, -2.0));
    const double s = 2.0;
    const double d = 0.01;
    struct Case { DQ device_motion; DQ expected; };
    for (const Case& c : {Case{point(d, 0, 0), point(-s*d, 0, 0)},
                          Case{point(0, d, 0), point(0, s*d, 0)},
                          Case{point(0, 0, d), point(0, 0, -s*d)}})
    {
        HapticPoseMapping mapping;
        mapping.engage(kDevice, kEndEffector, camera, s);
        const DQ moved = pose(kDevice.rotation(), kDevice.translation() + c.device_motion);
        expect_same_point(mapping.desired_pose(moved).translation() - kEndEffector.translation(), c.expected);
    }
}

TEST(HapticPoseMapping, TranslationInTheFrameOfAnyCamera)
{
    // t_d = t_e1 + Ad(r_c*r_offset)*(s*p_delta).
    HapticPoseMapping mapping;
    const double s = 3.0;
    mapping.engage(kDevice, kEndEffector, kCamera, s);
    const DQ p_delta = point(0.01, -0.03, 0.02);
    const DQ moved = pose(kDevice.rotation(), kDevice.translation() + p_delta);
    const DQ r_ref = kCamera.rotation()*HapticPoseMapping::camera_to_device_rotation();
    expect_same_point(mapping.desired_pose(moved).translation(),
                      kEndEffector.translation() + Ad(r_ref, s*p_delta));
    // The orientation does not change with a translation of the device.
    expect_same_pose(mapping.desired_pose(moved).rotation(), kEndEffector.rotation());
}

TEST(HapticPoseMapping, TranslationDoesNotDependOnHowTheStylusIsHeld)
{
    // The increments are in the device frame: the same displacement of the stylus gives the same displacement of
    // the end effector, whatever the orientation of the stylus at the press.
    const DQ p_delta = point(0.02, 0.0, 0.0);
    DQ offsets[2];
    const DQ stylus[2] = {DQ(1), normalize(rotation(i_, M_PI/4.0)*rotation(j_, 0.6))};
    for (int n = 0; n < 2; n++)
    {
        HapticPoseMapping mapping;
        const DQ pressed = pose(stylus[n], kDevice.translation());
        mapping.engage(pressed, kEndEffector, kCamera, 1.0);
        const DQ moved = pose(stylus[n], kDevice.translation() + p_delta);
        offsets[n] = mapping.desired_pose(moved).translation() - kEndEffector.translation();
    }
    expect_same_point(offsets[0], offsets[1]);
}

TEST(HapticPoseMapping, RotationAboutTheEndEffector)
{
    // A rotation of the stylus rotates the end effector about itself (its position does not change), about the axis
    // of the device as seen in the image: with the identity camera, +y of the device is +y, and +x is -x.
    const DQ camera = DQ(1);
    const double angle = 0.5;
    struct Case { DQ device_axis; DQ end_effector_axis; };
    for (const Case& c : {Case{j_, j_}, Case{i_, -1.0*i_}, Case{k_, -1.0*k_}})
    {
        HapticPoseMapping mapping;
        mapping.engage(kDevice, kEndEffector, camera, 2.0);
        // A rotation about the axis of the device frame: r_h = r_delta*r_h1.
        const DQ rotated = pose(normalize(rotation(c.device_axis, angle)*kDevice.rotation()), kDevice.translation());
        const DQ xd = mapping.desired_pose(rotated);
        expect_same_point(xd.translation(), kEndEffector.translation());
        expect_same_pose(xd.rotation(), normalize(rotation(c.end_effector_axis, angle)*kEndEffector.rotation()));
    }
}

TEST(HapticPoseMapping, FullPose)
{
    // Translation and rotation together: t_d = t_e1 + Ad(r_ref)*(s*p_delta), r_d = (r_ref*r_delta*r_ref^*)*r_e1.
    HapticPoseMapping mapping;
    const double s = 1.7;
    mapping.engage(kDevice, kEndEffector, kCamera, s);
    const DQ r_delta = normalize(rotation(i_, 0.3)*rotation(k_, -0.2));
    const DQ p_delta = point(-0.01, 0.015, 0.004);
    const DQ moved = pose(normalize(r_delta*kDevice.rotation()), kDevice.translation() + p_delta);
    const DQ r_ref = normalize(kCamera.rotation()*HapticPoseMapping::camera_to_device_rotation());
    const DQ xd = mapping.desired_pose(moved);
    expect_same_point(xd.translation(), kEndEffector.translation() + Ad(r_ref, s*p_delta));
    expect_same_pose(xd.rotation(), normalize(r_ref*r_delta*r_ref.conj()*kEndEffector.rotation()));
}

TEST(HapticPoseMapping, MaxTranslation)
{
    HapticPoseMapping mapping(0.05);
    mapping.engage(kDevice, kEndEffector, kCamera, 10.0);
    const DQ moved = pose(kDevice.rotation(), kDevice.translation() + point(0.03, 0.04, 0.0));   // 0.5 m scaled
    const DQ offset = mapping.desired_pose(moved).translation() - kEndEffector.translation();
    EXPECT_NEAR(offset.vec3().norm(), 0.05, kTolerance);
    // Below the limit, the displacement is not changed.
    const DQ small = pose(kDevice.rotation(), kDevice.translation() + point(0.003, 0.0, 0.0));   // 0.03 m scaled
    EXPECT_NEAR((mapping.desired_pose(small).translation() - kEndEffector.translation()).vec3().norm(), 0.03, kTolerance);
}

TEST(HapticPoseMapping, Disengage)
{
    HapticPoseMapping mapping;
    mapping.engage(kDevice, kEndEffector, kCamera, 1.0);
    mapping.disengage();
    EXPECT_FALSE(mapping.is_engaged());
    EXPECT_THROW(mapping.desired_pose(kDevice), std::runtime_error);
}

TEST(HapticPoseMapping, InvalidArguments)
{
    HapticPoseMapping mapping;
    EXPECT_THROW(mapping.desired_pose(kDevice), std::runtime_error);
    EXPECT_THROW(mapping.engage(kDevice, kEndEffector, kCamera, 0.0), std::invalid_argument);
    EXPECT_THROW(mapping.engage(kDevice, kEndEffector, kCamera, -1.0), std::invalid_argument);
    EXPECT_THROW(mapping.engage(kDevice, kEndEffector, kCamera, std::nan("")), std::invalid_argument);
    EXPECT_THROW(mapping.engage(2.0*kDevice, kEndEffector, kCamera, 1.0), std::invalid_argument);
    EXPECT_THROW(mapping.engage(kDevice, DQ(0), kCamera, 1.0), std::invalid_argument);
    EXPECT_FALSE(mapping.is_engaged());
    EXPECT_THROW(HapticPoseMapping(std::nan("")), std::invalid_argument);

    // A pose with rounding errors (e.g. a quaternion sent in single precision) is accepted, and normalized.
    mapping.engage((1.0 + 1e-5)*kDevice, kEndEffector, kCamera, 1.0);
    expect_same_pose(mapping.desired_pose(kDevice), kEndEffector);
}
