// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_READONLY_PARTS_H_
#define INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_READONLY_PARTS_H_

#include <cstddef>
#include <memory>

#include "intrinsic/icon/control/parts/fake_feature_interfaces.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

// Constructs a fake RealtimePart for an n-DoF arm that offers the following
// features:
// * JointPosition
// * JointPositionSensor
// * JointVelocity,
// * JointVelocityEstimator
// * JointTorque
// * JointTorqueSensor,
// * JointLimitsInterface
// * CartesianLimitsInterface
// * MockDynamics
// * ManipulatorKinematics
// * HandGuiding
// * MoveOk
// * Homing
//
// Note that the fake RealtimePart does not implement behavior, but only
// supplies plausible values for the const methods of those features.
// In particular, there is no "loopback" behavior, since RtclChannelFake does
// not send any commands to Parts anyway. `application_limits` and
// `system_limits` are reported by the JointLimitsInterface feature.
// `cart_limits` are reported by the CartLimits feature.
std::unique_ptr<RealtimePartInterface> FakeReadOnlyNDofArm(
    size_t ndof, const JointLimits& application_limits,
    const JointLimits& system_limits, const CartesianLimits& cart_limits);

// Same as above, but with unlimited JointLimits instances for both max and
// default limits, and unlimited CartesianLimitsInterface.
std::unique_ptr<RealtimePartInterface> FakeReadOnlyNDofArm(size_t ndof);

// Constructs a fake RealtimePart that implements the SimpleGripper feature.
std::unique_ptr<RealtimePartInterface> FakeReadOnlyGripper();

// Constructs a fake RealtimePart that implements the LinearGripper feature.
std::unique_ptr<RealtimePartInterface> FakeReadOnlyLinearGripper();

// Constructs a fake RealtimePart that implements the ForceTorqueSensorPart
// features.
std::unique_ptr<RealtimePartInterface> FakeReadOnlyForceTorqueSensor();

// Constructs a fake "composite" RealtimePart for an n-DoF arm combined with a
// ForceTorque sensor that offers the following features:
// * JointPosition
// * JointPositionSensor
// * JointVelocity,
// * JointVelocityEstimator
// * JointTorque
// * JointTorqueSensor,
// * JointLimitsInterface
// * CartesianLimitsInterface
// * MockDynamics
// * ManipulatorKinematics
// * ForceTorqueSensor
// * MoveOk
//
std::unique_ptr<RealtimePartInterface> FakeReadOnlyNDofArmForceTorqueSensorPart(
    size_t ndof, const JointLimits& application_limits,
    const JointLimits& system_limits, const CartesianLimits& cart_limits);

// Same as above, but with unlimited JointLimits instances for both max and
// default limits, and unlimited CartesianLimitsInterface.
std::unique_ptr<RealtimePartInterface> FakeReadOnlyNDofArmForceTorqueSensorPart(
    size_t ndof);

// Constructs a fake RealtimePart that implements the ADIO feature.
// Doesn't expose any analog-/digital-IOs. Can be adjusted using the parameter
// `state`.
std::unique_ptr<RealtimePartInterface> FakeReadOnlyADIO(
    const FakeADIO::FakeADIOState& state = {});

std::unique_ptr<RealtimePartInterface> FakeReadOnlyRangefinderPart(
    double sensed_distance, const Pose3d& pose);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_READONLY_PARTS_H_
