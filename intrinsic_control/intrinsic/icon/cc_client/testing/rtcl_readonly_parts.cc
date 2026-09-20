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

#include "intrinsic/icon/cc_client/testing/rtcl_readonly_parts.h"

#include <gmock/gmock.h>

#include <cstddef>
#include <memory>
#include <utility>

#include "intrinsic/icon/control/parts/fake_feature_interfaces.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/parts/testing/fake_realtime_part.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

std::unique_ptr<RealtimePartInterface> FakeReadOnlyNDofArm(
    size_t ndof, const JointLimits& application_limits,
    const JointLimits& system_limits, const CartesianLimits& cart_limits) {
  FakeJointPositionSensor position_sensor(ndof);
  FakeJointVelocityEstimator velocity_estimator(ndof);
  FakeJointAccelerationEstimator acceleration_estimator(ndof);
  FakeJointTorqueSensor torque_sensor(ndof);
  MockDynamics dynamics;
  ON_CALL(dynamics.DynamicsMock(), GetNumDof)
      .WillByDefault(::testing::Return(ndof));

  return std::make_unique<FakeRealtimePart<
      FakeJointPosition, FakeJointPositionSensor, FakeJointVelocity,
      FakeJointVelocityEstimator, FakeJointAccelerationEstimator,
      FakeJointTorque, FakeJointTorqueSensor, FakeJointLimits,
      FakeCartesianLimits, MockDynamics, FakeManipulatorKinematics,
      FakeHandGuiding, FakeMoveOk, FakeHoming>>(
      FakeJointPosition(ndof), std::move(position_sensor),
      FakeJointVelocity(ndof), std::move(velocity_estimator),
      std::move(acceleration_estimator), FakeJointTorque(ndof),
      std::move(torque_sensor),
      FakeJointLimits(application_limits, system_limits),
      FakeCartesianLimits(cart_limits), std::move(dynamics),
      FakeManipulatorKinematics(ndof), FakeHandGuiding(), FakeMoveOk(),
      FakeHoming());
}

std::unique_ptr<RealtimePartInterface> FakeReadOnlyNDofArm(size_t ndof) {
  INTRINSIC_RT_ASSIGN_OR_DIE(auto joint_limits, JointLimits::Unlimited(ndof));
  CartesianLimits cart_limits;
  cart_limits.SetUnlimited();
  return FakeReadOnlyNDofArm(ndof, joint_limits, joint_limits, cart_limits);
}

std::unique_ptr<RealtimePartInterface> FakeReadOnlyGripper() {
  return std::make_unique<FakeRealtimePart<FakeGripper>>(FakeGripper());
}

std::unique_ptr<RealtimePartInterface> FakeReadOnlyLinearGripper() {
  return std::make_unique<FakeRealtimePart<FakeLinearGripper>>(
      FakeLinearGripper());
}

std::unique_ptr<RealtimePartInterface> FakeReadOnlyForceTorqueSensor() {
  return std::make_unique<FakeRealtimePart<FakeForceTorqueSensor>>(
      FakeForceTorqueSensor());
}

std::unique_ptr<RealtimePartInterface> FakeReadOnlyNDofArmForceTorqueSensorPart(
    size_t ndof, const JointLimits& application_limits,
    const JointLimits& system_limits, const CartesianLimits& cart_limits) {
  FakeJointPositionSensor position_sensor(ndof);
  FakeJointVelocityEstimator velocity_estimator(ndof);
  FakeJointAccelerationEstimator acceleration_estimator(ndof);
  FakeJointTorqueSensor torque_sensor(ndof);
  MockDynamics dynamics;
  ON_CALL(dynamics.DynamicsMock(), GetNumDof)
      .WillByDefault(::testing::Return(ndof));
  FakeForceTorqueSensor ft_sensor;

  return std::make_unique<FakeRealtimePart<
      FakeJointPosition, FakeJointPositionSensor, FakeJointVelocity,
      FakeJointVelocityEstimator, FakeJointAccelerationEstimator,
      FakeJointTorque, FakeJointTorqueSensor, FakeJointLimits,
      FakeCartesianLimits, MockDynamics, FakeManipulatorKinematics,
      FakeForceTorqueSensor>>(
      FakeJointPosition(ndof), std::move(position_sensor),
      FakeJointVelocity(ndof), std::move(velocity_estimator),
      std::move(acceleration_estimator), FakeJointTorque(ndof),
      std::move(torque_sensor),
      FakeJointLimits(application_limits, system_limits),
      FakeCartesianLimits(cart_limits), std::move(dynamics),
      FakeManipulatorKinematics(ndof), std::move(ft_sensor));
}

std::unique_ptr<RealtimePartInterface> FakeReadOnlyNDofArmForceTorqueSensorPart(
    size_t ndof) {
  INTRINSIC_RT_ASSIGN_OR_DIE(auto joint_limits, JointLimits::Unlimited(ndof));
  CartesianLimits cart_limits;
  cart_limits.SetUnlimited();
  return FakeReadOnlyNDofArmForceTorqueSensorPart(ndof, joint_limits,
                                                  joint_limits, cart_limits);
}

std::unique_ptr<RealtimePartInterface> FakeReadOnlyADIO(
    const FakeADIO::FakeADIOState& state) {
  return std::make_unique<FakeRealtimePart<FakeADIO>>(FakeADIO(state));
}

std::unique_ptr<RealtimePartInterface> FakeReadOnlyRangefinderPart(
    double sensed_distance, const Pose3d& pose) {
  return std::make_unique<FakeRealtimePart<FakeRangeFinder>>(
      FakeRangeFinder(sensed_distance, pose));
}

}  // namespace intrinsic::icon
