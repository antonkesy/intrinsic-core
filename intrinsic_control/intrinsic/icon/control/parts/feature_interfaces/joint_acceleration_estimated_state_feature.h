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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_ESTIMATED_STATE_FEATURE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_ESTIMATED_STATE_FEATURE_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/algorithms/linear_joint_acceleration_filter.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/proto/linear_joint_acceleration_filter_config.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

// This class implements a JointAccelerationEstimator feature interface based on
// a linear filter estimation which uses a recursive estimation based on sensed
// position and velocity measurements (from hardware interfaces). The
// acceleration for each joint degree-of-freedom is estimated with its own
// single dof filter.
//
// Please note that this model will not yield meaningful results without tuning
// the parameters for a specific robot type.
class JointAccelerationEstimatedStateFeature
    : public HalFeatureInterfaceBase,
      public JointAccelerationEstimator {
  using JointPositionStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointPositionState>;
  using JointVelocityStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>;

 public:
  static absl::StatusOr<JointAccelerationEstimatedStateFeature> Create(
      JointPositionStateHardwareInterface
          joint_position_state_hardware_interface,
      JointVelocityStateHardwareInterface
          joint_velocity_state_hardware_interface,
      intrinsic_proto::icon::LinearJointAccelerationFilterConfig
          linear_joint_acceleration_filter_config,
      double control_frequency_hz);

  JointAccelerationEstimatedStateFeature(
      const JointAccelerationEstimatedStateFeature&) = delete;
  JointAccelerationEstimatedStateFeature& operator=(
      const JointAccelerationEstimatedStateFeature&) = delete;
  JointAccelerationEstimatedStateFeature(
      JointAccelerationEstimatedStateFeature&& other) = default;
  JointAccelerationEstimatedStateFeature& operator=(
      JointAccelerationEstimatedStateFeature&& other) = default;
  ~JointAccelerationEstimatedStateFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;
  RealtimeStatus Reset() override;
  JointStateA GetAccelerationEstimate() const override;

 private:
  // `joint_position_state_hardware_interface` and
  // `joint_velocity_state_hardware_interface` are read only hardware interfaces
  // for obtaining the measured position and velocity joint states.
  // `sensed_state` and `estimated_acceleration` are joint state types with
  // matching size corresponding with the size of the hardware interfaces.
  // Finally `joint_acceleration_estimators` is a vector of filters, with size
  // matching that of the hardware interfaces.
  JointAccelerationEstimatedStateFeature(
      JointPositionStateHardwareInterface
          joint_position_state_hardware_interface,
      JointVelocityStateHardwareInterface
          joint_velocity_state_hardware_interface,
      JointStatePV sensed_state, JointStateA estimated_acceleration,
      std::vector<intrinsic::icon::LinearJointAccelerationFilter>
          joint_acceleration_estimators);

  JointPositionStateHardwareInterface joint_position_state_hardware_interface_;
  JointVelocityStateHardwareInterface joint_velocity_state_hardware_interface_;
  JointStatePV sensed_state_;
  JointStateA estimated_acceleration_;
  std::vector<intrinsic::icon::LinearJointAccelerationFilter>
      joint_acceleration_estimators_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_ESTIMATED_STATE_FEATURE_H_
