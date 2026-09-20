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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_STATE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_STATE_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

// Implementation of the JointAccelerationStateFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class JointAccelerationStateFeature : public HalFeatureInterfaceBase,
                                      public JointAccelerationEstimator {
  using JointAccelerationStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointAccelerationState>;

 public:
  static absl::StatusOr<JointAccelerationStateFeature> Create(
      JointAccelerationStateHardwareInterface
          joint_acceleration_state_hardware_interface);

  JointAccelerationStateFeature(const JointAccelerationStateFeature&) = delete;
  JointAccelerationStateFeature& operator=(
      const JointAccelerationStateFeature&) = delete;
  JointAccelerationStateFeature(JointAccelerationStateFeature&& other) =
      default;
  JointAccelerationStateFeature& operator=(
      JointAccelerationStateFeature&& other) = default;
  ~JointAccelerationStateFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;
  JointStateA GetAccelerationEstimate() const override;

 private:
  explicit JointAccelerationStateFeature(
      JointAccelerationStateHardwareInterface
          joint_acceleration_state_hardware_interface,
      JointStateA sensed_acceleration);

  JointAccelerationStateHardwareInterface
      joint_acceleration_state_hardware_interface_;
  JointStateA sensed_acceleration_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_STATE_H_
