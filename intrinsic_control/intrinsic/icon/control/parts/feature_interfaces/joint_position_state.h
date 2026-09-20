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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_STATE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_STATE_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

// Implementation of the JointPositionStateFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class JointPositionStateFeature : public HalFeatureInterfaceBase,
                                  public JointPositionSensor {
  using JointPositionStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointPositionState>;

 public:
  static absl::StatusOr<JointPositionStateFeature> Create(
      JointPositionStateHardwareInterface
          joint_position_state_hardware_interface);

  JointPositionStateFeature(const JointPositionStateFeature&) = delete;
  JointPositionStateFeature& operator=(const JointPositionStateFeature&) =
      delete;
  JointPositionStateFeature(JointPositionStateFeature&& other) = default;
  JointPositionStateFeature& operator=(JointPositionStateFeature&& other) =
      default;
  ~JointPositionStateFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;
  JointStateP GetSensedPosition() const override;

 private:
  explicit JointPositionStateFeature(
      JointPositionStateHardwareInterface
          joint_position_state_hardware_interface,
      JointStateP sensed_position);

  JointPositionStateHardwareInterface joint_position_state_hardware_interface_;
  // The JointStateP type serves as an intermediate conversion from the
  // underlying flatbuffer type to a user-facing feature interface.
  JointStateP sensed_position_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_STATE_H_
