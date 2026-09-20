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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_TORQUE_COMMAND_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_TORQUE_COMMAND_H_

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// Implementation of the JointTorqueCommandFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class JointTorqueCommandFeature : public HalFeatureInterfaceBase,
                                  public JointTorque {
  using JointTorqueCommandHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::JointTorqueCommand>;

 public:
  static absl::StatusOr<JointTorqueCommandFeature> Create(
      JointTorqueCommandHardwareInterface
          joint_torque_command_hardware_interface);

  JointTorqueCommandFeature(const JointTorqueCommandFeature&) = delete;
  JointTorqueCommandFeature& operator=(const JointTorqueCommandFeature&) =
      delete;
  JointTorqueCommandFeature(JointTorqueCommandFeature&& other) = default;
  JointTorqueCommandFeature& operator=(JointTorqueCommandFeature&& other) =
      default;
  ~JointTorqueCommandFeature() override = default;

  RealtimeStatus SetTorqueSetpoints(
      const eigenmath::VectorNd& setpoints) override;

  eigenmath::VectorNd PreviousTorqueSetpoints() const override;

  // Since this is called at the very start of a cycle (before any ICON Actions
  // can interact with this class), we can save the current setpoints to use as
  // "previous setpoints" here.
  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

 private:
  explicit JointTorqueCommandFeature(
      JointTorqueCommandHardwareInterface
          joint_torque_command_hardware_interface,
      eigenmath::VectorNd previous_torque_setpoints);

  JointTorqueCommandHardwareInterface joint_torque_command_hardware_interface_;

  eigenmath::VectorNd previous_torque_setpoints_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_TORQUE_COMMAND_H_
