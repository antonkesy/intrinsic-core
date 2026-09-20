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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_COMMAND_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_COMMAND_H_

#include <memory>
#include <optional>
#include <variant>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/joint_acceleration_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

// Implementation of the JointAccelerationCommandFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class JointAccelerationCommandFeature : public HalFeatureInterfaceBase,
                                        public JointAcceleration {
  using JointAccelerationAndTorqueCommandHardwareInterface =
      MutableHardwareInterfaceHandle<
          intrinsic_fbs::JointAccelerationAndTorqueCommand>;

 public:
  static absl::StatusOr<JointAccelerationCommandFeature> Create(
      JointAccelerationAndTorqueCommandHardwareInterface hardware_interface);

  JointAccelerationCommandFeature(const JointAccelerationCommandFeature&) =
      delete;
  JointAccelerationCommandFeature& operator=(
      const JointAccelerationCommandFeature&) = delete;
  JointAccelerationCommandFeature(JointAccelerationCommandFeature&& other) =
      default;
  JointAccelerationCommandFeature& operator=(
      JointAccelerationCommandFeature&& other) = default;
  ~JointAccelerationCommandFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  RealtimeStatus SetAccelerationSetpoints(
      const JointAccelerationCommand& setpoints) override;

  JointAccelerationCommand PreviousAccelerationSetpoints() const override;

 private:
  JointAccelerationCommandFeature(
      JointAccelerationAndTorqueCommandHardwareInterface hardware_interface,
      JointAccelerationCommand previous_setpoints);

  JointAccelerationAndTorqueCommandHardwareInterface hardware_interface_;
  JointAccelerationCommand previous_setpoints_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_ACCELERATION_COMMAND_H_
