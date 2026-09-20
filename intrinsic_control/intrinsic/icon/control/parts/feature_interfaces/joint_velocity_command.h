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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_VELOCITY_COMMAND_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_VELOCITY_COMMAND_H_

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// Implementation of the JointVelocityCommandFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class JointVelocityCommandFeature : public HalFeatureInterfaceBase,
                                    public JointVelocity {
  using JointVelocityCommandHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::JointVelocityCommand>;

 public:
  static absl::StatusOr<JointVelocityCommandFeature> Create(
      JointVelocityCommandHardwareInterface
          joint_velocity_command_hardware_interface);

  JointVelocityCommandFeature(const JointVelocityCommandFeature&) = delete;
  JointVelocityCommandFeature& operator=(const JointVelocityCommandFeature&) =
      delete;
  JointVelocityCommandFeature(JointVelocityCommandFeature&& other) = default;
  JointVelocityCommandFeature& operator=(JointVelocityCommandFeature&& other) =
      default;
  ~JointVelocityCommandFeature() override = default;

  RealtimeStatus SetVelocitySetpoints(
      const eigenmath::VectorNd& setpoints) override;

 private:
  explicit JointVelocityCommandFeature(
      JointVelocityCommandHardwareInterface
          joint_velocity_command_hardware_interface);

  JointVelocityCommandHardwareInterface
      joint_velocity_command_hardware_interface_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_VELOCITY_COMMAND_H_
