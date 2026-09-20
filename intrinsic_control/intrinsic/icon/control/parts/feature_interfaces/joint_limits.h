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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_LIMITS_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_LIMITS_H_

#include <memory>
#include <optional>

#include "absl/container/fixed_array.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/proto/v1/mode_of_safe_operation_limits_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_limits.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"

namespace intrinsic::icon {

// Implementation of the JointLimitsFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
// Automatically computes a separate limit set that leaves some room for control
// by scaling the VAJT component of the limits provided by the hardware
// interface.
// Additional limits may be configured for additional safety modes
// via the ModeOfSafeOperationLimitsConfig.
class JointLimitsFeature : public HalFeatureInterfaceBase,
                           public JointLimitsInterface {
  using JointLimitsHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointLimits>;
  using JointPositionCommandHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointPositionCommand>;
  using MutableJointLimitHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::JointLimits>;

 public:
  // Creates a new JointLimitsFeature that uses `system_limits` and
  // `application_limits`.
  //
  // This factory uses those system and application limits to fill in any
  // unpopulated entries in the mode->limit map in `config`.
  //
  // When a valid `joint_position_command_hardware_interface` and a valid
  // `rigid_body_dynamics` are provided, at each cycle within the `ReadStatus`
  // method, the joint acceleration limits from dynamics are updated based
  // on the previous command setpoint (joint position/joint velocity) and
  // minimum and maximum torque limits. Otherwise, querying these limits will
  // return std::nullopt.
  //
  // Returns FailedPreconditionError if any of the application limits do not
  // leave a sufficient margin to the system limits.
  static absl::StatusOr<JointLimitsFeature> Create(
      JointLimits system_limits, JointLimits application_limits,
      const intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig& config,
      std::optional<JointPositionCommandHardwareInterface>
          joint_position_command_hardware_interface = std::nullopt,
      std::unique_ptr<icon::RigidBodyInterface> rigid_body_dynamics = nullptr,
      std::optional<MutableJointLimitHardwareInterface>
          joint_system_limits_hardware_interface = std::nullopt);

  JointLimitsFeature(const JointLimitsFeature&) = delete;
  JointLimitsFeature& operator=(const JointLimitsFeature&) = delete;
  JointLimitsFeature(JointLimitsFeature&& other) = default;
  JointLimitsFeature& operator=(JointLimitsFeature&& other) = delete;
  ~JointLimitsFeature() override = default;

  // Bundles the system and application limits used by the part.
  struct JointLimitBundle {
    // The application joint limits that leave some room for control to the
    // system joint limits.
    JointLimits application_limits;
    // The system joint limits in which the robot is allowed to operate.
    JointLimits system_limits;
  };

  JointLimits GetApplicationLimits() const override;
  JointLimits GetSystemLimits() const override;
  icon::RealtimeStatusOr<std::optional<JointAccelerationLimitsFromDynamics>>
  GetJointAccelerationLimitsFromDynamics() const override;

  // Save the latest safety status during this call.
  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) override;

  const JointLimitBundle& GetLimitBundleForModeOfSafeOperation(
      intrinsic_fbs::ModeOfSafeOperation mode) const;

 private:
  explicit JointLimitsFeature(
      absl::Span<JointLimitBundle> limits_in_enum_order,
      std::optional<JointPositionCommandHardwareInterface>
          joint_position_command_hardware_interface,
      std::unique_ptr<icon::RigidBodyInterface> rigid_body_dynamics,
      std::optional<MutableJointLimitHardwareInterface>
          joint_system_limits_hardware_interface);

  SafetyStatus current_safety_status_;

  // Contains a JointLimitBundle (application and system limits) for every
  // possible ModeOfSafeOperation in the order of the values of
  // ModeOfSafeOperation.
  absl::FixedArray<JointLimitBundle> limits_in_enum_order_;

  // Joint acceleration limits at minimum and maximum torque values computed
  // via forward dynamics (only if a rigid body dynamics model is available).
  mutable std::optional<
      JointLimitsInterface::JointAccelerationLimitsFromDynamics>
      joint_acceleration_limits_from_dynamics_;

  // Access to previous joint position command to construct limits based on
  // forward dynamics computations at previous setpoints.
  std::optional<JointPositionCommandHardwareInterface>
      joint_position_command_hardware_interface_;

  std::optional<MutableJointLimitHardwareInterface>
      joint_system_limits_hardware_interface_;

  // Rigid body dynamics model to compute forward dynamics acceleration limits.
  std::unique_ptr<RigidBodyInterface> rigid_body_dynamics_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_LIMITS_H_
