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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_PID_TORQUE_CONTROLLER_FEATURE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_PID_TORQUE_CONTROLLER_FEATURE_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/joint_position_pid_torque_controller.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/move_checker.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/proto/joint_position_pid_torque_controller_config.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/signals/numerical_differentiation.h"

namespace intrinsic::icon {
// Implementation of the JointPosition feature interface which calculates joint
// torque values and mutates the corresponding
// JointTorqueCommandHardwareInterface based on a
// `JointPositionPidTorqueController` for each degree of freedom.
class JointPositionPidTorqueControllerFeature : public HalFeatureInterfaceBase,
                                                public JointPosition {
  using JointTorqueCommandHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::JointTorqueCommand>;
  using JointPositionStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointPositionState>;
  using JointVelocityStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>;

 public:
  static absl::StatusOr<JointPositionPidTorqueControllerFeature> Create(
      JointTorqueCommandHardwareInterface
          joint_torque_command_hardware_interface,
      JointPositionStateHardwareInterface
          joint_position_state_hardware_interface,
      JointVelocityStateHardwareInterface
          joint_velocity_state_hardware_interface,
      intrinsic_proto::icon::ArmPositionPidTorqueControllerConfig config,
      double control_frequency_hz,
      const JointLimitsInterface* joint_limits_interface);

  JointPositionPidTorqueControllerFeature(
      const JointPositionPidTorqueControllerFeature&) = delete;
  JointPositionPidTorqueControllerFeature& operator=(
      const JointPositionPidTorqueControllerFeature&) = delete;
  JointPositionPidTorqueControllerFeature(
      JointPositionPidTorqueControllerFeature&& other) = default;
  JointPositionPidTorqueControllerFeature& operator=(
      JointPositionPidTorqueControllerFeature&& other) = default;
  ~JointPositionPidTorqueControllerFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;
  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) override;
  RealtimeStatus Reset() override;
  RealtimeStatus SetPositionSetpoints(
      const JointPositionCommand& setpoints) override;
  JointPositionCommand PreviousPositionSetpoints() const override;

 private:
  static constexpr int kNumDiffOrder = 4;

  JointPositionPidTorqueControllerFeature(
      JointTorqueCommandHardwareInterface
          joint_torque_command_hardware_interface,
      JointPositionStateHardwareInterface
          joint_position_state_hardware_interface,
      JointVelocityStateHardwareInterface
          joint_velocity_state_hardware_interface,
      JointStatePV sensed_state, JointPositionCommand position_setpoints,
      JointPositionCommand previous_setpoints, double control_frequency_hz,
      const JointLimitsInterface* joint_limits_interface,
      std::unique_ptr<MoveChecker> move_checker,
      std::vector<std::unique_ptr<JointPositionPidTorqueController>>
          controllers);

  // Computes torques and sets them in the hardware interface if this function
  // has not already been called this cycle.
  RealtimeStatus MaybeComputeTorques();

  // Used to track whether torques were computed in this cycle.
  bool computed_torques_this_cycle_ = false;

  const JointLimitsInterface* joint_limits_interface_;
  TimeSeriesNumDiff<eigenmath::VectorNd> position_setpoints_numdiff_;

  JointTorqueCommandHardwareInterface joint_torque_command_hardware_interface_;
  JointPositionStateHardwareInterface joint_position_state_hardware_interface_;
  JointVelocityStateHardwareInterface joint_velocity_state_hardware_interface_;

  std::optional<JointPositionCommand> position_setpoints_;
  JointPositionCommand previous_setpoints_;
  // This time marks when SetPositionSetpoints is called and so indicates
  // whether an Action is actively using this feature interface.
  //
  // Reset to nullopt each cycle.
  std::optional<Time> updated_time_;

  std::unique_ptr<MoveChecker> move_checker_;

  JointStatePV sensed_state_;
  std::vector<std::unique_ptr<JointPositionPidTorqueController>> controllers_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_PID_TORQUE_CONTROLLER_FEATURE_H_
