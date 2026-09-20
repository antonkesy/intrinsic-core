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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_COMMAND_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_COMMAND_H_

#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/collision/robot_collision_checker.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/move_checker.h"
#include "intrinsic/icon/control/parts/plane_checker.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/math/signals/numerical_differentiation.h"

namespace intrinsic::icon {

// Implementation of the JointPositionCommandFeatureInterface.
// The implementation wraps a hardware interface allocated on a shared memory
// segment and thus links a hardware module to a part.
class JointPositionCommandFeature : public HalFeatureInterfaceBase,
                                    public JointPosition {
  using JointPositionCommandHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::JointPositionCommand>;
  // Typedef for the reporting the currently active command. Extra type is
  // needed for auto configuration of ICON.
  using ActiveCommandHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointCommandedPosition>;

  using JointPositionStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointPositionState>;

 public:
  static constexpr int kNumDiffOrder = 4;

  // Factory for the JointPositionCommandFeature which checks both JointLimits
  // (position, velocity and acceleration) and optionally the Cartesian position
  // limits.
  //
  // `cartesian_limits` and `manipulator_kinematics_interface` are both
  // optional. If they are both included then JointPositionCommandFeature will
  // be created with Cartesian position limit checking. The absence of either
  // will result in no Cartesian limit checking.
  //
  // `collision_checker` is optional. If it is included the
  // JointPositionCommandFeature will be created with self collision checking
  // for each setpoint.
  static absl::StatusOr<JointPositionCommandFeature> Create(
      JointPositionCommandHardwareInterface
          joint_position_command_hardware_interface,
      JointPositionStateHardwareInterface
          joint_position_state_hardware_interface,
      std::optional<ActiveCommandHardwareInterface>
          joint_command_position_hardware_interface,
      double control_frequency_hz,
      const JointLimitsInterface* joint_limits_interface,
      std::optional<CartesianLimits> cartesian_limits = std::nullopt,
      const ManipulatorKinematics* manipulator_kinematics_interface = nullptr,
      std::unique_ptr<collision::RobotCollisionChecker> collision_checker =
          nullptr);

  JointPositionCommandFeature(const JointPositionCommandFeature&) = delete;
  JointPositionCommandFeature& operator=(const JointPositionCommandFeature&) =
      delete;
  JointPositionCommandFeature(JointPositionCommandFeature&& other) = default;
  JointPositionCommandFeature& operator=(JointPositionCommandFeature&& other) =
      default;
  ~JointPositionCommandFeature() override = default;

  // Resets the previous setpoints to the current sensed position and zero
  // velocity and acceleration. This accounts for scenarios where a robot is
  // disabled in ICON, then moved "offline", then re-enabled. Old "previous
  // setpoints" values are likely to not be valid any more.
  //
  // Also resets the internal velocity/acceleration estimator, because changing
  // the previous setpoints as outlined above may introduce a discontinuity that
  // makes the estimator's output invalid.
  RealtimeStatus Reset() override;

  // Since this is called at the very start of a cycle (before any ICON Actions
  // can interact with this class), we can save the current setpoints to use as
  // "previous setpoints" here.
  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  // This function uses the setpoints from the most recent call to
  // `SetPositionSetpoints()` (the rest of this comment will refer to those as
  // just "current setpoints").
  //
  // Using the current setpoints, this function then
  //
  // 1) Updates the internal velocity/acceleration estimator with the position
  //    values from the current setpoints.
  // 2) Uses the updated estimator to fill in any missing feedforwards (velocity
  //    and/or acceleration) in the current setpoints.
  // 3) Writes the current setpoints to the hardware module.
  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) override;

  RealtimeStatus SetPositionSetpoints(
      const JointPositionCommand& setpoints) override;

  JointPositionCommand PreviousPositionSetpoints() const override;

 private:
  // `manipulator_kinematics_interface` and `plane_checker` are optional. The
  // inclusion of those parameters will construct the class with Cartesian
  // position limit checking.
  JointPositionCommandFeature(
      JointPositionCommandHardwareInterface&&
          joint_position_command_hardware_interface,
      JointPositionStateHardwareInterface&&
          joint_position_state_hardware_interface,
      std::optional<ActiveCommandHardwareInterface>
          active_command_hardware_interface,
      JointPositionCommand position_setpoints,
      JointPositionCommand previous_setpoints, double control_frequency_hz,
      const JointLimitsInterface* joint_limits_interface,
      const ManipulatorKinematics* manipulator_kinematics_interface,
      std::unique_ptr<MoveChecker> move_checker,
      std::unique_ptr<PlaneChecker> plane_checker);

  const JointLimitsInterface* joint_limits_interface_;

  // If the last position command is considered stale, then this function
  // overrides the previous_setpoints_ with the current state. This ensures that
  // when the caller starts calling PreviousPositionSetpoints again, they read a
  // value that is representative of the current state (since the robot may have
  // moved under a different control mode).
  //
  // Note: this approach might run into problems if the user is intentionally
  // switching control modes at a rate faster than our notion of "stale", but
  // this is an uncommon use-case.
  //
  RealtimeStatus OverridePreviousSetpointsIfStale();

  // The `manipulator_kinematics_interface_` is used to generate a robot tip
  // pose from the setpoints so as to check Cartesian position limits using
  // `plane_checker_`.
  const ManipulatorKinematics* manipulator_kinematics_interface_;
  TimeSeriesNumDiff<eigenmath::VectorNd> position_setpoints_numdiff_;
  // Use numdiff to get an estimate for the state from the position.
  TimeSeriesNumDiff<eigenmath::VectorNd> sensed_position_numdiff_;

  JointPositionCommandHardwareInterface
      joint_position_command_hardware_interface_;
  JointPositionStateHardwareInterface joint_position_state_hardware_interface_;
  std::optional<ActiveCommandHardwareInterface>
      active_command_hardware_interface_;

  // New position setpoints to be set in this cycle, and the associated time of
  // update.
  std::optional<JointPositionCommand> position_setpoints_;
  std::optional<Time> updated_time_;

  // Number of cycles since the last call to SetPositionSetpoints.
  int cycles_since_last_update_;

  JointPositionCommand previous_setpoints_;

  std::unique_ptr<MoveChecker> move_checker_;

  std::unique_ptr<PlaneChecker> plane_checker_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_POSITION_COMMAND_H_
