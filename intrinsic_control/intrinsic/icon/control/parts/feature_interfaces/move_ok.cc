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

#include "intrinsic/icon/control/parts/feature_interfaces/move_ok.h"

#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "flatbuffers/vector.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/move_checker.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::StatusOr<MoveOkFeature> MoveOkFeature::Create(
    double control_frequency_hz, const JointLimits& application_limits,
    const JointLimits& system_limits,
    const JointPosition* joint_position_command_feature_interface) {
  // Initialize the move_checker.
  INTR_ASSIGN_OR_RETURN(
      auto move_checker,
      MoveChecker::Create(
          joint_position_command_feature_interface->PreviousPositionSetpoints()
              .Size(),
          control_frequency_hz));

  return MoveOkFeature(std::move(move_checker), application_limits,
                       system_limits, joint_position_command_feature_interface);
}

MoveOkFeature::MoveOkFeature(
    std::unique_ptr<MoveChecker> move_checker,
    const JointLimits& application_limits, const JointLimits& system_limits,
    const JointPosition* joint_position_command_feature_interface)
    : joint_position_command_feature_interface_(
          joint_position_command_feature_interface),
      move_checker_(std::move(move_checker)),
      application_limits_(application_limits),
      system_limits_(system_limits) {}

RealtimeStatus MoveOkFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  return move_checker_->UpdatePreviousSetpoint(
      joint_position_command_feature_interface_->PreviousPositionSetpoints());
}

bool MoveOkFeature::IsMoveOkWithPartLimits(
    const JointPositionCommand& setpoint) {
  RealtimeStatus status = move_checker_->CheckSetpoint(
      setpoint, application_limits_, system_limits_);
  if (!status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(INFO)
        << "Queried setpoint is not OK: " << status.message();
  }
  return status.ok();
}

bool MoveOkFeature::IsMoveOkWithUserLimits(const JointPositionCommand& setpoint,
                                           const JointLimits& joint_limits) {
  // Check if the user provided `joint_limits` are within
  // GetApplicationLimits().
  auto limit_check = IsWithinLimits(joint_limits, application_limits_);
  if (!limit_check.ok()) {
    return limit_check.ok();
  }
  if (!limit_check.value()) {
    INTRINSIC_RT_LOG_THROTTLED(INFO)
        << "Queried setpoint with user limits is not OK: The user provided "
           "limits violated the application limits.";
    return false;
  }
  RealtimeStatus status =
      move_checker_->CheckSetpoint(setpoint, joint_limits, system_limits_);
  if (!status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(INFO)
        << "Queried setpoint is not OK: " << status.message();
  }
  return status.ok();
}

RealtimeStatus MoveOkFeature::UpdatePreviousPositionSetpoints(
    const JointPositionCommand& setpoints) {
  // Update the move_checker_'s previous setpoint.
  INTRINSIC_RT_RETURN_IF_ERROR(
      move_checker_->UpdatePreviousSetpoint(setpoints));
  return OkStatus();
}

RealtimeStatus MoveOkFeature::UpdateJointLimits(
    const JointLimits& application_limits, const JointLimits& system_limits) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto limits_consistent_check_result,
      IsWithinLimits(application_limits, system_limits));
  if (!limits_consistent_check_result) {
    return InvalidArgumentError(
        "The system and application limits are inconsistent");
  }
  application_limits_ = application_limits;
  system_limits_ = system_limits;
  return OkStatus();
}

}  // namespace intrinsic::icon
