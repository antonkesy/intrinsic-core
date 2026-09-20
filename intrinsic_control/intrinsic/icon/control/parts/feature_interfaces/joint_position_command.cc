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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_position_command.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/vector.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/collision/robot_collision_checker.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/move_checker.h"
#include "intrinsic/icon/control/parts/plane_checker.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

constexpr int kCyclesTillStale = 5;

// Convert a flatbuffer vector to a eigen VectorNd.
eigenmath::VectorNd ToEigenVector(
    const flatbuffers::Vector<double>* fbs_double_vector) {
  eigenmath::VectorNd ret;
  ret.resize(fbs_double_vector->size());
  for (int i = 0; i < fbs_double_vector->size(); ++i) {
    ret[i] = fbs_double_vector->Get(i);
  }

  return ret;
}

}  // namespace

absl::StatusOr<JointPositionCommandFeature> JointPositionCommandFeature::Create(
    JointPositionCommandHardwareInterface
        joint_position_command_hardware_interface,
    JointPositionStateHardwareInterface joint_position_state_hardware_interface,
    std::optional<ActiveCommandHardwareInterface>
        active_command_hardware_interface,
    double control_frequency_hz,
    const JointLimitsInterface* joint_limits_interface,
    const std::optional<CartesianLimits> cartesian_limits,
    const ManipulatorKinematics* manipulator_kinematics_interface,
    std::unique_ptr<collision::RobotCollisionChecker> collision_checker) {
  if (*joint_position_command_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointPositionCommand is not "
        "initialized.");
  }
  if (*joint_position_state_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointPositionState is not "
        "initialized.");
  }
  INTR_ASSIGN_OR_RETURN(
      auto move_checker,
      MoveChecker::Create(
          joint_position_command_hardware_interface->position()->size(),
          control_frequency_hz, std::move(collision_checker)));

  std::unique_ptr<PlaneChecker> plane_checker;
  if (cartesian_limits.has_value() &&
      manipulator_kinematics_interface != nullptr) {
    INTR_ASSIGN_OR_RETURN(plane_checker,
                          PlaneChecker::Create(cartesian_limits.value()));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointPositionCommand position_setpoints,
      JointPositionCommand::Create(
          ToEigenVector(joint_position_command_hardware_interface->position()),
          eigenmath::VectorNd::Zero(
              joint_position_command_hardware_interface->position()->size()),
          eigenmath::VectorNd::Zero(
              joint_position_command_hardware_interface->position()->size())));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointPositionCommand previous_setpoints,
      JointPositionCommand::Create(
          ToEigenVector(joint_position_command_hardware_interface->position()),
          eigenmath::VectorNd::Zero(
              joint_position_command_hardware_interface->position()->size()),
          eigenmath::VectorNd::Zero(
              joint_position_command_hardware_interface->position()->size())));

  INTRINSIC_RT_RETURN_IF_ERROR(
      move_checker->UpdatePreviousSetpoint(previous_setpoints));

  return JointPositionCommandFeature(
      std::move(joint_position_command_hardware_interface),
      std::move(joint_position_state_hardware_interface),
      std::move(active_command_hardware_interface),
      std::move(position_setpoints), std::move(previous_setpoints),
      control_frequency_hz, joint_limits_interface,
      manipulator_kinematics_interface, std::move(move_checker),
      std::move(plane_checker));
}

JointPositionCommandFeature::JointPositionCommandFeature(
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
    std::unique_ptr<PlaneChecker> plane_checker)
    : joint_limits_interface_(joint_limits_interface),
      manipulator_kinematics_interface_(manipulator_kinematics_interface),
      position_setpoints_numdiff_(/*dt_sec=*/1.0 / control_frequency_hz),
      sensed_position_numdiff_(/*dt_sec=*/1.0 / control_frequency_hz),
      joint_position_command_hardware_interface_(
          std::move(joint_position_command_hardware_interface)),
      joint_position_state_hardware_interface_(
          std::move(joint_position_state_hardware_interface)),
      active_command_hardware_interface_(
          std::move(active_command_hardware_interface)),
      // See comment in Reset for why we initialize this way.
      cycles_since_last_update_(kCyclesTillStale + 1),
      previous_setpoints_(std::move(previous_setpoints)),
      move_checker_(std::move(move_checker)),
      plane_checker_(std::move(plane_checker)) {}

RealtimeStatus JointPositionCommandFeature::Reset() {
  position_setpoints_numdiff_.Reset();
  sensed_position_numdiff_.Reset();
  if (active_command_hardware_interface_.has_value()) {
    // We are setting previous_setpoints_ to a reasonable value, so it is safe
    // to immediately return it to the user.
    cycles_since_last_update_ = 0;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        previous_setpoints_,
        JointPositionCommand::Create(
            ToEigenVector(
                active_command_hardware_interface_.value()->position()),
            ToEigenVector(active_command_hardware_interface_.value()
                              ->velocity_feedforward()),
            ToEigenVector(active_command_hardware_interface_.value()
                              ->acceleration_feedforward())));

  } else {
    // We zero initialize previous_setpoints_ here to clear the values, but we
    // shouldn't return this to the user since a "0" value may not be within
    // limits for some joints.
    //
    // Instead, we set cycles_since_last_update_ so that
    // OverridePreviousSetpointsIfStale() immediately treats previous_setpoints_
    // as stale and updates previous_setpoints_ to the sensed position. This is
    // safer than zeros.
    cycles_since_last_update_ = kCyclesTillStale + 1;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        previous_setpoints_,
        JointPositionCommand::Create(
            ToEigenVector(joint_position_state_hardware_interface_->position()),
            eigenmath::VectorNd::Zero(
                joint_position_command_hardware_interface_->position()->size()),
            eigenmath::VectorNd::Zero(
                joint_position_command_hardware_interface_->position()
                    ->size())));
  }

  INTRINSIC_RT_RETURN_IF_ERROR(
      move_checker_->UpdatePreviousSetpoint(previous_setpoints_));

  for (int i = 0; i < previous_setpoints_.Size(); ++i) {
    joint_position_command_hardware_interface_->mutable_position()->Mutate(
        i, previous_setpoints_.position()[i]);
  }

  // Zero the velocity and acceleration feedforwards.
  for (int i = 0; i < previous_setpoints_.Size(); ++i) {
    joint_position_command_hardware_interface_->mutable_velocity_feedforward()
        ->Mutate(i, 0);
    joint_position_command_hardware_interface_
        ->mutable_acceleration_feedforward()
        ->Mutate(i, 0);
  }

  return OkStatus();
}

RealtimeStatus JointPositionCommandFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  position_setpoints_ = std::nullopt;
  updated_time_ = std::nullopt;
  cycles_since_last_update_++;

  // Update the sensed state.
  auto sensed_pos =
      ToEigenVector(joint_position_state_hardware_interface_->position());
  sensed_position_numdiff_.Append(sensed_pos);

  // Update the previous set points. Even though velocity and acceleration are
  // optional from an ICON Action's perspective, we always populate the
  // flatbuffer fields in ApplyStatus below, so these values are safe to read.
  auto eigen_pos =
      ToEigenVector(joint_position_command_hardware_interface_->position());
  auto eigen_vel = ToEigenVector(
      joint_position_command_hardware_interface_->velocity_feedforward());
  auto eigen_acc = ToEigenVector(
      joint_position_command_hardware_interface_->acceleration_feedforward());
  // TODO(b/230436992) Relax constness when updating a joint position command in
  // place.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      previous_setpoints_,
      JointPositionCommand::Create(eigen_pos, eigen_vel, eigen_acc));

  // We do this overwriting in ReadStatus to ensure that the caller has access
  // to reasonable values on the first call to SetPositionSetpoints or
  // PreviousPositionSetpoints after a break.
  INTRINSIC_RT_RETURN_IF_ERROR(OverridePreviousSetpointsIfStale());

  // Make sure this call stays after OverridePreviousSetpointsIfStale since that
  // may modify previous_setpoints_.
  return move_checker_->UpdatePreviousSetpoint(previous_setpoints_);
}

RealtimeStatus JointPositionCommandFeature::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters params) {
  // If position_setpoints is not set (i.e. SetPositionSetpoints was not
  // called), then we will update the numeric differentiator with the current
  // position of the robot.
  //
  // We do this because the numerical differentiator should be fed updates at a
  // consistent frequency and the sensed value is the best approximation of the
  // target position. This doesn't update the hardware interfaces themselves,
  // but ensures that the position_setpoints_numdiff_ is in a decent state for
  // querying.
  if (!position_setpoints_.has_value()) {
    eigenmath::VectorNd sensed_position =
        ToEigenVector(joint_position_state_hardware_interface_->position());
    position_setpoints_numdiff_.Append(sensed_position);
    return OkStatus();
  }

  if (!updated_time_.has_value()) {
    return InternalError(
        "No time of update even though a new position setpoint was passed. "
        "This is an ICON programming error.");
  }

  for (int i = 0; i < position_setpoints_.value().Size(); ++i) {
    joint_position_command_hardware_interface_->mutable_position()->Mutate(
        i, position_setpoints_.value().position()[i]);
  }
  joint_position_command_hardware_interface_.UpdatedAt(*updated_time_);

  position_setpoints_numdiff_.Append(position_setpoints_->position());
  JointLimits system_limits = joint_limits_interface_->GetSystemLimits();
  if (position_setpoints_.value().velocity_feedforward().has_value()) {
    for (int i = 0; i < position_setpoints_.value().Size(); ++i) {
      joint_position_command_hardware_interface_->mutable_velocity_feedforward()
          ->Mutate(
              i, position_setpoints_.value().velocity_feedforward().value()[i]);
    }
  } else {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto velocity_estimate,
        position_setpoints_numdiff_.FirstDerivativeBackward(kNumDiffOrder));
    if (!eigenmath::ClampVector(-system_limits.max_velocity,
                                system_limits.max_velocity,
                                velocity_estimate)) {
      return icon::InternalError(
          "Clamping the velocity feedforwards failed. This is an ICON "
          "programming error.");
    }
    for (int i = 0; i < position_setpoints_.value().Size(); ++i) {
      joint_position_command_hardware_interface_->mutable_velocity_feedforward()
          ->Mutate(i, velocity_estimate(i));
    }
  }
  if (position_setpoints_.value().acceleration_feedforward().has_value()) {
    for (int i = 0; i < position_setpoints_.value().Size(); ++i) {
      joint_position_command_hardware_interface_
          ->mutable_acceleration_feedforward()
          ->Mutate(i, position_setpoints_.value()
                          .acceleration_feedforward()
                          .value()[i]);
    }
  } else {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto acceleration_estimate,
        position_setpoints_numdiff_.SecondDerivativeBackward(kNumDiffOrder));
    if (!eigenmath::ClampVector(-system_limits.max_acceleration,
                                system_limits.max_acceleration,
                                acceleration_estimate)) {
      return icon::InternalError(
          "Clamping the acceleration feedforwards failed. This is an ICON "
          "programming error.");
    }
    for (int i = 0; i < position_setpoints_.value().Size(); ++i) {
      joint_position_command_hardware_interface_
          ->mutable_acceleration_feedforward()
          ->Mutate(i, acceleration_estimate(i));
    }
  }
  return icon::OkStatus();
}

RealtimeStatus JointPositionCommandFeature::SetPositionSetpoints(
    const JointPositionCommand& setpoints) {
  if (setpoints.Size() !=
      joint_position_command_hardware_interface_->position()->size()) {
    return icon::InvalidArgumentError(
        "Invalid setpoints: Wrong number of DoFs.");
  }
  // TODO(b/270674017): Limit check in HalArmPart should check against
  // application limits by default - and special case safety (and feedback
  // actions).
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const std::optional<
          JointLimitsInterface::JointAccelerationLimitsFromDynamics>
          joint_acceleration_limits_from_dynamics,
      joint_limits_interface_->GetJointAccelerationLimitsFromDynamics());
  INTRINSIC_RT_RETURN_IF_ERROR(move_checker_->CheckSetpoint(
      setpoints, joint_limits_interface_->GetSystemLimits(),
      joint_limits_interface_->GetSystemLimits(),
      joint_acceleration_limits_from_dynamics));

  if (plane_checker_ != nullptr &&
      manipulator_kinematics_interface_ != nullptr) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto tcp_pose_current,
        manipulator_kinematics_interface_->ComputeChainFK(
            setpoints.position()));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto tcp_pose_previous,
        manipulator_kinematics_interface_->ComputeChainFK(
            previous_setpoints_.position()));
    CartStateP end_effector_pos_current{tcp_pose_current};
    CartStateP end_effector_pos_previous{tcp_pose_previous};
    INTRINSIC_RT_RETURN_IF_ERROR(plane_checker_->CheckSetpoint(
        end_effector_pos_previous.pose, end_effector_pos_current.pose));
  }
  position_setpoints_ = setpoints;
  updated_time_ = Clock::now();
  cycles_since_last_update_ = 0;

  return OkStatus();
}

JointPositionCommand JointPositionCommandFeature::PreviousPositionSetpoints()
    const {
  return previous_setpoints_;
}

RealtimeStatus JointPositionCommandFeature::OverridePreviousSetpointsIfStale() {
  if (cycles_since_last_update_ > kCyclesTillStale) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto velocity_estimate,
        sensed_position_numdiff_.FirstDerivativeBackward(kNumDiffOrder));
    // We could set the acceleration with the estimate below, but we find that
    // in practice, there is enough noise in the position readings for some
    // robots (i.e. the franka) that the acceleration estimates produce a limit
    // violation. Setting it to zero here avoids that issue at the expense of
    // making the reference generated stop a bit more abrupt when switch control
    // modes (torque switching into position).
    // INTRINSIC_RT_ASSIGN_OR_RETURN(
    //     auto acceleration_estimate,
    //     sensed_position_numdiff_.SecondDerivativeBackward(kNumDiffOrder));
    eigenmath::VectorNd acceleration_estimate =
        eigenmath::VectorNd::Zero(velocity_estimate.size());

    INTRINSIC_RT_ASSIGN_OR_RETURN(
        previous_setpoints_,
        JointPositionCommand::Create(
            ToEigenVector(joint_position_state_hardware_interface_->position()),
            velocity_estimate, acceleration_estimate));
  }
  return OkStatus();
}

}  // namespace intrinsic::icon
