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

#include "intrinsic/icon/control/algorithms/cartesian_admittance_controller.h"

#include <cstddef>
#include <memory>
#include <optional>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/manifolds.h"
#include "intrinsic/eigenmath/pseudo_inverse.h"
#include "intrinsic/eigenmath/so3.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/settling_time_counter.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/check_cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pluecker_transform.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

using cartesian_impedance::AlgorithmConfiguration;
using cartesian_impedance::ApplyDeadband;
using cartesian_impedance::RealTimeCartesianTarget;
using cartesian_impedance::RealTimeConstraints;
using cartesian_impedance::RealTimeNullspaceTarget;
using cartesian_impedance::RotateCartesianVector;
using cartesian_impedance::TransformTaskMatrix;

namespace {

icon::RealtimeStatus IntegrateSymplecticEuler(
    double frequency_hz, const RealTimeConstraints& constraints,
    JointStatePVA& joint_space_motion_reference) {
  // Saturate the joint acceleration command in a way that preserves the
  // Cartesian motion direction: avoid naive clamping, scale instead.
  const double acc_overshoot_factor =
      (joint_space_motion_reference.acceleration.cwiseAbs().cwiseQuotient(
           constraints.joint_limits.max_acceleration.cwiseAbs()))
          .maxCoeff();
  if (acc_overshoot_factor > 1.0) {
    joint_space_motion_reference.acceleration /= acc_overshoot_factor;
  }

  // Forward-integrate the motion reference using symplectic integration.
  joint_space_motion_reference.velocity +=
      joint_space_motion_reference.acceleration / frequency_hz;

  // Saturate the joint velocity command in a way that preserves the
  // Cartesian motion direction: avoid naive clamping, scale instead.
  const double vel_overshoot_factor =
      (joint_space_motion_reference.velocity.cwiseAbs().cwiseQuotient(
           constraints.joint_limits.max_velocity.cwiseAbs()))
          .maxCoeff();
  if (vel_overshoot_factor > 1.0) {
    joint_space_motion_reference.velocity /= vel_overshoot_factor;
  }

  joint_space_motion_reference.position +=
      joint_space_motion_reference.velocity / frequency_hz;

  // Saturate the joint position command.
  if (!eigenmath::ClampVector(constraints.joint_limits.min_position,
                              constraints.joint_limits.max_position,
                              joint_space_motion_reference.position)) {
    return InternalError("Clamping joint positions failed.");
  }

  return OkStatus();
}

}  // namespace

absl::StatusOr<std::unique_ptr<CartesianAdmittanceController>>
CartesianAdmittanceController::Create(size_t njoints, double frequency_hz) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (njoints < 1) {
    return absl::FailedPreconditionError("Number of DoF must be > 0.");
  }
  if (frequency_hz < cartesian_impedance::kNumericPrecision) {
    return absl::FailedPreconditionError("Controller frequency must be > 0.");
  }

  // Using `new` to access a non-public constructor.
  return absl::WrapUnique(
      new CartesianAdmittanceController(njoints, frequency_hz));
}

CartesianAdmittanceController::CartesianAdmittanceController(
    size_t njoints, double frequency_hz)
    : njoints_(njoints),
      frequency_hz_(frequency_hz),
      jacobian_(eigenmath::Matrix6Nd::Zero(6, njoints_)),
      settling_time_counter_(frequency_hz) {
  CHECK_OK(joint_space_motion_reference_.SetSize(njoints_));
}

void CartesianAdmittanceController::SetCartesianTarget(
    const RealTimeCartesianTarget& target) {
  cartesian_reference_ = target;

  // Reset the error state to zero, which corresponds to no motion (without
  // computing a different error first).
  error_state_in_base_frame_.pose.setZero();
  error_state_in_base_frame_.twist.setZero();
  error_state_in_base_frame_.wrench.setZero();
}

void CartesianAdmittanceController::SetNullspaceTarget(
    const RealTimeNullspaceTarget& target) {
  nullspace_reference_ = target;
}

void CartesianAdmittanceController::SetAlgorithmConfiguration(
    const AlgorithmConfiguration& config) {
  algorithm_configuration_ = config;
}

RealtimeStatus CartesianAdmittanceController::SetConstraints(
    const RealTimeConstraints& constraints) {
  if (constraints.joint_limits.max_velocity.array().isInf().any()) {
    return InvalidArgumentError(
        "joint_limits.max_velocity contains 'inf' value(s). Algorithm "
        "requires "
        "strictly bounded limits.");
  }
  if (constraints.joint_limits.max_acceleration.array().isInf().any()) {
    return InvalidArgumentError(
        "joint_limits.max_acceleration contains 'inf' value(s). Algorithm "
        "requires strictly bounded limits.");
  }
  constraints_ = constraints;
  return OkStatus();
}

RealtimeStatus CartesianAdmittanceController::ResetInternalState(
    const Pose3d& previously_commanded_base_t_tool,
    const Twist& previously_commanded_twist_in_base_frame,
    const Acceleration& previously_commanded_acceleration_in_base_frame,
    const JointStatePVA& previously_commanded_joint_state) {
  // Reset the error state to zero, which corresponds to no motion (without
  // computing a different error first).
  error_state_in_base_frame_.pose.setZero();
  error_state_in_base_frame_.twist.setZero();
  error_state_in_base_frame_.wrench.setZero();

  // Set joint and Cartesian nominal states based on previously commanded
  // state.
  joint_space_motion_reference_ = previously_commanded_joint_state;
  base_t_tool_nominal_ = previously_commanded_base_t_tool;
  nominal_twist_in_base_frame_ = previously_commanded_twist_in_base_frame;

  cartesian_acceleration_command_in_base_frame_.setZero();
  jacobian_.reset();
  jacobian_pinv_.reset();
  sensed_wrench_at_tool_in_task_frame_.setZero();
  settling_time_counter_.Reset();

  return OkStatus();
}

RealtimeStatus CartesianAdmittanceController::PrepareControl(
    kinematics::ElementId tip_id, const Wrench& sensed_wrench_in_tip_frame,
    const Wrench& post_sensor_dynamic_load_in_tip_frame,
    kinematics::State& state) {
  if (!cartesian_reference_.has_value()) {
    return FailedPreconditionError(
        "No cartesian reference, did you call SetCartesianTarget()?");
  }
  if (!nullspace_reference_.has_value() && njoints_ > 6) {
    return FailedPreconditionError(
        "Have redundant robot but no nullspace reference, did you call "
        "SetNullspaceTarget()?");
  }
  if (!constraints_.has_value()) {
    return FailedPreconditionError(
        "No constraints set, did you call SetConstraints()?");
  }

  // Evaluate the endeffector forward kinematics for the reference motion. All
  // resulting quantities are expressed in the robot's base frame.
  INTRINSIC_RT_RETURN_IF_ERROR(
      state.SetStatePVA(joint_space_motion_reference_));

  INTRINSIC_RT_ASSIGN_OR_RETURN(Pose3d base_t_tip_nominal,
                                state.GetTransform(tip_id));

  // The nominal target is the current nominal tip position plus the
  // supplementary tip-to-target transform.
  base_t_tool_nominal_ =
      base_t_tip_nominal * cartesian_reference_->robot_tip_t_robot_tool;

  const Pose3d task_t_tool_nominal =
      cartesian_reference_->robot_base_t_task.inverse() * base_t_tool_nominal_;

  // Compute the Jacobian, the corresponding damped Jacobian pseudo-inverse, and
  // the time-derivative of the pseudo-inverse.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      jacobian_,
      state.ComputeJacobian(
          tip_id, cartesian_reference_->robot_tip_t_robot_tool.translation()));

  // In order to be able to compute the time derivative of the Jacobian
  // pseudo-inverse numerically, either need to cache the Jacobian
  // pseudo-inverse ...
  eigenmath::MatrixNMd jacobian_pinv_previous_cycle;
  if (jacobian_pinv_.has_value()) {
    jacobian_pinv_previous_cycle = *jacobian_pinv_;
  } else {
    // ... or approximate it based on back-stepping of the joint-space motion
    // reference.
    JointStateP backstepped_joint_position(
        joint_space_motion_reference_.position);
    backstepped_joint_position.position -=
        joint_space_motion_reference_.velocity / frequency_hz_;
    INTRINSIC_RT_RETURN_IF_ERROR(
        state.SetDofPositions(backstepped_joint_position));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto jacobian_backstepped,
        state.ComputeJacobian(
            tip_id,
            cartesian_reference_->robot_tip_t_robot_tool.translation()));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        jacobian_pinv_previous_cycle,
        ComputePseudoInverse(jacobian_backstepped,
                             algorithm_configuration_.jacobian_pinv_damping,
                             /*fail_if_badly_conditioned=*/true));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      jacobian_pinv_,
      ComputePseudoInverse(*jacobian_,
                           algorithm_configuration_.jacobian_pinv_damping,
                           /*fail_if_badly_conditioned=*/true));
  // Simple first order finite-difference of Jacobian pseudo inverse wrt time.
  jacobian_pinv_time_derivative_ =
      (*jacobian_pinv_ - jacobian_pinv_previous_cycle) * frequency_hz_;

  // Transform the desired wrench expressed in the task frame to the tool frame.
  Wrench goal_wrench_in_tool_frame = RotateCartesianVector(
      base_t_tool_nominal_.quaternion().inverse() *
          cartesian_reference_->robot_base_t_task.quaternion(),
      cartesian_reference_->tool_reference_wrench);

  // Express the desired wrench in the tip frame using a force transform. This
  // accounts for "lever arms" introduced by the virtual target pose offset
  // and represents the wrench that we control for at the tip.
  eigenmath::Matrix6d tip_F_tool =
      PlueckerForceTransform(cartesian_reference_->robot_tip_t_robot_tool);
  Wrench goal_wrench_in_tip_frame(tip_F_tool * goal_wrench_in_tool_frame);

  // Compute error w.r.t. desired wrench. We express the wrench error in the
  // tip frame first, because the deadband wrench is specified in the tip
  // frame. After deadband subtraction, the wrench error is transformed into
  // the robot base frame, where the control law is formulated. By ICON
  // convention, the wrenches represent the forces that the robot applies to
  // the environment. We negate the forces to obtain the forces that the
  // environment applies to the virtual mass.
  Wrench wrench_error_in_tip_frame(
      -(sensed_wrench_in_tip_frame - goal_wrench_in_tip_frame));

  // Apply the deadband on the wrench error in the tip frame.
  Wrench filtered_wrench_error_in_tip_frame(wrench_error_in_tip_frame);
  INTRINSIC_RT_RETURN_IF_ERROR(
      ApplyDeadband(algorithm_configuration_.sensed_wrench_deadband,
                    filtered_wrench_error_in_tip_frame));
  // Express the wrench error in the tool frame (account for lever arm
  // caused by offset pose).
  eigenmath::Matrix6d tool_F_tip = PlueckerForceTransformInverse(
      cartesian_reference_->robot_tip_t_robot_tool);
  Wrench filtered_wrench_error_in_tool_frame(
      tool_F_tip * filtered_wrench_error_in_tip_frame);

  // Express the filtered wrench error in the base frame, since the overall
  // control law is formulated in the base frame.
  // TODO(b/213165528): consider using the currently *measured*
  // base_t_tool transform, wherever sensed wrenches are transformed!
  error_state_in_base_frame_.wrench = RotateCartesianVector(
      base_t_tool_nominal_.quaternion(), filtered_wrench_error_in_tool_frame);

  sensed_wrench_at_tool_in_task_frame_ =
      RotateCartesianVector(task_t_tool_nominal.quaternion(),
                            Wrench(tool_F_tip * sensed_wrench_in_tip_frame));
  post_sensor_dynamic_load_in_task_frame_ = RotateCartesianVector(
      task_t_tool_nominal.quaternion(),
      Wrench(tool_F_tip * post_sensor_dynamic_load_in_tip_frame));

  // Due to ICON sign conventions, the post-sensor dynamic load must be
  // added to the error wrench.
  error_state_in_base_frame_.wrench +=
      algorithm_configuration_.post_sensor_dynamic_load_multiplier *
      post_sensor_dynamic_load_in_task_frame_;

  // Compute pose error between current motion reference (=currently commanded
  // pose) and desired tool pose, expressed in base frame.
  const Pose3d base_t_tool_desired =
      cartesian_reference_->robot_base_t_task *
      Pose3d(cartesian_reference_->task_t_tool_reference_orientation,
             cartesian_reference_->task_t_tool_reference_position);
  error_state_in_base_frame_.pose.head<3>() =
      base_t_tool_nominal_.translation() - base_t_tool_desired.translation();
  // Orientation error in nominal (current) tool pose frame:
  const eigenmath::SO3d tool_nominal_q_tool_reference(
      base_t_tool_nominal_.quaternion().inverse() *
      base_t_tool_desired.quaternion());
  error_state_in_base_frame_.pose.tail<3>() =
      -base_t_tool_nominal_.rotationMatrix() *
      eigenmath::logSO3(tool_nominal_q_tool_reference);

  // Compute twist error between motion reference and desired endeffector
  // twist, expressed in base frame.
  nominal_twist_in_base_frame_ =
      *jacobian_ * joint_space_motion_reference_.velocity;
  error_state_in_base_frame_.twist =
      nominal_twist_in_base_frame_ -
      RotateCartesianVector(
          cartesian_reference_->robot_base_t_task.quaternion(),
          cartesian_reference_->tool_reference_twist);

  // Check nominal Cartesian controller state vs Cartesian limits.
  CartStatePV statepv_to_be_checked;
  statepv_to_be_checked.pose = base_t_tool_nominal_;
  statepv_to_be_checked.velocity = nominal_twist_in_base_frame_;
  if (!IsWithinLimits(static_cast<CartStateP>(statepv_to_be_checked),
                      constraints_->cart_limits)) {
    return icon::InternalError(
        "base_t_target violates Cartesian position box limits in this "
        "control cycle.");
  }

  if (!IsWithinLimits(statepv_to_be_checked, constraints_->cart_limits)) {
    return icon::InternalError(
        "nominal_twist_in_base_frame violates Cartesian velocity box limits "
        "in this control cycle.");
  }

  return OkStatus();
}

RealtimeStatusOr<JointStatePVA> CartesianAdmittanceController::ComputeControl(
    const kinematics::InverseKinematicsInterface* /*ik*/,
    const Wrench& /*sensed_wrench_in_tip_frame*/) {
  if (!cartesian_reference_.has_value()) {
    return FailedPreconditionError(
        "No cartesian reference, did you call SetCartesianTarget()?");
  }
  if (!nullspace_reference_.has_value() && njoints_ > 6) {
    return FailedPreconditionError(
        "Have redundant robot but no nullspace reference, did you call "
        "SetNullspaceTarget()?");
  }
  if (!constraints_.has_value()) {
    return FailedPreconditionError(
        "No constraints set, did you call SetConstraints()?");
  }

  // Solve virtual dynamics model for Cartesian acceleration, formulated in
  // robot base frame.
  cartesian_acceleration_command_in_base_frame_ =
      RotateCartesianVector(
          cartesian_reference_->robot_base_t_task.quaternion(),
          cartesian_reference_->tool_reference_acceleration) +
      -TransformTaskMatrix(
          cartesian_reference_->robot_base_t_task,
          cartesian_reference_->virtual_cartesian_inertia_inverse) *
          (TransformTaskMatrix(cartesian_reference_->robot_base_t_task,
                               cartesian_reference_->cartesian_stiffness) *
               error_state_in_base_frame_.pose +
           TransformTaskMatrix(cartesian_reference_->robot_base_t_task,
                               cartesian_reference_->cartesian_damping) *
               error_state_in_base_frame_.twist -
           TransformTaskMatrix(cartesian_reference_->robot_base_t_task,
                               cartesian_reference_->wrench_selection_matrix) *
               error_state_in_base_frame_.wrench);

  // We reconstruct the Cartesian velocity command from the previous control
  // cycle from the joint space motion reference velocity.
  eigenmath::Vector6d previous_cartesian_velocity_command_ =
      *jacobian_ * joint_space_motion_reference_.velocity;
  // The new Cartesian velocity command.
  eigenmath::Vector6d cartesian_velocity_command =
      previous_cartesian_velocity_command_ +
      cartesian_acceleration_command_in_base_frame_ / frequency_hz_;

  // Map Cartesian acceleration into joint-space using Jacobian pseudo-inverse
  // and Jacobian time derivative.
  joint_space_motion_reference_.acceleration =
      *jacobian_pinv_ * cartesian_acceleration_command_in_base_frame_ +
      jacobian_pinv_time_derivative_ * cartesian_velocity_command;

  // Joint nullspace PD control law for redundant manipulators.
  if (njoints_ > 6 && nullspace_reference_.has_value()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        eigenmath::MatrixNMd jacobian_transpose_pinv,
        ComputePseudoInverse(jacobian_->transpose(),
                             algorithm_configuration_.jacobian_pinv_damping,
                             /*fail_if_badly_conditioned=*/true));
    // Note that this is generally not equal to I-jac_pinv*J.
    eigenmath::MatrixNMd nullspace_projector =
        eigenmath::MatrixNMd::Identity(njoints_, njoints_) -
        jacobian_->transpose() * jacobian_transpose_pinv;

    // Compute nullspace-invariant feedback.
    eigenmath::VectorNd qdd_desired_nullspace =
        nullspace_projector *
        (nullspace_reference_->joint_state.acceleration +
         nullspace_reference_->nullspace_stiffness *
             (joint_space_motion_reference_.position -
              nullspace_reference_->joint_state.position) +
         nullspace_reference_->nullspace_damping *
             joint_space_motion_reference_.velocity -
         nullspace_reference_->joint_state.velocity);

    joint_space_motion_reference_.acceleration -= qdd_desired_nullspace;
  }

  // Forward integration of joint accelerations. Important: the integration
  // scheme must enforce the constraints. We do generally not check if the
  // supplied input references are smooth or "slow" enough for the computed
  // control signal to stay within bounds. In fact, input references might be
  // arbitrarily discontinuous compared to a previously commanded state from a
  // different action - joint-space constraint satisfaction is done via
  // constrained integration.
  INTRINSIC_RT_RETURN_IF_ERROR(IntegrateSymplecticEuler(
      frequency_hz_, *constraints_, joint_space_motion_reference_));

  // An explicit constructor call is required since JointStatePVA is not
  // trivially copy-constructible.
  return JointStatePVA(joint_space_motion_reference_);
}

double CartesianAdmittanceController::UpdateSettlingTime(
    const JointStateV& joint_velocity,
    double translational_velocity_error_threshold,
    double angular_velocity_error_threshold, double joint_velocity_threshold) {
  return settling_time_counter_.ComputeSettlingTime(
      translational_velocity_error_threshold, angular_velocity_error_threshold,
      joint_velocity_threshold, joint_velocity,
      Twist(error_state_in_base_frame_.twist));
}

double CartesianAdmittanceController::SensedForceMagnitude() const {
  return sensed_wrench_at_tool_in_task_frame_.head<3>().norm();
}

double CartesianAdmittanceController::SensedTorqueMagnitude() const {
  return sensed_wrench_at_tool_in_task_frame_.tail<3>().norm();
}

Pose3d CartesianAdmittanceController::GetRobotTaskToTool() const {
  return cartesian_reference_->robot_base_t_task.inverse() *
         base_t_tool_nominal_;
}

Pose3d CartesianAdmittanceController::GetRobotTaskToTip() const {
  return cartesian_reference_->robot_base_t_task.inverse() *
         base_t_tool_nominal_ *
         cartesian_reference_->robot_tip_t_robot_tool.inverse();
}

Twist CartesianAdmittanceController::GetTwist() const {
  return nominal_twist_in_base_frame_;
}

RealtimeStatusOr<eigenmath::Matrix6Nd>
CartesianAdmittanceController::GetJacobian() const {
  if (jacobian_.has_value()) {
    return eigenmath::Matrix6Nd(*jacobian_);
  }
  return icon::InternalError("No Jacobian set.");
}

const std::optional<RealTimeCartesianTarget>&
CartesianAdmittanceController::GetCartesianTarget() const {
  return cartesian_reference_;
}

const std::optional<RealTimeNullspaceTarget>&
CartesianAdmittanceController::GetNullspaceTarget() const {
  return nullspace_reference_;
}

cartesian_impedance::CartesianErrorState
CartesianAdmittanceController::GetCartesianErrorState() const {
  return error_state_in_base_frame_;
}

}  // namespace intrinsic::icon
