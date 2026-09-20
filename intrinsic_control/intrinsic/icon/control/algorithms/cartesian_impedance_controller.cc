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

#include "intrinsic/icon/control/algorithms/cartesian_impedance_controller.h"

#include <memory>
#include <optional>

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
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pluecker_transform.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

using cartesian_impedance::ApplyDeadband;
using cartesian_impedance::RealTimeCartesianTarget;
using cartesian_impedance::RealTimeNullspaceTarget;
using cartesian_impedance::RotateCartesianVector;
using cartesian_impedance::TransformTaskMatrix;

absl::StatusOr<std::unique_ptr<CartesianImpedanceController>>
CartesianImpedanceController::Create(
    const kinematics::ModelInterface& kinematics_model, double frequency_hz) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (kinematics_model.GetNumberDegreesOfFreedom() < 1) {
    return absl::FailedPreconditionError("Number of DoF must be > 0.");
  }
  if (frequency_hz < cartesian_impedance::kNumericPrecision) {
    return absl::FailedPreconditionError("Controller frequency must be > 0.");
  }

  INTR_ASSIGN_OR_RETURN(auto tip,
                        kinematics_model.FindNonBranchingKinematicChainTip());

  // Using `new` to access a non-public constructor.
  return absl::WrapUnique(
      new CartesianImpedanceController(kinematics_model, tip, frequency_hz));
}

CartesianImpedanceController::CartesianImpedanceController(
    const kinematics::ModelInterface& kinematics_model,
    kinematics::ElementId tip_frame_id, double frequency_hz)
    : kinematics_state_(&kinematics_model),
      tip_frame_id_(tip_frame_id),
      njoints_(kinematics_model.GetNumberDegreesOfFreedom()),
      frequency_hz_(frequency_hz),
      jacobian_(eigenmath::Matrix6Nd::Zero(6, njoints_)),
      settling_time_counter_(frequency_hz) {}

void CartesianImpedanceController::SetCartesianTarget(
    const cartesian_impedance::RealTimeCartesianTarget& target) {
  cartesian_reference_ = target;
}

void CartesianImpedanceController::SetNullspaceTarget(
    const cartesian_impedance::RealTimeNullspaceTarget& target) {
  nullspace_reference_ = target;
}

void CartesianImpedanceController::SetAlgorithmConfiguration(
    const cartesian_impedance::AlgorithmConfiguration& config) {
  algorithm_configuration_ = config;
}

void CartesianImpedanceController::SetConstraints(
    const cartesian_impedance::RealTimeConstraints& constraints) {
  constraints_ = constraints;
}

void CartesianImpedanceController::ResetInternalState() {
  jacobian_.reset();
  jacobian_pinv_.reset();
  sensed_wrench_in_base_frame_.setZero();
  cartesian_acceleration_command_.setZero();
  base_t_tool_sensed_ = Pose3d::Identity();
  error_state_in_base_frame_.Reset();
  settling_time_counter_.Reset();
}

RealtimeStatus CartesianImpedanceController::PrepareControl(
    const JointStatePVA& joint_state, const Wrench& sensed_wrench_in_tip_frame,
    const Wrench& post_sensor_dynamic_load_in_tip_frame) {
  if (!cartesian_reference_.has_value()) {
    return FailedPreconditionError(
        "No cartesian reference, did you call SetCartesianTarget()?");
  }
  if (!nullspace_reference_.has_value() && njoints_ > 6) {
    return FailedPreconditionError(
        "Redundant robot but no nullspace reference, did you call "
        "SetNullspaceTarget()?");
  }
  if (!constraints_.has_value()) {
    return FailedPreconditionError(
        "No constraints set, did you call SetConstraints()?");
  }

  // In order to be able to compute the time derivative of the Jacobian
  // pseudo-inverse numerically, either need to cache the Jacobian
  // pseudo-inverse ...
  eigenmath::Matrix6Nd jacobian_previous_cycle;
  if (jacobian_.has_value()) {
    jacobian_previous_cycle = *jacobian_;
  } else {
    // ... or approximate it based on back-stepping of the joint-space motion
    // reference.
    JointStateP backstepped_joint_position(joint_state.position);
    backstepped_joint_position.position -= joint_state.velocity / frequency_hz_;
    INTRINSIC_RT_RETURN_IF_ERROR(
        kinematics_state_.SetDofPositions(backstepped_joint_position));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        jacobian_previous_cycle,
        kinematics_state_.ComputeJacobian(
            tip_frame_id_,
            cartesian_reference_->robot_tip_t_robot_tool.translation()));
  }

  // Evaluate the end-effector forward kinematics for the current joint state.
  // All resulting quantities are expressed in the robot's base frame.
  INTRINSIC_RT_RETURN_IF_ERROR(kinematics_state_.SetStatePVA(joint_state));

  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d base_t_tip,
                                kinematics_state_.GetTransform(tip_frame_id_));

  sensed_wrench_in_base_frame_ = RotateCartesianVector(
      base_t_tip.quaternion(), sensed_wrench_in_tip_frame);

  // The nominal target is the current nominal tip position plus the
  // supplementary tip-to-target transform.
  base_t_tool_sensed_ =
      base_t_tip * cartesian_reference_->robot_tip_t_robot_tool;

  const Pose3d task_t_tool_nominal =
      cartesian_reference_->robot_base_t_task.inverse() * base_t_tool_sensed_;

  // Compute the Jacobian, the corresponding damped Jacobian pseudo-inverse, and
  // the time-derivative of the pseudo-inverse.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      jacobian_,
      kinematics_state_.ComputeJacobian(
          tip_frame_id_,
          cartesian_reference_->robot_tip_t_robot_tool.translation()));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      jacobian_pinv_,
      ComputePseudoInverse(*jacobian_,
                           algorithm_configuration_.jacobian_pinv_damping,
                           /*fail_if_badly_conditioned=*/true));

  // Simple first order finite-difference of Jacobian pseudo inverse wrt time.
  jacobian_time_derivative_ =
      (*jacobian_ - jacobian_previous_cycle) * frequency_hz_;

  // Transform the desired wrench expressed in the task frame to the tool frame.
  const Wrench goal_wrench_in_tool_frame = RotateCartesianVector(
      base_t_tool_sensed_.quaternion().inverse() *
          cartesian_reference_->robot_base_t_task.quaternion(),
      cartesian_reference_->tool_reference_wrench);

  // Express the desired wrench in the tip frame using a force transform. This
  // accounts for "lever arms" introduced by the virtual target pose offset
  // and represents the wrench that we control for at the tip.
  const eigenmath::Matrix6d tip_F_tool =
      PlueckerForceTransform(cartesian_reference_->robot_tip_t_robot_tool);
  const Wrench goal_wrench_in_tip_frame(tip_F_tool * goal_wrench_in_tool_frame);

  // Compute error w.r.t. desired wrench. We express the wrench error in the
  // tip frame first, because the deadband wrench is specified in the tip
  // frame. After deadband subtraction, the wrench error is transformed into
  // the robot base frame, where the control law is formulated. By ICON
  // convention, the wrenches represent the forces that the robot applies to
  // the environment. We negate the forces to obtain the forces that the
  // environment applies to the virtual mass.
  const Wrench wrench_error_in_tip_frame(
      -(sensed_wrench_in_tip_frame - goal_wrench_in_tip_frame));

  // Apply the deadband on the wrench error in the tip frame.
  Wrench filtered_wrench_error_in_tip_frame(wrench_error_in_tip_frame);
  INTRINSIC_RT_RETURN_IF_ERROR(
      ApplyDeadband(algorithm_configuration_.sensed_wrench_deadband,
                    filtered_wrench_error_in_tip_frame));
  // Express the wrench error in the tool frame (account for lever arm
  // caused by offset pose).
  const eigenmath::Matrix6d tool_F_tip = PlueckerForceTransformInverse(
      cartesian_reference_->robot_tip_t_robot_tool);
  const Wrench filtered_wrench_error_in_tool_frame(
      tool_F_tip * filtered_wrench_error_in_tip_frame);

  // Express the filtered wrench error in the base frame, since the overall
  // control law is formulated in the base frame.
  error_state_in_base_frame_.wrench = RotateCartesianVector(
      base_t_tool_sensed_.quaternion(), filtered_wrench_error_in_tool_frame);

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
      RotateCartesianVector(
          cartesian_reference_->robot_base_t_task.quaternion(),
          post_sensor_dynamic_load_in_task_frame_);

  // Compute pose error between the sensed and reference tool poses.
  const Pose3d base_t_tool_desired =
      cartesian_reference_->robot_base_t_task *
      Pose3d(cartesian_reference_->task_t_tool_reference_orientation,
             cartesian_reference_->task_t_tool_reference_position);
  error_state_in_base_frame_.pose.head<3>() =
      base_t_tool_sensed_.translation() - base_t_tool_desired.translation();
  // Orientation error in nominal (current) tool pose frame:
  const eigenmath::SO3d tool_nominal_q_tool_reference(
      base_t_tool_sensed_.quaternion().inverse() *
      base_t_tool_desired.quaternion());
  error_state_in_base_frame_.pose.tail<3>() =
      -base_t_tool_sensed_.rotationMatrix() *
      eigenmath::logSO3(tool_nominal_q_tool_reference);

  // Update integrated pose error and saturate it according to user-defined
  // bounds.
  error_state_in_base_frame_.pose_integrated +=
      error_state_in_base_frame_.pose / frequency_hz_;
  if (!eigenmath::ClampVector(
          -algorithm_configuration_.pose_error_integrator_bound,
          algorithm_configuration_.pose_error_integrator_bound,
          error_state_in_base_frame_.pose_integrated)) {
    return InternalError("Clamping integrated pose error failed.");
  }

  // Compute twist error between measured and desired endeffector twist,
  // expressed in base frame.
  sensed_twist_in_base_frame_ = *jacobian_ * joint_state.velocity;
  error_state_in_base_frame_.twist =
      sensed_twist_in_base_frame_ -
      RotateCartesianVector(
          cartesian_reference_->robot_base_t_task.quaternion(),
          cartesian_reference_->tool_reference_twist);

  return OkStatus();
}

RealtimeStatusOr<JointStateA> CartesianImpedanceController::ComputeControl(
    const JointStatePVA& joint_state) {
  if (!cartesian_reference_.has_value()) {
    return FailedPreconditionError(
        "No cartesian reference, did you call SetCartesianTarget()?");
  }
  if (!nullspace_reference_.has_value() && njoints_ > 6) {
    return FailedPreconditionError(
        "Redundant robot but no nullspace reference, did you call "
        "SetNullspaceTarget()?");
  }
  if (!constraints_.has_value()) {
    return FailedPreconditionError(
        "No constraints set, did you call SetConstraints()?");
  }

  // Solve virtual dynamics model for cartesian acceleration. This is in the
  // base frame.
  cartesian_acceleration_command_ =
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

  eigenmath::VectorNd joint_acceleration_command =
      *jacobian_pinv_ * (cartesian_acceleration_command_ -
                         jacobian_time_derivative_ * joint_state.velocity);

  // Joint nullspace PD control law for redundant manipulators.
  if (njoints_ > 6) {
    eigenmath::MatrixNMd nullspace_projector =
        eigenmath::MatrixNMd::Identity(njoints_, njoints_) -
        (*jacobian_pinv_) * (*jacobian_);
    // Compute nullspace-invariant feedback.
    joint_acceleration_command -=
        nullspace_projector *
        (nullspace_reference_->joint_state.acceleration +
         nullspace_reference_->nullspace_stiffness *
             (joint_state.position -
              nullspace_reference_->joint_state.position) +
         nullspace_reference_->nullspace_damping *
             (joint_state.velocity -
              nullspace_reference_->joint_state.velocity));
  }

  // Saturate the joint acceleration command.
  if (!eigenmath::ClampVector(-constraints_->joint_limits.max_acceleration,
                              constraints_->joint_limits.max_acceleration,
                              joint_acceleration_command)) {
    return InternalError("Clamping joint accelerations failed.");
  }

  return JointStateA(joint_acceleration_command);
}

RealtimeStatusOr<JointStateT> CartesianImpedanceController::ComputeControl(
    const JointStatePVA& joint_state, RigidBodyInterface& dynamics) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStateA joint_acceleration_command,
                                ComputeControl(joint_state));
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto tip_frame_id, dynamics.GetDefaultTip());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::VectorNd output_joint_torques,
      dynamics.ComputeInverseDynamics(
          joint_state.position, joint_state.velocity,
          joint_acceleration_command.acceleration,
          Wrench(sensed_wrench_in_base_frame_), tip_frame_id));

  return JointStateT(output_joint_torques);
}

double CartesianImpedanceController::UpdateSettlingTime(
    const JointStateV& joint_velocity,
    double translational_velocity_error_threshold,
    double angular_velocity_error_threshold, double joint_velocity_threshold) {
  return settling_time_counter_.ComputeSettlingTime(
      translational_velocity_error_threshold, angular_velocity_error_threshold,
      joint_velocity_threshold, joint_velocity,
      Twist(error_state_in_base_frame_.twist));
}

Wrench CartesianImpedanceController::SensedWrenchAtToolInTaskFrame() const {
  return sensed_wrench_at_tool_in_task_frame_;
}

Wrench CartesianImpedanceController::PostSensorDynamicLoadInTaskFrame() const {
  return post_sensor_dynamic_load_in_task_frame_;
}

double CartesianImpedanceController::SensedForceMagnitude() const {
  return sensed_wrench_at_tool_in_task_frame_.head<3>().norm();
}

double CartesianImpedanceController::SensedTorqueMagnitude() const {
  return sensed_wrench_at_tool_in_task_frame_.tail<3>().norm();
}

Pose3d CartesianImpedanceController::GetRobotTaskToTip() const {
  return cartesian_reference_->robot_base_t_task.inverse() *
         base_t_tool_sensed_ *
         cartesian_reference_->robot_tip_t_robot_tool.inverse();
}

Pose3d CartesianImpedanceController::GetRobotTaskToTool() const {
  return cartesian_reference_->robot_base_t_task.inverse() *
         base_t_tool_sensed_;
}

const Twist& CartesianImpedanceController::GetTwist() const {
  return sensed_twist_in_base_frame_;
}

const std::optional<cartesian_impedance::RealTimeCartesianTarget>&
CartesianImpedanceController::GetCartesianTarget() const {
  return cartesian_reference_;
}

const std::optional<cartesian_impedance::RealTimeNullspaceTarget>&
CartesianImpedanceController::GetNullspaceTarget() const {
  return nullspace_reference_;
}

cartesian_impedance::CartesianErrorState
CartesianImpedanceController::GetCartesianErrorState() const {
  return error_state_in_base_frame_;
}

eigenmath::Vector6d
CartesianImpedanceController::GetCartesianAccelerationCommand() const {
  return cartesian_acceleration_command_;
}

RealtimeStatusOr<eigenmath::Matrix6Nd>
CartesianImpedanceController::GetJacobian() const {
  if (!jacobian_.has_value()) {
    return FailedPreconditionError("Jacobian is not available.");
  }
  return eigenmath::Matrix6Nd(*jacobian_);
}

RealtimeStatusOr<eigenmath::MatrixNMd>
CartesianImpedanceController::GetJacobianPinv() const {
  if (!jacobian_pinv_.has_value()) {
    return FailedPreconditionError("Jacobian pseudo-inverse is not available.");
  }
  return eigenmath::MatrixNMd(*jacobian_pinv_);
}

RealtimeStatusOr<eigenmath::VectorNd>
CartesianImpedanceController::GetSensedWrenchJointTorques() const {
  if (!jacobian_.has_value()) {
    return FailedPreconditionError("Jacobian is not available.");
  }
  return eigenmath::VectorNd(
      -(jacobian_.value().transpose() * sensed_wrench_in_base_frame_));
};

}  // namespace intrinsic::icon
