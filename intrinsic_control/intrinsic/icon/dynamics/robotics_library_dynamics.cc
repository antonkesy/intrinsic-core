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

#include "intrinsic/icon/dynamics/robotics_library_dynamics.h"

#include <memory>

#include "Eigen/Core"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/math/pluecker_transform.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"
#include "rl/math/Spatial.h"
#include "rl/math/Transform.h"
#include "rl/math/Vector.h"
#include "rl/mdl/Body.h"
#include "rl/mdl/Dynamic.h"

namespace intrinsic::icon {

namespace {

// Maps a motion vector from the local frame to the local world aligned frame.
// The local world aligned frame is defined as the frame with the same origin as
// the local frame, but with the same orientation as the base frame. In
// addition, it adds an offset between the local frame and the target frame.
eigenmath::Vector6d MotionVectorFromLocalToLocalWorldAligned(
    const ::rl::math::MotionVector& local,
    const eigenmath::Matrix3d& base_R_frame,
    const eigenmath::Vector3d& frame_to_target_position_offset) {
  return PlueckerMotionTransform(Pose3d(base_R_frame)) *
         PlueckerMotionTransformInverse(
             Pose3d(frame_to_target_position_offset)) *
         (eigenmath::Vector6d() << local.linear(), local.angular()).finished();
}

}  // namespace

icon::RealtimeStatusOr<kinematics::ElementId>
RoboticsLibraryDynamics::GetDefaultTip() const {
  if (model_interface_ == nullptr) {
    return icon::FailedPreconditionError("Model interface is null.");
  }
  return model_interface_->FindNonBranchingKinematicChainTip();
}

void RoboticsLibraryDynamics::SetGravity(const eigenmath::Vector3d& gravity) {
  // By ICON convention, the vector-valued gravitational acceleration is
  // expressed in the base frame. The 'Robotics Library' expects inverted
  // gravity vectors, therefore we negate the `gravity` vector provided above.
  // Example: if the robot base frame's z-axis is pointing upwards, the standard
  // gravitational acceleration vector will be (0,0,-9.81), but the 'Robotics
  // Library' expects (0,0,9.81).
  dynamic_->setWorldGravity(-gravity);
}

icon::RealtimeStatusOr<Pose3d>
RoboticsLibraryDynamics::ComputeForwardKinematics(
    const eigenmath::VectorNd& joint_coordinates,
    kinematics::ElementId frame_id) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof()");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto default_tip, GetDefaultTip());
  if (frame_id != default_tip) {
    return icon::InvalidArgumentError(
        "Non-default tip not supported in Robotics Library.");
  }

  dynamic_->setPosition(joint_coordinates);
  dynamic_->forwardPosition();
  const ::rl::math::Transform& pose =
      dynamic_->getOperationalPosition(/*frame index*/ 0);

  return Pose3d(pose.rotation(), pose.translation());
}

icon::RealtimeStatusOr<eigenmath::VectorNd>
RoboticsLibraryDynamics::ComputeInverseDynamics(
    const eigenmath::VectorNd& joint_coordinates,
    const eigenmath::VectorNd& joint_velocities,
    const eigenmath::VectorNd& joint_accelerations) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof()");
  }
  if (joint_velocities.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_velocities size != GetNumDof()");
  }
  if (joint_accelerations.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_accelerations size != GetNumDof()");
  }

  dynamic_->setPosition(joint_coordinates);
  dynamic_->setVelocity(joint_velocities);
  dynamic_->setAcceleration(joint_accelerations);
  dynamic_->inverseDynamics();

  return eigenmath::VectorNd(dynamic_->getTorque());
}

icon::RealtimeStatusOr<eigenmath::VectorNd>
RoboticsLibraryDynamics::ComputeInverseDynamics(
    const eigenmath::VectorNd& joint_coordinates,
    const eigenmath::VectorNd& joint_velocities,
    const eigenmath::VectorNd& joint_accelerations, const Wrench& wrench_at_tip,
    kinematics::ElementId frame_id) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::VectorNd joint_torques,
      ComputeInverseDynamics(joint_coordinates, joint_velocities,
                             joint_accelerations));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::Matrix6Nd jacobian,
      ComputeJacobian(
          joint_coordinates,
          /*frame_to_target_position_offset=*/eigenmath::Vector3d::Zero(),
          frame_id));

  return eigenmath::VectorNd(joint_torques -
                             jacobian.transpose() * wrench_at_tip);
}

icon::RealtimeStatusOr<eigenmath::VectorNd>
RoboticsLibraryDynamics::ComputeForwardDynamics(
    const eigenmath::VectorNd& joint_coordinates,
    const eigenmath::VectorNd& joint_velocities,
    const eigenmath::VectorNd& joint_torques) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof()");
  }
  if (joint_velocities.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_velocities size != GetNumDof()");
  }
  if (joint_torques.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_torques size != GetNumDof()");
  }

  dynamic_->setPosition(joint_coordinates);
  dynamic_->setVelocity(joint_velocities);
  dynamic_->setTorque(joint_torques);
  dynamic_->forwardDynamics();

  return eigenmath::VectorNd(dynamic_->getAcceleration());
}

icon::RealtimeStatusOr<eigenmath::MatrixNd>
RoboticsLibraryDynamics::ComputeJointSpaceInertiaMatrixInverse(
    const eigenmath::VectorNd& joint_coordinates) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof()");
  }

  dynamic_->setPosition(joint_coordinates);
  dynamic_->calculateMassMatrixInverse();

  return eigenmath::MatrixNd(dynamic_->getMassMatrixInverse());
}

icon::RealtimeStatusOr<eigenmath::Matrix6Nd>
RoboticsLibraryDynamics::ComputeJacobian(
    const eigenmath::VectorNd& joint_coordinates,
    const eigenmath::Vector3d& frame_to_target_position_offset,
    kinematics::ElementId frame_id) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof()");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto default_tip, GetDefaultTip());
  if (frame_id != default_tip) {
    return icon::InvalidArgumentError(
        "Non-default tip not supported in Robotics Library.");
  }

  dynamic_->setPosition(joint_coordinates);
  dynamic_->forwardPosition();
  dynamic_->calculateJacobian();

  eigenmath::Matrix6Nd jacobian_in_base_coordinates(dynamic_->getJacobian());
  // Account for the offset between the target frame and the frame described by
  // `frame_id`.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      Pose3d base_t_frame,
      ComputeForwardKinematics(joint_coordinates, frame_id));
  const eigenmath::Vector3d base_p_target =
      base_t_frame * frame_to_target_position_offset;

  for (int i = 0; i < jacobian_in_base_coordinates.cols(); ++i) {
    jacobian_in_base_coordinates.col(i).head<3>() +=
        (base_t_frame.translation() - base_p_target)
            .cross(jacobian_in_base_coordinates.col(i).tail<3>());
  }
  return jacobian_in_base_coordinates;
}

icon::RealtimeStatusOr<eigenmath::Matrix6Nd>
RoboticsLibraryDynamics::ComputeJacobianTimeDerivative(
    const eigenmath::VectorNd& joint_coordinates,
    const eigenmath::VectorNd& joint_velocity,
    const eigenmath::Vector3d& frame_to_target_position_offset,
    kinematics::ElementId frame_id) {
  return icon::UnimplementedError(
      "Jacobian time derivative computation not implemented.");
}

icon::RealtimeStatusOr<eigenmath::Vector6d>
RoboticsLibraryDynamics::ComputeJacobianTimeDerivativeVector(
    const eigenmath::VectorNd& joint_coordinates,
    const eigenmath::VectorNd& joint_velocity,
    const eigenmath::Vector3d& frame_to_target_position_offset,
    kinematics::ElementId frame_id) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof().");
  }
  if (joint_velocity.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_velocity size != GetNumDof().");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(const kinematics::ElementId default_tip,
                                GetDefaultTip());
  if (frame_id != default_tip) {
    return icon::InvalidArgumentError(
        "Non-default tip not supported in Robotics Library.");
  }

  // Store the current value of the gravity vector and for the purpose of these
  // calculations set it to zero.
  eigenmath::Vector3d gravity;
  dynamic_->getWorldGravity(gravity.x(), gravity.y(), gravity.z());
  dynamic_->setWorldGravity(0.0, 0.0, 0.0);

  // Compute forward kinematics with the given joint coordinates, joint velocity
  // and zero joint acceleration.
  dynamic_->setPosition(joint_coordinates);
  dynamic_->forwardPosition();
  const eigenmath::Matrix3d base_R_frame =
      dynamic_->getOperationalPosition(0).rotation();

  // Compute the spatial velocity/acceleration of the `default_tip` expressed at
  // the local frame and map it to the local frame whose axis are aligned with
  // the base frame. It also adds the effect of the
  // `frame_to_target_position_offset`.
  dynamic_->setVelocity(joint_velocity);
  dynamic_->forwardVelocity();
  const eigenmath::Vector6d spatial_vel_local_world_aligned_at_target =
      MotionVectorFromLocalToLocalWorldAligned(
          dynamic_->getOperationalVelocity(0), base_R_frame,
          frame_to_target_position_offset);

  dynamic_->setAcceleration(0.0 * joint_velocity);
  dynamic_->forwardAcceleration();
  const eigenmath::Vector6d spatial_acc_local_world_aligned_at_target =
      MotionVectorFromLocalToLocalWorldAligned(
          dynamic_->getOperationalAcceleration(0), base_R_frame,
          frame_to_target_position_offset);

  // Compute the classical acceleration out of the spatial acceleration and
  // spatial velocity.
  eigenmath::Vector6d classical_acc_local_world_aligned =
      spatial_acc_local_world_aligned_at_target;
  classical_acc_local_world_aligned.head<3>() +=
      spatial_vel_local_world_aligned_at_target.tail<3>().cross(
          spatial_vel_local_world_aligned_at_target.head<3>());

  // Set the gravity vector back to its original value.
  dynamic_->setWorldGravity(gravity.x(), gravity.y(), gravity.z());

  return classical_acc_local_world_aligned;
}

icon::RealtimeStatusOr<eigenmath::VectorNd>
RoboticsLibraryDynamics::ComputeGeneralizedGravityVector(
    const eigenmath::VectorNd& joint_coordinates) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof()");
  }

  dynamic_->setPosition(joint_coordinates);
  dynamic_->calculateGravity();

  return eigenmath::VectorNd(dynamic_->getGravity());
}

icon::RealtimeStatusOr<eigenmath::VectorNd>
RoboticsLibraryDynamics::ComputeCoriolisVector(
    const eigenmath::VectorNd& joint_coordinates,
    const eigenmath::VectorNd& joint_velocities) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof()");
  }
  if (joint_velocities.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_velocities size != GetNumDof()");
  }

  dynamic_->setPosition(joint_coordinates);
  dynamic_->setVelocity(joint_velocities);
  dynamic_->calculateCentrifugalCoriolis();

  return eigenmath::VectorNd(dynamic_->getCentrifugalCoriolis());
}

icon::RealtimeStatusOr<eigenmath::MatrixNd>
RoboticsLibraryDynamics::ComputeJointSpaceInertiaMatrix(
    const eigenmath::VectorNd& joint_coordinates) {
  if (joint_coordinates.size() != GetNumDof()) {
    return icon::InvalidArgumentError(
        "Argument joint_coordinates size != GetNumDof()");
  }

  dynamic_->setPosition(joint_coordinates);
  dynamic_->calculateMassMatrix();

  return eigenmath::MatrixNd(dynamic_->getMassMatrix());
}

icon::RealtimeStatus RoboticsLibraryDynamics::SetEndEffectorDynamicsParameters(
    const Pose3d& tip_t_cog, double mass,
    const eigenmath::Matrix3d& inertia_matrix, kinematics::ElementId frame_id) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto default_tip, GetDefaultTip());
  if (frame_id != default_tip) {
    return icon::InvalidArgumentError(
        "Non-default tip not supported in Robotics Library.");
  }

  // The end effector is the last Body for single tip chains.
  if (dynamic_->getBodies() < 1) {
    return icon::InvalidArgumentError(
        "underlying kinematic chain contains no bodies.");
  }
  ::rl::mdl::Body* tip = dynamic_->getBody(dynamic_->getBodies() - 1);

  // Transforms the inertia matrix (originally expressed at the frame located at
  // the body center of mass) in the tip frame.
  const eigenmath::Matrix3d rotation =
      tip_t_cog.quaternion().toRotationMatrix();
  const eigenmath::Matrix3d rotated_inertia_matrix =
      rotation * inertia_matrix * rotation.transpose();

  tip->setMass(mass);
  tip->setInertia(rotated_inertia_matrix(0, 0), rotated_inertia_matrix(1, 1),
                  rotated_inertia_matrix(2, 2), rotated_inertia_matrix(1, 2),
                  rotated_inertia_matrix(0, 2), rotated_inertia_matrix(0, 1));
  tip->setCenterOfMass(tip_t_cog.translation().x(), tip_t_cog.translation().y(),
                       tip_t_cog.translation().z());

  return icon::RealtimeStatus();
}

icon::RealtimeStatus RoboticsLibraryDynamics::SetEndEffectorDynamicsParameters(
    const eigenmath::Vector3d& tip_t_cog, double mass,
    kinematics::ElementId tip_id) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto default_tip, GetDefaultTip());
  if (tip_id != default_tip) {
    return icon::InvalidArgumentError(
        "Non-default tip not supported in Robotics Library.");
  }

  // The end effector is the last body for single tip chains.
  if (dynamic_->getBodies() < 1) {
    return icon::InvalidArgumentError(
        "Underlying kinematic chain contains no bodies.");
  }
  ::rl::mdl::Body* tip = dynamic_->getBody(dynamic_->getBodies() - 1);
  tip->setMass(mass);
  tip->setCenterOfMass(tip_t_cog.x(), tip_t_cog.y(), tip_t_cog.z());

  return icon::OkStatus();
}

}  // namespace intrinsic::icon
