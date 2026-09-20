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

#include "intrinsic/kinematics/state.h"

#include <optional>

#include "absl/log/check.h"
#include "intrinsic/eigenmath/skew_symmetric_matrix.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/analytic_jacobian_utils.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/state_values.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pluecker_transform.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic {
namespace kinematics {

namespace {

icon::RealtimeStatusOr<Pose3d> GetOutboundParentTransform(
    const ModelInterface& model, const ElementId& element_id,
    const JointStateP& dof_positions) {
  const auto joint_or = model.GetJoint(element_id);
  if (joint_or.ok()) {
    const auto* joint = joint_or.value();
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const auto derived_joint_value,
        model.ComputeJointsDerivedValue(element_id, dof_positions));
    return joint->GetJointOutboundTransform(derived_joint_value);
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* element,
                                model.GetElement(element_id));
  return element->GetParentTThis();
}

icon::RealtimeStatus CheckModelConsistency(const ModelInterface* model,
                                           int num_dof) {
  if (model->GetNumberDegreesOfFreedom() != num_dof) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        "Size of position values is ", num_dof,
        " which is different of size of number of dof in model (",
        model->GetNumberDegreesOfFreedom(), ")."));
  }
  return icon::OkStatus();
}

icon::RealtimeStatus CheckWithinLimits(double value, double lower,
                                       double upper) {
  if (value < lower || value > upper) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Value: ", value, " exceeds state limits [ ", lower, ",", upper, "]"));
  }
  return icon::OkStatus();
}

icon::RealtimeStatus CheckWithinLimits(double value, double upper) {
  if (value < -upper || value > upper) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Value: ", value, " exceeds state limits [ ", -upper, ",", upper, "]"));
  }
  return icon::OkStatus();
}

// Computes the Coriolis acceleration of a frame that is in motion with
// `total_cart_velocity_at_joint` but also rotates with
// `cart_velocity_at_joint`. To clarify, `cart_velocity_at_joint` corresponds
// to the Cartesian velocity of a frame solely due to its immediate parent
// actuator (revolute or prismatic joint), while `total_cart_velocity_at_joint`
// is the Cartesian velocity of the frame due to its actuator and the effect of
// the Cartesian velocity of the parent frame on the child frame.
eigenmath::Vector6d ComputeCoriolisAcceleration(
    const eigenmath::Vector6d& cart_velocity_at_joint,
    const eigenmath::Vector6d& total_cart_velocity_at_joint) {
  eigenmath::Vector6d acceleration;
  acceleration.tail<3>() = total_cart_velocity_at_joint.tail<3>().cross(
      cart_velocity_at_joint.tail<3>());
  acceleration.head<3>() = total_cart_velocity_at_joint.head<3>().cross(
                               cart_velocity_at_joint.tail<3>()) +
                           total_cart_velocity_at_joint.tail<3>().cross(
                               cart_velocity_at_joint.head<3>());
  return acceleration;
}

// Takes a `motion_vector` (spatial velocity or acceleration values in local
// coordinates) of a frame whose pose wrt the robot base is given by
// `base_t_robot_frame` and maps it to the `reference_frame`. The
// `motion_vector` is represented as a 6-vector of the form [linear_velocity,
// angular_velocity] or [linear_acceleration, angular_acceleration]. It takes
// an additional offset `robot_frame_p_target` from the robot frame. This is
// nonzero only if there is not a robot frame for the point of interest. The
// `reference_frame` can be the local frame ('kLocal'), the base frame
// ('kRobotBase'), or the local frame with axis aligned with the base frame
// (`kLocalRobotBaseAligned`).
icon::RealtimeStatusOr<eigenmath::Vector6d> ExpressMotionVectorInReferenceFrame(
    const eigenmath::Vector6d& motion_vector, const Pose3d& base_t_robot_frame,
    const ReferenceFrame reference_frame,
    const eigenmath::Vector3d& robot_frame_p_target) {
  switch (reference_frame) {
    case ReferenceFrame::kLocal:
      return eigenmath::Vector6d(
          PlueckerMotionTransformInverse(Pose3d(robot_frame_p_target)) *
          motion_vector);
    case ReferenceFrame::kRobotBase:
      return eigenmath::Vector6d(PlueckerMotionTransform(base_t_robot_frame) *
                                 motion_vector);
    case ReferenceFrame::kLocalRobotBaseAligned: {
      eigenmath::Vector6d motion_vector_in_frame;
      motion_vector_in_frame.head<3>() =
          base_t_robot_frame.rotationMatrix() *
          (motion_vector.head<3>() +
           motion_vector.tail<3>().cross(robot_frame_p_target));
      motion_vector_in_frame.tail<3>() =
          base_t_robot_frame.rotationMatrix() * motion_vector.tail<3>();
      return motion_vector_in_frame;
    }
    default:
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "Unknown reference frame: ", reference_frame));
  }
}

// Returns the pose of the `frame_id` with respect to the non-fixed parent
// joint. A fixed joint does not appear in the chain of dof ids of `frame_id`
// obtained by GetDofChainForElement. Thus, it needs to be identified and taken
// into account separately. This makes it possible to consistently compute a
// kinematic quantity such as a frame velocity or acceleration for robots with
// several fixed joints.
icon::RealtimeStatusOr<Pose3d> GetFramePlacementWrtNonFixedParentJoint(
    const ElementId& frame_id, const ModelInterface& model) {
  // TODO(b/356067211): Extend this computation to support kinematic chains with
  // multiple tips (branched structures).
  if (!model.HasOneTip()) {
    return icon::UnimplementedError("Only kinematic chains are supported.");
  }
  Pose3d first_fixed_joint_t_frame = Pose3d::Identity();

  // Loop over all joints and construct out of the fixed joints the placement of
  // the end-effector with respect to the non FIXED joints.
  for (const ElementId& joint_id : model.GetAllJointIds()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint, model.GetJoint(joint_id));
    // We only need to consider fixed joints that are before the
    // `frame_id` in the kinematic chain.
    if (joint->GetType() == Joint::FIXED && joint_id < frame_id) {
      first_fixed_joint_t_frame =
          first_fixed_joint_t_frame * joint->GetParentTThis();
    }
  }
  return first_fixed_joint_t_frame;
}

}  // namespace

State::State(const ModelInterface* model)
    : model_(model), number_dof_(model_->GetNumberDegreesOfFreedom()) {
  INTRINSIC_RT_ASSIGN_OR_DIE(values_, model_->GetDefaultStateValues());
}

JointStateP State::GetDofPositions() const { return values_.dof_positions; }

icon::RealtimeStatusOr<double> State::GetJointPosition(
    const ElementId& joint_id) const {
  return model_->ComputeJointsDerivedValue(joint_id, values_.dof_positions);
}

JointStateV State::GetDofVelocities() const { return values_.dof_velocities; }

icon::RealtimeStatusOr<double> State::GetJointVelocity(
    const ElementId& joint_id) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint, model_->GetJoint(joint_id));
  if (!joint->IsDof()) {
    // TODO(b/184203032): Currently we assume only joints that have a degree of
    // freedom have a velocity.
    return icon::UnimplementedError(
        "GetJointVelocity not yet supported for non dof joints.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto dof_index,
                                model_->GetDofIndexForElementId(joint_id));
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_velocities.size()));
  return values_.dof_velocities.velocity[dof_index];
}

JointStateA State::GetDofAccelerations() const {
  return values_.dof_accelerations;
}

icon::RealtimeStatusOr<double> State::GetJointAcceleration(
    const ElementId& joint_id) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint, model_->GetJoint(joint_id));
  if (!joint->IsDof()) {
    // TODO(b/184203032): Currently we assume only joints that have a degree of
    // freedom have a acceleration.
    return icon::UnimplementedError(
        "GetJointAcceleration not yet supported for non dof joints.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto dof_index,
                                model_->GetDofIndexForElementId(joint_id));
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_accelerations.size()));
  return values_.dof_accelerations.acceleration[dof_index];
}

icon::RealtimeStatus State::SetStatePV(const JointStatePV& joint_state_pv) {
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_positions.size()));
  if (joint_state_pv.size() != values_.dof_positions.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided state has wrong size. Expected ",
        values_.dof_positions.size(), ", actual ", joint_state_pv.size()));
  }

  values_.dof_positions = joint_state_pv.position;
  values_.dof_velocities = joint_state_pv.velocity;
  return icon::OkStatus();
}

icon::RealtimeStatus State::SetStatePVA(const JointStatePVA& joint_state_pva) {
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_positions.size()));
  if (joint_state_pva.size() != values_.dof_positions.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided state has wrong size. Expected ",
        values_.dof_positions.size(), ", actual ", joint_state_pva.size()));
  }

  values_.dof_positions = joint_state_pva.position;
  values_.dof_velocities = joint_state_pva.velocity;
  values_.dof_accelerations = joint_state_pva.acceleration;
  return icon::OkStatus();
}

icon::RealtimeStatus State::SetDofPositions(const JointStateP& dof_positions,
                                            bool check_limits) {
  if (dof_positions.size() != values_.dof_positions.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of provided dof positions of ", dof_positions.size(),
        " is different of expected size ", values_.dof_positions.size()));
  }
  if (check_limits) {
    for (int i = 0; i < dof_positions.size(); ++i) {
      INTRINSIC_RT_RETURN_IF_ERROR(CheckWithinLimits(
          dof_positions.position[i], values_.dof_limits.min_position[i],
          values_.dof_limits.max_position[i]));
    }
  }
  values_.dof_positions = dof_positions;
  return icon::OkStatus();
}

icon::RealtimeStatus State::SetDofPosition(const ElementId& dof_id,
                                           double value) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto dof_index,
                                model_->GetDofIndexForElementId(dof_id));
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_positions.size()));

  // Check if within limits
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckWithinLimits(value, values_.dof_limits.min_position[dof_index],
                        values_.dof_limits.max_position[dof_index]));

  values_.dof_positions.position[dof_index] = value;
  return icon::OkStatus();
}

void State::SetToDefaultJointPosition() {
  INTRINSIC_RT_ASSIGN_OR_DIE(const StateValues default_state_values,
                             model_->GetDefaultStateValues());
  const JointStateP& default_configuration = default_state_values.dof_positions;
  CHECK_EQ(default_configuration.size(), values_.dof_positions.size());
  values_.dof_positions = default_configuration;
}

icon::RealtimeStatus State::SetDofVelocities(const JointStateV& dof_velocities,
                                             bool check_limits) {
  if (dof_velocities.size() != values_.dof_velocities.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of provided dof velocities of ", dof_velocities.size(),
        " is different of expected size ", values_.dof_velocities.size()));
  }
  if (check_limits) {
    for (int i = 0; i < dof_velocities.size(); ++i) {
      INTRINSIC_RT_RETURN_IF_ERROR(CheckWithinLimits(
          dof_velocities.velocity[i], values_.dof_limits.max_velocity[i]));
    }
  }
  values_.dof_velocities = dof_velocities;
  return icon::OkStatus();
}

icon::RealtimeStatus State::SetDofVelocity(const ElementId& dof_id,
                                           double value) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto dof_index,
                                model_->GetDofIndexForElementId(dof_id));
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_velocities.size()));
  // Check if within state limits
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckWithinLimits(value, values_.dof_limits.max_velocity[dof_index]));

  values_.dof_velocities.velocity[dof_index] = value;
  return icon::OkStatus();
}
void State::SetDofVelocitiesZero() {
  values_.dof_velocities.velocity.setZero();
}

icon::RealtimeStatus State::SetDofAccelerations(
    const JointStateA& dof_accelerations, bool check_limits) {
  if (dof_accelerations.size() != values_.dof_accelerations.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of acceleration values has unexpected size. Expected: ",
        dof_accelerations.size(), " Actual ",
        values_.dof_accelerations.size()));
  }
  if (check_limits) {
    for (int i = 0; i < dof_accelerations.size(); ++i) {
      INTRINSIC_RT_RETURN_IF_ERROR(
          CheckWithinLimits(dof_accelerations.acceleration[i],
                            values_.dof_limits.max_acceleration[i]));
    }
  }
  values_.dof_accelerations = dof_accelerations;
  return icon::OkStatus();
}

icon::RealtimeStatus State::SetDofAcceleration(const ElementId& dof_id,
                                               double value) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto dof_index,
                                model_->GetDofIndexForElementId(dof_id));
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_accelerations.size()));

  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckWithinLimits(value, values_.dof_limits.max_acceleration[dof_index]));

  values_.dof_accelerations.acceleration[dof_index] = value;
  return icon::OkStatus();
}

void State::SetDofAccelerationsZero() {
  values_.dof_accelerations.acceleration.setZero();
}

JointStateT State::GetDofEfforts() const { return values_.dof_efforts; }

icon::RealtimeStatusOr<double> State::GetDofEffort(
    const ElementId& dof_id) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto dof_index,
                                model_->GetDofIndexForElementId(dof_id));
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_efforts.size()));
  return values_.dof_efforts.torque[dof_index];
}

icon::RealtimeStatus State::SetDofEfforts(const JointStateT& dof_efforts) {
  if (dof_efforts.size() != values_.dof_efforts.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Effort values have unexpected size. Expected: ", dof_efforts.size(),
        " Actual ", values_.dof_efforts.size()));
  }
  const auto system_limits = model_->GetDofSystemLimits();
  if (system_limits.size() != dof_efforts.size()) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        "Effort limits of underlying model have unexpected size. Expected ",
        dof_efforts.size(), ", actual: ", system_limits.size()));
  }
  for (int i = 0; i < system_limits.size(); ++i) {
    INTRINSIC_RT_RETURN_IF_ERROR(
        CheckWithinLimits(dof_efforts.torque[i], system_limits.max_torque[i]));
  }

  values_.dof_efforts = dof_efforts;
  return icon::OkStatus();
}

icon::RealtimeStatus State::SetDofEffort(const ElementId& dof_id,
                                         double value) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto dof_index,
                                model_->GetDofIndexForElementId(dof_id));
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_efforts.size()));

  const auto system_limits = model_->GetDofSystemLimits();
  if (system_limits.size() != values_.dof_efforts.size()) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        "Effort limits of underlying model have unexpected size. Expected ",
        values_.dof_efforts.size(), ", actual: ", system_limits.size()));
  }

  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckWithinLimits(value, system_limits.max_torque[dof_index]));

  values_.dof_efforts.torque[dof_index] = value;
  return icon::OkStatus();
}

void State::SetDofEffortsZero() { values_.dof_efforts.torque.setZero(); }

JointLimits State::GetDofLimits() const { return values_.dof_limits; }

icon::RealtimeStatus State::SetDofLimits(const JointLimits& limits) {
  if (!limits.IsValid()) {
    return icon::InvalidArgumentError("The provided state limits are invalid.");
  }
  if (limits.size() != values_.dof_limits.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The provided state limits does not have expected size. Expected ",
        values_.dof_limits.size(), ", actual ", limits.size()));
  }

  // Check if limits are within system limits.
  auto system_limits = model_->GetDofSystemLimits();
  if (!system_limits.IsValid()) {
    return icon::InternalError(
        "Internal error. System limits of model are invalid.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto limit_check_result,
                                IsWithinLimits(limits, system_limits));
  if (!limit_check_result) {
    return icon::InvalidArgumentError("Provided limits exceed system limits.");
  }

  values_.dof_limits = limits;
  return icon::OkStatus();
}

const ModelInterface* State::GetKinematicModel() const { return model_; }

icon::RealtimeStatusOr<eigenmath::Matrix6Nd> State::ComputeJacobian() const {
  auto tips = model_->GetTipIds();
  if (tips.size() != 1) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "There should be a single tip. Got ", tips.size()));
  }
  return ComputeJacobian(tips.front());
}

icon::RealtimeStatusOr<eigenmath::Vector6d> State::ComputeSpatialFrameVelocity(
    const ElementId& frame_id, const eigenmath::Vector3d& robot_frame_p_target,
    const ReferenceFrame reference_frame) const {
  // Get the placement of the frame with respect to the fixed parent joint and
  // include the additional desired offset `robot_frame_p_target`.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      Pose3d frame_placement,
      GetFramePlacementWrtNonFixedParentJoint(frame_id, *model_));
  frame_placement = frame_placement * Pose3d(robot_frame_p_target);

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      (const FixedVector<ElementId, eigenmath::VectorNd::MaxSizeAtCompileTime>
           chain_dof_ids),
      model_->GetDofChainForElement(frame_id));

  // Velocity vector composed of linear and angular velocity.
  eigenmath::Vector6d parent_frame_vel = eigenmath::Vector6d::Zero();

  Pose3d base_t_dof_frame = Pose3d::Identity();
  ElementId last_chain_dof_id = model_->GetBaseId();
  for (int i = 0; i < chain_dof_ids.size(); ++i) {
    const ElementId& chain_dof_id = chain_dof_ids[i];

    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const int dof_id, model_->GetDofIndexForElementId(chain_dof_id));
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* dof_element,
                                  model_->GetJoint(chain_dof_id));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const Pose3d parent_t_this_dof,
        GetTransform(last_chain_dof_id, chain_dof_id));
    last_chain_dof_id = chain_dof_id;

    eigenmath::Vector6d cart_velocity_at_joint = eigenmath::Vector6d::Zero();
    switch (dof_element->GetType()) {
      case Joint::REVOLUTE: {
        const eigenmath::Vector3d& joint_axis = dof_element->GetAxis();
        cart_velocity_at_joint.tail<3>() =
            joint_axis * (values_.joint_dependency_matrix *
                          values_.dof_velocities.velocity)[dof_id];
        break;
      }
      case Joint::PRISMATIC:
        // TODO(b/356320208): Implement computation of velocity for prismatic
        // joints.
        return icon::UnimplementedError("Prismatic joint not implemented.");
      default:
        return icon::InternalError(
            "Dof has to be either a revolute or a prismatic joint");
    }

    eigenmath::Vector6d frame_vel =
        cart_velocity_at_joint +
        PlueckerMotionTransformInverse(parent_t_this_dof) * parent_frame_vel;

    base_t_dof_frame = base_t_dof_frame * parent_t_this_dof;
    parent_frame_vel = frame_vel;
  }

  return ExpressMotionVectorInReferenceFrame(parent_frame_vel, base_t_dof_frame,
                                             reference_frame,
                                             frame_placement.translation());
}

icon::RealtimeStatusOr<eigenmath::Vector6d>
State::ComputeSpatialFrameAcceleration(
    const ElementId& frame_id, const eigenmath::Vector3d& robot_frame_p_target,
    const ReferenceFrame reference_frame) const {
  // Get the placement of the frame with respect to the fixed parent joint and
  // include the additional desired offset `robot_frame_p_target`.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      Pose3d frame_placement,
      GetFramePlacementWrtNonFixedParentJoint(frame_id, *model_));
  frame_placement = frame_placement * Pose3d(robot_frame_p_target);

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      (const FixedVector<ElementId, eigenmath::VectorNd::MaxSizeAtCompileTime>
           chain_dof_ids),
      model_->GetDofChainForElement(frame_id));

  // Velocity vector composed of linear and angular velocity.
  eigenmath::Vector6d parent_frame_vel = eigenmath::Vector6d::Zero();
  eigenmath::Vector6d parent_frame_acc = eigenmath::Vector6d::Zero();

  Pose3d base_t_dof_frame = Pose3d::Identity();
  ElementId last_chain_dof_id = model_->GetBaseId();
  for (int i = 0; i < chain_dof_ids.size(); ++i) {
    const ElementId& chain_dof_id = chain_dof_ids[i];

    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const int dof_id, model_->GetDofIndexForElementId(chain_dof_id));
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* dof_element,
                                  model_->GetJoint(chain_dof_id));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const Pose3d parent_t_this_dof,
        GetTransform(last_chain_dof_id, chain_dof_id));
    last_chain_dof_id = chain_dof_id;

    eigenmath::Vector6d cart_velocity_at_joint = eigenmath::Vector6d::Zero();
    eigenmath::Vector6d cart_acceleration_at_joint =
        eigenmath::Vector6d::Zero();
    switch (dof_element->GetType()) {
      case Joint::REVOLUTE: {
        const eigenmath::Vector3d& joint_axis = dof_element->GetAxis();
        cart_velocity_at_joint.tail<3>() =
            joint_axis * (values_.joint_dependency_matrix *
                          values_.dof_velocities.velocity)[dof_id];
        cart_acceleration_at_joint.tail<3>() =
            joint_axis * (values_.joint_dependency_matrix *
                          values_.dof_accelerations.acceleration)[dof_id];
        break;
      }
      case Joint::PRISMATIC:
        // TODO(b/356320208): Implement computation of acceleration for
        // prismatic joints.
        return icon::UnimplementedError("Prismatic joint not implemented.");
      default:
        return icon::InternalError(
            "Dof has to be either a revolute or a prismatic joint");
    }

    eigenmath::Vector6d frame_vel =
        cart_velocity_at_joint +
        PlueckerMotionTransformInverse(parent_t_this_dof) * parent_frame_vel;
    eigenmath::Vector6d frame_acc =
        cart_acceleration_at_joint +
        ComputeCoriolisAcceleration(cart_velocity_at_joint, frame_vel) +
        PlueckerMotionTransformInverse(parent_t_this_dof) * parent_frame_acc;

    base_t_dof_frame = base_t_dof_frame * parent_t_this_dof;
    parent_frame_vel = frame_vel;
    parent_frame_acc = frame_acc;
  }
  return ExpressMotionVectorInReferenceFrame(parent_frame_acc, base_t_dof_frame,
                                             reference_frame,
                                             frame_placement.translation());
}

icon::RealtimeStatusOr<eigenmath::Vector6d>
State::ComputeClassicalFrameAcceleration(
    const ElementId& frame_id, const eigenmath::Vector3d& robot_frame_p_target,
    const ReferenceFrame reference_frame) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::Vector6d frame_vel,
      State::ComputeSpatialFrameVelocity(frame_id, robot_frame_p_target,
                                         reference_frame));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::Vector6d frame_acc,
      State::ComputeSpatialFrameAcceleration(frame_id, robot_frame_p_target,
                                             reference_frame));

  eigenmath::Vector6d frame_classical_acc = frame_acc;
  frame_classical_acc.head<3>() +=
      frame_vel.tail<3>().cross(frame_vel.head<3>());
  return frame_classical_acc;
}

icon::RealtimeStatusOr<eigenmath::Matrix6Nd> State::ComputeJacobian(
    const ElementId& frame_id,
    const eigenmath::Vector3d& robot_frame_p_target) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      (const FixedVector<ElementId, eigenmath::VectorNd::MaxSizeAtCompileTime>
           chain_dof_ids),
      model_->GetDofChainForElement(frame_id));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d base_t_robot_frame,
                                GetTransform(frame_id));
  const eigenmath::Vector3d base_p_target =
      base_t_robot_frame * robot_frame_p_target;

  eigenmath::Matrix6Nd jacobian = eigenmath::Matrix6Nd::Zero(6, number_dof_);
  Pose3d base_t_dof = Pose3d::Identity();
  ElementId last_chain_dof_id = model_->GetBaseId();
  for (int i = 0; i < chain_dof_ids.size(); ++i) {
    const ElementId& chain_dof_id = chain_dof_ids[i];

    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const int dof_index, model_->GetDofIndexForElementId(chain_dof_id));
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* dof_element,
                                  model_->GetJoint(chain_dof_id));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const Pose3d last_chain_dof_t_this_dof,
        GetTransform(last_chain_dof_id, chain_dof_id));

    base_t_dof *= last_chain_dof_t_this_dof;
    last_chain_dof_id = chain_dof_id;
    const eigenmath::Vector3d axis_i =
        base_t_dof.quaternion() * dof_element->GetAxis();
    const eigenmath::Vector3d& base_p_dof = base_t_dof.translation();

    // The jacobian provides the relationship between joint velocities and
    // cartesian velocities for a cartesian coordinate in the kinematic chain.
    // As a result, we have as many columns as we have dof within this chain.
    // The column for each joint is computed dependent on the type of joint.
    // Each column has six entries. The first three rows define the linear
    // velocities (i.e., how fast the end-point is moving in x, y, and z
    // dicrection) relative to the base frame. The last tree rows represent the
    // angular velocity (i.e., how fast the end-point rotates around the x, y,
    // and z axis) with respect to the base frame.
    switch (dof_element->GetType()) {
      case Joint::REVOLUTE: {
        // The linear velocity is defined by the rotational movement of the
        // joint relative to base and the lever length of the arm attached to
        // it. The angular velocity is defined by the rotation of the joint axis
        // itself.
        jacobian.template block<3, 1>(0, dof_index) =
            eigenmath::SkewSymmetricMatrix(axis_i) *
            (base_p_target - base_p_dof);
        jacobian.template block<3, 1>(3, dof_index) = axis_i;
        break;
      }
      case Joint::PRISMATIC:
        // Prismatic joints only cause a displacement of the end-points,
        // therefore the angular velocity (last three entries of the column) is
        // zero while the linear velocity (the first three rows) is defined by
        // the direction of movement of the prismatic joint (== axis_i).
        jacobian.template block<3, 1>(0, dof_index) = axis_i;
        jacobian.template block<3, 1>(3, dof_index) =
            eigenmath::Vector3d::Zero();
        break;
      default:
        return icon::InternalError(
            "Dof has to be either a revolute or a prismatic joint");
    }
  }

  eigenmath::Matrix6Nd jacobian_with_dependent_joints =
      jacobian * values_.joint_dependency_matrix;

  return jacobian_with_dependent_joints;
}

icon::RealtimeStatusOr<eigenmath::Matrix6Nd> State::ComputeJacobian(
    const ElementId& frame_id, const eigenmath::Vector3d& robot_frame_p_target,
    const eigenmath::Quaterniond& base_R_view) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::Matrix6Nd geometric_jacobian_wrt_base,
      ComputeJacobian(frame_id, robot_frame_p_target));
  eigenmath::Quaterniond view_R_base(base_R_view.conjugate());
  eigenmath::Matrix6Nd geometric_jacobian_wrt_view(
      6, model_->GetNumberDegreesOfFreedom());
  geometric_jacobian_wrt_view.topRows<3>() =
      view_R_base.matrix() * geometric_jacobian_wrt_base.topRows<3>();
  geometric_jacobian_wrt_view.bottomRows<3>() =
      view_R_base.matrix() * geometric_jacobian_wrt_base.bottomRows<3>();

  return geometric_jacobian_wrt_view;
}

icon::RealtimeStatusOr<eigenmath::Matrix6Nd> State::ComputeAnalyticJacobian()
    const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::Matrix6Nd geometric_jacobian_wrt_base, ComputeJacobian());
  // The ComputeJacobian() above has already checked/assured that we only have a
  // single tip here, so we can proceed directly below without no more checking
  // on the number of tips in the model.
  INTRINSIC_RT_ASSIGN_OR_RETURN(Pose3d base_t_tip,
                                GetTransform(model_->GetTipIds().front()));
  eigenmath::Matrix6d geometric_to_analytic_jacobian_mapper =
      ComputeGeometricToAnalyticJacobianMapper(base_t_tip.so3());
  return eigenmath::Matrix6Nd(geometric_to_analytic_jacobian_mapper *
                              geometric_jacobian_wrt_base);
}

icon::RealtimeStatusOr<eigenmath::Matrix6Nd> State::ComputeAnalyticJacobian(
    const ElementId& frame_id, const Pose3d& robot_frame_t_target) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::Matrix6Nd geometric_jacobian_wrt_base,
      ComputeJacobian(frame_id, robot_frame_t_target.translation()));
  INTRINSIC_RT_ASSIGN_OR_RETURN(Pose3d base_t_robot_frame,
                                GetTransform(frame_id));
  Pose3d base_t_target = base_t_robot_frame * robot_frame_t_target;
  eigenmath::Matrix6d geometric_to_analytic_jacobian_mapper =
      ComputeGeometricToAnalyticJacobianMapper(base_t_target.so3());
  return eigenmath::Matrix6Nd(geometric_to_analytic_jacobian_mapper *
                              geometric_jacobian_wrt_base);
}

icon::RealtimeStatusOr<eigenmath::Matrix6Nd> State::ComputeAnalyticJacobian(
    const ElementId& frame_id, const Pose3d& robot_frame_t_target,
    const eigenmath::Quaterniond& base_R_view) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::Matrix6Nd analytic_jacobian_wrt_base,
      ComputeAnalyticJacobian(frame_id, robot_frame_t_target));
  eigenmath::Quaterniond view_R_base(base_R_view.conjugate());
  eigenmath::Matrix6Nd analytic_jacobian_wrt_view =
      eigenmath::Matrix6Nd::Zero(6, model_->GetNumberDegreesOfFreedom());
  analytic_jacobian_wrt_view.topRows<3>() =
      view_R_base.matrix() * analytic_jacobian_wrt_base.topRows<3>();
  analytic_jacobian_wrt_view.bottomRows<3>() =
      view_R_base.matrix() * analytic_jacobian_wrt_base.bottomRows<3>();
  return analytic_jacobian_wrt_view;
}

icon::RealtimeStatusOr<Pose3d> State::GetTransform(
    const ElementId& element_id) const {
  return GetTransform(model_->GetBaseId(), element_id);
}

icon::RealtimeStatusOr<Pose3d> State::GetTransform(const ElementId& from,
                                                   const ElementId& to) const {
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckModelConsistency(model_, values_.dof_positions.size()));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const ElementId common_ancestor_id,
                                model_->GetCommonAncestor(from, to));

  // Computes the transform from `common_ancestor` to `element_id`.
  auto get_ancestor_t_element =
      [&](ElementId element_id) -> icon::RealtimeStatusOr<Pose3d> {
    Pose3d ancestor_t_element;
    while (element_id != common_ancestor_id) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(const Element* element,
                                    model_->GetElement(element_id));
      // Compute the outbound transformation, i.e., the `parent_t_this *
      // joint_transform(q)`
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const Pose3d parent_t_this,
          GetOutboundParentTransform(*model_, element_id,
                                     values_.dof_positions));
      // Generate ancestor_t_element by performing the incremental
      // operation: `ancestor_t_this = parent_t_this * this_t_element` until
      // parent is common_ancestor.
      ancestor_t_element = parent_t_this * ancestor_t_element;
      std::optional<const Element*> parent = element->GetParentElement();
      if (parent.has_value()) {
        INTRINSIC_RT_ASSIGN_OR_RETURN(element_id,
                                      model_->GetElementId(parent.value()));
      } else {
        // We should never reach this point.
        return icon::InternalError(icon::RealtimeStatus::StrCat(
            "Element has no parent. Element ", element->GetName(),
            " (id: ", element_id.value(), ")"));
      }
    }
    return ancestor_t_element;
  };

  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d ancestor_t_from,
                                get_ancestor_t_element(from));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d ancestor_t_to,
                                get_ancestor_t_element(to));

  return Pose3d(ancestor_t_from.inverse() * ancestor_t_to);
}

}  // namespace kinematics
}  // namespace intrinsic
