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

#include "intrinsic/motion_planning/proto/motion_specification_proto_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/joint_position_limits.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/joint_position_sum_limit.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/point_at.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/pose_equality.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/position_bounding_box.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/position_equality.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/rotation_cone.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/rotation_equality.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_limits.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/objects/transform_node_internal.h"

namespace intrinsic {
namespace {

using intrinsic_proto::motion_planning::v1::GeometricConstraint;
using intrinsic_proto::motion_planning::v1::UniformGeometricConstraint;

bool IsInfiniteVector(const eigenmath::VectorNd& vec) {
  return std::any_of(vec.begin(), vec.end(),
                     [](double v) { return std::isinf(v); });
}
}  // namespace
CartesianLimits ParseCartesianLimitsFromProto(
    const intrinsic_proto::motion_planning::v1::DynamicCartesianLimits&
        cartesian_limits,
    const CartesianLimits& default_limits) {
  CartesianLimits new_cart_limits = default_limits;
  // Only set the limits that are set by the user.
  if (cartesian_limits.has_max_rotational_acceleration()) {
    new_cart_limits.max_rotational_acceleration =
        cartesian_limits.max_rotational_acceleration();
  }
  if (cartesian_limits.has_max_rotational_velocity()) {
    new_cart_limits.max_rotational_velocity =
        cartesian_limits.max_rotational_velocity();
  }
  if (cartesian_limits.has_max_translational_acceleration()) {
    new_cart_limits.max_translational_acceleration.setConstant(
        cartesian_limits.max_translational_acceleration());
    new_cart_limits.min_translational_acceleration.setConstant(
        -cartesian_limits.max_translational_acceleration());
  }
  if (cartesian_limits.has_max_translational_velocity()) {
    new_cart_limits.max_translational_velocity.setConstant(
        cartesian_limits.max_translational_velocity());
    new_cart_limits.min_translational_velocity.setConstant(
        -cartesian_limits.max_translational_velocity());
  }
  return new_cart_limits;
}

CartesianLimits ParseCartesianLimitsFromProto(
    const intrinsic_proto::motion_planning::v1::MotionSegment& motion_segment,
    const CartesianLimits& default_limits) {
  if (motion_segment.has_cartesian_limits()) {
    return ParseCartesianLimitsFromProto(motion_segment.cartesian_limits(),
                                         default_limits);
  }
  return default_limits;
}

absl::Status VerifyRobotInJointPositionConstraint(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::JointPositionLimits&
        joint_position_proto) {
  switch (joint_position_proto.joint_group_case()) {
    case intrinsic_proto::motion_planning::v1::JointPositionLimits::
        JointGroupCase::kObjectId: {
      INTR_ASSIGN_OR_RETURN(
          auto robot_for_limits,
          object_world::GetKinematicObjectByReference(
              object_world, joint_position_proto.object_id()));
      if (robot_for_limits == nullptr) {
        return absl::InvalidArgumentError(
            "Cannot parse kinematic object from joint limit proto.");
      }

      if (robot_for_limits->GetRobotEntityId() != robot.GetRobotEntityId()) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Robot for path limits (id = %i ) is different than the robot "
            "specified for planning (id = %i).",
            robot_for_limits->GetRobotEntityId().value(),
            robot.GetRobotEntityId().value()));
      }
      break;
    }
    case intrinsic_proto::motion_planning::v1::JointPositionLimits::
        JointGroupCase::kJointIds: {
      return absl::InvalidArgumentError(
          "Using individual JointIds for joint limit constraints not yet "
          "supported.");
    }
    case intrinsic_proto::motion_planning::v1::JointPositionLimits::
        JointGroupCase::JOINT_GROUP_NOT_SET: {
      return absl::InvalidArgumentError("Robot reference was unspecified.");
    }
  }
  return absl::OkStatus();
}

absl::Status SetJointPositionLimitsFromProto(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::JointPositionLimits&
        joint_position_proto,
    JointLimits& joint_limits) {
  // TODO(b/266783354): Currently only support single robot definitions. Extend
  // to joint groups.
  INTR_RETURN_IF_ERROR(VerifyRobotInJointPositionConstraint(
      object_world, robot, joint_position_proto));

  INTR_ASSIGN_OR_RETURN(
      eigenmath::VectorNd max_position,
      icon::RepeatedDoubleToVectorNd(joint_position_proto.upper_limits()));
  INTR_ASSIGN_OR_RETURN(
      eigenmath::VectorNd min_position,
      icon::RepeatedDoubleToVectorNd(joint_position_proto.lower_limits()));

  if (max_position.size() != 0) {
    if (joint_limits.max_position.size() != max_position.size()) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Maximum position limits in path constraint are of different "
          "size then expected. "
          "Expected size %i, found size %i.",
          joint_limits.max_position.size(), max_position.size()));
    }
    joint_limits.max_position = max_position;
  }

  if (min_position.size() != 0) {
    if (joint_limits.min_position.size() != min_position.size()) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Minimum position limits in path constraint are of different "
          "size then expected. "
          "Expected size %i, found size %i.",
          joint_limits.min_position.size(), min_position.size()));
    }

    joint_limits.min_position = min_position;
  }

  if ((joint_limits.max_position - joint_limits.min_position).minCoeff() < 0) {
    return absl::InvalidArgumentError(
        "Minimum position limits exceed maximum position limits.");
  }

  return absl::OkStatus();
}

absl::StatusOr<JointLimits> ParseJointLimitsFromProto(
    const intrinsic_proto::motion_planning::v1::MotionSegment& motion_segment,
    const JointLimits& default_limits) {
  JointLimits planning_joint_limits = default_limits;
  // Parse the position limit values.
  if (!motion_segment.has_joint_limits()) {
    return planning_joint_limits;
  }

  const auto& limits = motion_segment.joint_limits();
  if (limits.has_max_position()) {
    INTR_ASSIGN_OR_RETURN(
        planning_joint_limits.max_position,
        icon::RepeatedDoubleToVectorNd(limits.max_position().values()));
  }
  if (limits.has_min_position()) {
    INTR_ASSIGN_OR_RETURN(
        planning_joint_limits.min_position,
        icon::RepeatedDoubleToVectorNd(limits.min_position().values()));
  }

  // Parse the dynamic limit values.
  if (limits.has_max_velocity()) {
    INTR_ASSIGN_OR_RETURN(
        planning_joint_limits.max_velocity,
        icon::RepeatedDoubleToVectorNd(limits.max_velocity().values()));
  }
  if (limits.has_max_acceleration()) {
    INTR_ASSIGN_OR_RETURN(
        planning_joint_limits.max_acceleration,
        icon::RepeatedDoubleToVectorNd(limits.max_acceleration().values()));
  }
  if (limits.has_max_jerk()) {
    INTR_ASSIGN_OR_RETURN(
        planning_joint_limits.max_jerk,
        icon::RepeatedDoubleToVectorNd(limits.max_jerk().values()));
  }

  if (!planning_joint_limits.IsValid()) {
    // TODO(kmuelling): Write a function for this.
    if (!planning_joint_limits.IsSizeConsistent()) {
      if (planning_joint_limits.max_position.size() !=
          default_limits.max_position.size()) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Provided maximum position limits in motion segment are of "
            "different size then expected. Expected size %i, actual size %i.",
            default_limits.max_position.size(),
            planning_joint_limits.max_position.size()));
      }
      if (planning_joint_limits.min_position.size() !=
          default_limits.min_position.size()) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Provided minimum position limits in motion segment are of "
            "different size then expected. Expected size %i, actual size %i.",
            default_limits.min_position.size(),
            planning_joint_limits.min_position.size()));
      }
      if (planning_joint_limits.max_velocity.size() !=
          default_limits.max_velocity.size()) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Provided joint velocity limits in motion segment are of different "
            "size then expected. Expected size %i, actual size %i.",
            default_limits.max_velocity.size(),
            planning_joint_limits.max_velocity.size()));
      }
      if (planning_joint_limits.max_acceleration.size() !=
          default_limits.max_acceleration.size()) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Provided joint acceleration limits in motion segment are of "
            "different size then expected. Expected size %i, actual size %i.",
            default_limits.max_acceleration.size(),
            planning_joint_limits.max_acceleration.size()));
      }
      if (planning_joint_limits.max_jerk.size() !=
          default_limits.max_jerk.size()) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Provided joint jerk limits in motion segment are of different "
            "size then expected. Expected size %i, actual size %i.",
            default_limits.max_jerk.size(),
            planning_joint_limits.max_jerk.size()));
      }

      return absl::InvalidArgumentError(
          "Provided joint limits contain inconsistent sizes.");
    }

    if ((planning_joint_limits.max_position -
         planning_joint_limits.min_position)
            .minCoeff() < 0) {
      return absl::InvalidArgumentError(
          "Minimum position limits exceed maximum position limits.");
    }
    if ((planning_joint_limits.max_velocity.array() < 0).all()) {
      return absl::InvalidArgumentError(
          "Provided joint velocity limits contain negative values. "
          "Require zero or positive values.");
    }
    if ((planning_joint_limits.max_acceleration.array() < 0).all()) {
      return absl::InvalidArgumentError(
          "Provided joint acceleration limits contain negative values. "
          "Require zero or positive values.");
    }
    if ((planning_joint_limits.max_jerk.array() < 0).all()) {
      return absl::InvalidArgumentError(
          "Provided joint jerk limits contain negative values. "
          "Require zero or positive values.");
    }

    return absl::InvalidArgumentError("Provided joint limits are invalid.");
  }

  return planning_joint_limits;
}

absl::StatusOr<JointLimits> ToJointLimits(
    const intrinsic_proto::motion_planning::v1::JointLimitsUpdate&
        joint_limits_proto) {
  size_t num_joints = 0;
  if (joint_limits_proto.has_min_position()) {
    num_joints = joint_limits_proto.min_position().values_size();
  } else if (joint_limits_proto.has_max_position()) {
    num_joints = joint_limits_proto.max_position().values_size();
  } else if (joint_limits_proto.has_max_velocity()) {
    num_joints = joint_limits_proto.max_velocity().values_size();
  } else if (joint_limits_proto.has_max_acceleration()) {
    num_joints = joint_limits_proto.max_acceleration().values_size();
  } else if (joint_limits_proto.has_max_jerk()) {
    num_joints = joint_limits_proto.max_jerk().values_size();
  } else {
    return absl::InvalidArgumentError(
        absl::StrCat("No field is set in the JointLimitsUpdate proto:\n",
                     joint_limits_proto));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(JointLimits limits,
                                JointLimits::Unlimited(num_joints));
  if (joint_limits_proto.has_min_position()) {
    limits.min_position =
        RepeatedDoubleToVectorXd(joint_limits_proto.min_position().values());
  }
  if (joint_limits_proto.has_max_position()) {
    limits.max_position =
        RepeatedDoubleToVectorXd(joint_limits_proto.max_position().values());
  }
  if (joint_limits_proto.has_max_velocity()) {
    limits.max_velocity =
        RepeatedDoubleToVectorXd(joint_limits_proto.max_velocity().values());
  }
  if (joint_limits_proto.has_max_acceleration()) {
    limits.max_acceleration = RepeatedDoubleToVectorXd(
        joint_limits_proto.max_acceleration().values());
  }
  if (joint_limits_proto.has_max_jerk()) {
    limits.max_jerk =
        RepeatedDoubleToVectorXd(joint_limits_proto.max_jerk().values());
  }
  if (!limits.IsValid()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "JointLimitsUpdate proto are invalid:\n", joint_limits_proto));
  }
  return limits;
}

intrinsic_proto::motion_planning::v1::JointLimitsUpdate
ToJointLimitsUpdateProto(const JointLimits& joint_limits) {
  intrinsic_proto::motion_planning::v1::JointLimitsUpdate joint_limits_proto;
  if (!IsInfiniteVector(joint_limits.min_position)) {
    VectorXdToRepeatedDouble(
        joint_limits.min_position,
        joint_limits_proto.mutable_min_position()->mutable_values());
  }
  if (!IsInfiniteVector(joint_limits.max_position)) {
    VectorXdToRepeatedDouble(
        joint_limits.max_position,
        joint_limits_proto.mutable_max_position()->mutable_values());
  }
  if (!IsInfiniteVector(joint_limits.max_velocity)) {
    VectorXdToRepeatedDouble(
        joint_limits.max_velocity,
        joint_limits_proto.mutable_max_velocity()->mutable_values());
  }
  if (!IsInfiniteVector(joint_limits.max_acceleration)) {
    VectorXdToRepeatedDouble(
        joint_limits.max_acceleration,
        joint_limits_proto.mutable_max_acceleration()->mutable_values());
  }
  if (!IsInfiniteVector(joint_limits.max_jerk)) {
    VectorXdToRepeatedDouble(
        joint_limits.max_jerk,
        joint_limits_proto.mutable_max_jerk()->mutable_values());
  }
  return joint_limits_proto;
}

intrinsic_proto::motion_planning::v1::DynamicCartesianLimits
ToDynamicCartesianLimitsProto(const CartesianLimits& cartesian_limits) {
  intrinsic_proto::motion_planning::v1::DynamicCartesianLimits
      cart_limits_proto;
  if (!std::isinf(cartesian_limits.max_rotational_velocity)) {
    cart_limits_proto.set_max_rotational_velocity(
        cartesian_limits.max_rotational_velocity);
  }
  if (!std::isinf(cartesian_limits.max_rotational_acceleration)) {
    cart_limits_proto.set_max_rotational_acceleration(
        cartesian_limits.max_rotational_acceleration);
  }
  if (!IsInfiniteVector(cartesian_limits.max_translational_velocity)) {
    cart_limits_proto.set_max_translational_velocity(
        cartesian_limits.max_translational_velocity.maxCoeff());
  }
  if (!IsInfiniteVector(cartesian_limits.max_translational_acceleration)) {
    cart_limits_proto.set_max_translational_acceleration(
        cartesian_limits.max_translational_acceleration.maxCoeff());
  }
  return cart_limits_proto;
}

absl::StatusOr<std::vector<std::unique_ptr<ConstraintInterface>>>
GetUniformGeometricConstraintsFromProto(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::UniformGeometricConstraint&
        geometric_constraint_proto) {
  std::vector<std::unique_ptr<ConstraintInterface>> resolved_constraints;

  if (geometric_constraint_proto.ByteSizeLong() == 0) {
    VLOG(1)
        << "Empty geometric_constraint_proto passed. Returning empty vector.";
    return resolved_constraints;
  }
  // Using queue as the container to allow multiple constraints that might exist
  // in constraint intersection.
  std::queue<UniformGeometricConstraint> queue;
  queue.push(geometric_constraint_proto);
  // TODO(b/275386091)
  constexpr double kDefaultTolerance = 1e-5;
  while (!queue.empty()) {
    const intrinsic_proto::motion_planning::v1::UniformGeometricConstraint
        constraint = queue.front();
    queue.pop();
    // TODO(b/276764433): Use visitor patterns.
    switch (constraint.constraint_case()) {
      // For an intersection, we add all constraints to the queue.
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kUniformGeometricConstraintIntersection: {
        for (const intrinsic_proto::motion_planning::v1::
                 UniformGeometricConstraint& element :
             constraint.uniform_geometric_constraint_intersection()
                 .constraints()) {
          queue.push(element);
        }
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kRotationCone: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<motion_planning::RotationConeConstraint>
                cone_constraint,
            motion_planning::RotationConeConstraint::Create(
                object_world, constraint.rotation_cone(), kDefaultTolerance));
        resolved_constraints.push_back(std::move(cone_constraint));
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kPositionBoundingBox: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<motion_planning::PositionBoundingBoxConstraint>
                position_bounding_box_constraint,
            motion_planning::PositionBoundingBoxConstraint::Create(
                object_world, constraint.position_bounding_box(),
                kDefaultTolerance));
        resolved_constraints.push_back(
            std::move(position_bounding_box_constraint));
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kJointPositionSumLimit: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<motion_planning::JointPositionSumLimitConstraint>
                joint_position_sum_limit_constraint,
            motion_planning::JointPositionSumLimitConstraint::Create(
                object_world, constraint.joint_position_sum_limit(),
                kDefaultTolerance));
        resolved_constraints.push_back(
            std::move(joint_position_sum_limit_constraint));
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kPointAt: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<motion_planning::PointAtConstraint>
                joint_position_sum_limit_constraint,
            motion_planning::PointAtConstraint::Create(object_world,
                                                       constraint.point_at()));
        resolved_constraints.push_back(
            std::move(joint_position_sum_limit_constraint));
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          CONSTRAINT_NOT_SET: {
        return absl::InvalidArgumentError(
            "Constraint type unknown or unspecified.");
      }
      default: {
        return absl::InvalidArgumentError(
            "Un-identified UniformGeometricConstraint provided.");
      }
    }
  }
  return std::move(resolved_constraints);
}

absl::StatusOr<intrinsic_proto::motion_planning::v1::PositionEquality>
ConvertRelativePositionConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::RelativePositionEquality&
        relative_constraint) {
  INTR_ASSIGN_OR_RETURN(
      const object_world::TransformNode* transform_node,
      GetTransformNodeByReference(world, relative_constraint.moving_frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId moving_id,
                        transform_node->GetTransformOriginEntityId());

  AttachmentEntityId reference_id;
  if (relative_constraint.has_reference_frame()) {
    INTR_ASSIGN_OR_RETURN(transform_node,
                          GetTransformNodeByReference(
                              world, relative_constraint.reference_frame()));
    INTR_ASSIGN_OR_RETURN(reference_id,
                          transform_node->GetTransformOriginEntityId());
  } else {
    // If there is no reference frame set, we set the reference frame to be the
    // pose of the moving_frame at the beginning of the motion. This results in
    // motion relative to moving_frame's local frame.
    reference_id = moving_id;
  }

  eigenmath::Vector3d reference_p_translation =
      FromProto(relative_constraint.relative_position());

  eigenmath::Vector3d moving_p_offset(0, 0, 0);
  if (relative_constraint.has_moving_frame_offset()) {
    moving_p_offset = FromProto(relative_constraint.moving_frame_offset());
  }

  Pose3d moving_t_reference =
      world.GetEntityWorld().GetTransform(moving_id, reference_id);
  eigenmath::Vector3d moving_p_goal =
      moving_p_offset + (moving_t_reference.so3() * reference_p_translation);

  intrinsic_proto::motion_planning::v1::PositionEquality result_proto;
  *result_proto.mutable_moving_frame() = relative_constraint.moving_frame();
  if (relative_constraint.has_moving_frame_offset()) {
    *result_proto.mutable_moving_frame_offset() =
        relative_constraint.moving_frame_offset();
  }
  *result_proto.mutable_target_frame() = relative_constraint.moving_frame();
  *result_proto.mutable_target_frame_offset() = ToProto(moving_p_goal);

  return result_proto;
}
absl::StatusOr<intrinsic_proto::motion_planning::v1::RotationEquality>
ConvertRelativeRotationConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::RelativeRotationEquality&
        relative_constraint) {
  INTR_ASSIGN_OR_RETURN(
      const object_world::TransformNode* transform_node,
      GetTransformNodeByReference(world, relative_constraint.moving_frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId moving_id,
                        transform_node->GetTransformOriginEntityId());

  AttachmentEntityId reference_id;
  if (relative_constraint.has_reference_frame()) {
    INTR_ASSIGN_OR_RETURN(transform_node,
                          GetTransformNodeByReference(
                              world, relative_constraint.reference_frame()));
    INTR_ASSIGN_OR_RETURN(reference_id,
                          transform_node->GetTransformOriginEntityId());
  } else {
    // If there is no reference frame set, we set the reference frame to be the
    // pose of the moving_frame at the beginning of the motion. This results in
    // motion relative to moving_frame's local frame.
    reference_id = moving_id;
  }

  eigenmath::Quaterniond reference_r_offset =
      FromProto(relative_constraint.relative_rotation());

  // We can view reference_r_offset as a transformation that takes any
  // orientation "reference_r_input" as input, and outputs "reference_r_output"
  // with the desired relative rotation applied. In order to apply this to our
  // moving_frame, we need to first represent the moving frame w.r.t. the
  // reference frame:
  eigenmath::Quaterniond reference_r_moving =
      world.GetEntityWorld().GetTransform(reference_id, moving_id).quaternion();

  // Then we apply the transformation to get the rotated (goal) frame, still
  // w.r.t. the reference:
  eigenmath::Quaterniond reference_r_goal =
      reference_r_offset * reference_r_moving;

  // Finally, we view the goal frame w.r.t. the moving frame as required by our
  // constraint proto:
  eigenmath::Quaterniond moving_r_reference = reference_r_moving.inverse();
  eigenmath::Quaterniond moving_r_goal = moving_r_reference * reference_r_goal;

  intrinsic_proto::motion_planning::v1::RotationEquality result_proto;
  *result_proto.mutable_moving_frame() = relative_constraint.moving_frame();
  *result_proto.mutable_target_frame() = relative_constraint.moving_frame();
  *result_proto.mutable_rotation_offset() = ToProto(moving_r_goal);

  return result_proto;
}

absl::StatusOr<PositionAndRotationEquality> ConvertRelativePoseConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::RelativePoseEquality&
        relative_pose_constraint) {
  PositionAndRotationEquality result;

  intrinsic_proto::motion_planning::v1::RelativePositionEquality
      relative_position;
  *relative_position.mutable_moving_frame() =
      relative_pose_constraint.moving_frame();
  if (relative_pose_constraint.has_reference_frame()) {
    *relative_position.mutable_reference_frame() =
        relative_pose_constraint.reference_frame();
  }

  *relative_position.mutable_relative_position() =
      relative_pose_constraint.relative_pose().position();

  INTR_ASSIGN_OR_RETURN(
      result.position_equality,
      ConvertRelativePositionConstraint(world, relative_position));

  intrinsic_proto::motion_planning::v1::RelativeRotationEquality
      relative_rotation;
  *relative_rotation.mutable_moving_frame() =
      relative_pose_constraint.moving_frame();
  if (relative_pose_constraint.has_reference_frame()) {
    *relative_rotation.mutable_reference_frame() =
        relative_pose_constraint.reference_frame();
  }
  *relative_rotation.mutable_relative_rotation() =
      relative_pose_constraint.relative_pose().orientation();

  INTR_ASSIGN_OR_RETURN(
      result.rotation_equality,
      ConvertRelativeRotationConstraint(world, relative_rotation));

  return result;
}

absl::StatusOr<intrinsic_proto::motion_planning::v1::PoseEquality>
ConvertRelativePoseConstraintToPoseConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::RelativePoseEquality&
        relative_pose_constraint) {
  INTR_ASSIGN_OR_RETURN(
      PositionAndRotationEquality pos_and_rot,
      ConvertRelativePoseConstraint(world, relative_pose_constraint));

  intrinsic_proto::motion_planning::v1::PoseEquality pose_constraint;
  *pose_constraint.mutable_moving_frame() =
      relative_pose_constraint.moving_frame();
  *pose_constraint.mutable_target_frame() =
      relative_pose_constraint.moving_frame();

  intrinsic_proto::Pose* pose = pose_constraint.mutable_target_frame_offset();
  *pose->mutable_position() =
      pos_and_rot.position_equality.target_frame_offset();
  *pose->mutable_orientation() =
      pos_and_rot.rotation_equality.rotation_offset();

  return pose_constraint;
}

absl::StatusOr<std::vector<std::unique_ptr<ConstraintInterface>>>
GetConstraintsFromProto(const object_world::ObjectWorld& world,
                        const object_world::KinematicObject& robot,
                        const GeometricConstraint& geometric_constraint) {
  constexpr double kDefaultTolerance = 1e-5;
  constexpr double kDefaultSquaredTolerance = 1e-9;

  std::vector<std::unique_ptr<ConstraintInterface>> resolved_constraints;

  std::queue<GeometricConstraint> queue;
  queue.push(geometric_constraint);

  while (!queue.empty()) {
    const auto constraint = queue.front();
    queue.pop();
    switch (constraint.constraint_case()) {
      case GeometricConstraint::ConstraintCase::kConstraintIntersection: {
        // For an intersection we can add all constraints on the queue.
        for (const auto& element :
             constraint.constraint_intersection().constraints()) {
          queue.push(element);
        }
        break;
      }
      case GeometricConstraint::ConstraintCase::kJointPositionLimits: {
        INTR_ASSIGN_OR_RETURN(
            auto joint_limit_constraint,
            motion_planning::JointPositionLimitsConstraint::Create(
                world, constraint.joint_position_limits(), 0));
        resolved_constraints.push_back(std::move(joint_limit_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kJointPosition: {
        auto joint_position =
            RepeatedDoubleToVectorXd(constraint.joint_position().joints());
        intrinsic_proto::motion_planning::v1::JointPositionEquality
            joint_position_constraint_proto;
        joint_position_constraint_proto.mutable_object_id()->set_id(
            robot.GetId().value());
        *joint_position_constraint_proto.mutable_joint_positions() =
            constraint.joint_position();
        INTR_ASSIGN_OR_RETURN(
            auto joint_position_constraint,
            motion_planning::JointPositionLimitsConstraint::Create(
                world, joint_position_constraint_proto));
        resolved_constraints.push_back(std::move(joint_position_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kCartesianPose: {
        INTR_ASSIGN_OR_RETURN(auto pose_constraints,
                              motion_planning::CreatePoseEquality(
                                  world, constraint.cartesian_pose(),
                                  kDefaultSquaredTolerance, kDefaultTolerance));
        for (auto& elements : pose_constraints) {
          resolved_constraints.push_back(std::move(elements));
        }
        break;
      }
      case GeometricConstraint::ConstraintCase::kPositionEquality: {
        INTR_ASSIGN_OR_RETURN(
            auto position_constraint,
            motion_planning::PositionEqualityConstraint::Create(
                world, constraint.position_equality(),
                kDefaultSquaredTolerance));
        resolved_constraints.push_back(std::move(position_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kRotationCone: {
        INTR_ASSIGN_OR_RETURN(
            auto cone_constraint,
            motion_planning::RotationConeConstraint::Create(
                world, constraint.rotation_cone(), kDefaultTolerance));
        resolved_constraints.push_back(std::move(cone_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kRotationEquality: {
        INTR_ASSIGN_OR_RETURN(
            auto rotation_constraint,
            motion_planning::RotationEqualityConstraint::Create(
                world, constraint.rotation_equality(), kDefaultTolerance));
        resolved_constraints.push_back(std::move(rotation_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kPositionBoundingBox: {
        INTR_ASSIGN_OR_RETURN(
            auto position_bounding_box_constraint,
            motion_planning::PositionBoundingBoxConstraint::Create(
                world, constraint.position_bounding_box()));
        resolved_constraints.push_back(
            std::move(position_bounding_box_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kJointPositionSumLimit: {
        INTR_ASSIGN_OR_RETURN(
            auto joint_position_sum_limit_constraint,
            motion_planning::JointPositionSumLimitConstraint::Create(
                world, constraint.joint_position_sum_limit(),
                kDefaultTolerance));
        resolved_constraints.push_back(
            std::move(joint_position_sum_limit_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kPointAt: {
        INTR_ASSIGN_OR_RETURN(auto point_at_constraint,
                              motion_planning::PointAtConstraint::Create(
                                  world, constraint.point_at()));
        resolved_constraints.push_back(std::move(point_at_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kRelativePositionEquality: {
        INTR_ASSIGN_OR_RETURN(
            auto position_constraint_proto,
            ConvertRelativePositionConstraint(
                world, constraint.relative_position_equality()));
        INTR_ASSIGN_OR_RETURN(
            auto position_constraint,
            motion_planning::PositionEqualityConstraint::Create(
                world, position_constraint_proto));
        resolved_constraints.push_back(std::move(position_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kRelativeRotationEquality: {
        INTR_ASSIGN_OR_RETURN(
            auto rotation_constraint_proto,
            ConvertRelativeRotationConstraint(
                world, constraint.relative_rotation_equality()));
        INTR_ASSIGN_OR_RETURN(
            auto rotation_constraint,
            motion_planning::RotationEqualityConstraint::Create(
                world, rotation_constraint_proto));
        resolved_constraints.push_back(std::move(rotation_constraint));
        break;
      }
      case GeometricConstraint::ConstraintCase::kRelativeCartesianPose: {
        INTR_ASSIGN_OR_RETURN(auto constraints_proto,
                              ConvertRelativePoseConstraint(
                                  world, constraint.relative_cartesian_pose()));

        INTR_ASSIGN_OR_RETURN(
            auto position_constraint,
            motion_planning::PositionEqualityConstraint::Create(
                world, constraints_proto.position_equality));
        resolved_constraints.push_back(std::move(position_constraint));

        INTR_ASSIGN_OR_RETURN(
            auto rotation_constraint,
            motion_planning::RotationEqualityConstraint::Create(
                world, constraints_proto.rotation_equality));
        resolved_constraints.push_back(std::move(rotation_constraint));

        break;
      }

      case GeometricConstraint::CONSTRAINT_NOT_SET: {
        return absl::InvalidArgumentError(
            "Constraint type unknown or unspecified.");
      }
    }
  }
  return std::move(resolved_constraints);
}

absl::StatusOr<std::vector<GeometricConstraint>> GetGeometricConstraintFrom(
    const intrinsic_proto::motion_planning::v1::UniformGeometricConstraint&
        uniform_geometric_constraints) {
  std::vector<GeometricConstraint> geometric_constraints;
  if (uniform_geometric_constraints.ByteSizeLong() == 0) {
    LOG(WARNING) << "Empty uniform_geometric_constraints passed. Returning "
                    "empty vector.";
    return geometric_constraints;
  }

  // Using queue as the container to allow multiple constraints that might exist
  // in constraint intersection.
  std::queue<UniformGeometricConstraint> queue;
  queue.push(uniform_geometric_constraints);
  while (!queue.empty()) {
    const intrinsic_proto::motion_planning::v1::UniformGeometricConstraint
        constraint = queue.front();
    queue.pop();
    switch (constraint.constraint_case()) {
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kUniformGeometricConstraintIntersection: {
        for (const intrinsic_proto::motion_planning::v1::
                 UniformGeometricConstraint& element :
             constraint.uniform_geometric_constraint_intersection()
                 .constraints()) {
          queue.push(element);
        }
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kRotationCone: {
        GeometricConstraint temp_rotation_cone_constraint;
        *temp_rotation_cone_constraint.mutable_rotation_cone() =
            constraint.rotation_cone();
        geometric_constraints.push_back(temp_rotation_cone_constraint);
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kPositionBoundingBox: {
        GeometricConstraint temp_position_bounding_box_constraint;
        *temp_position_bounding_box_constraint.mutable_position_bounding_box() =
            constraint.position_bounding_box();
        geometric_constraints.push_back(temp_position_bounding_box_constraint);
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kJointPositionSumLimit: {
        GeometricConstraint temp_joint_position_sum_limit_constraint;
        *temp_joint_position_sum_limit_constraint
             .mutable_joint_position_sum_limit() =
            constraint.joint_position_sum_limit();
        geometric_constraints.push_back(
            temp_joint_position_sum_limit_constraint);
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          ConstraintCase::kPointAt: {
        GeometricConstraint temp_point_at_constraint;
        *temp_point_at_constraint.mutable_point_at() = constraint.point_at();
        geometric_constraints.push_back(temp_point_at_constraint);
        break;
      }
      case intrinsic_proto::motion_planning::v1::UniformGeometricConstraint::
          CONSTRAINT_NOT_SET: {
        return absl::InvalidArgumentError(
            "Constraint type unknown or unspecified.");
      }
      default: {
        return absl::InvalidArgumentError(
            "Cannot get GeometricConstraint: Un-identified "
            "UniformGeometricConstraint provided.");
      }
    }
  }
  return std::move(geometric_constraints);
}

absl::Status ValidatePositionBoundingBoxConstraintAgainstBasePPoint(
    const motion_planning::PositionBoundingBoxConstraint&
        position_bounding_box_constraint,
    const eigenmath::Vector3d& base_p_point) {
  INTR_ASSIGN_OR_RETURN(
      bool is_satisfied,
      position_bounding_box_constraint.IsSatisfiedPoint3d(base_p_point));
  if (!is_satisfied) {
    return absl::InvalidArgumentError(
        "The `base_p_point` does not satisfy the "
        "`position_bounding_box_constraint`.");
  }
  return absl::OkStatus();
}

absl::StatusOr<eigenmath::Vector3d>
GetGeometricConstraintRepresentativeBasePPoint(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint,
    const Pose3d& flange_t_tool) {
  INTR_ASSIGN_OR_RETURN(
      std::vector<std::unique_ptr<ConstraintInterface>> resolved_constraints,
      GetConstraintsFromProto(world, robot, geometric_constraint));
  std::optional<eigenmath::Vector3d> resolved_base_p_point = std::nullopt;
  const motion_planning::PositionBoundingBoxConstraint*
      position_bounding_box_constraint = nullptr;
  for (const auto& constraint : resolved_constraints) {
    if (constraint->Name() == "PositionEqualityConstraint") {
      const motion_planning::PositionEqualityConstraint* const
          position_constraint = static_cast<
              const motion_planning::PositionEqualityConstraint* const>(
              constraint.get());
      // TODO(b/444227141): Clarify the `moving_frame` used in the constraint,
      // whether it is the `flange` or `tip` or `tool` frame. The pose offset
      // `flange_t_tool` needs to match those of every constraint involved, to
      // ensure consistency across all constraints.
      if (flange_t_tool.translation() != position_constraint->TipPPoint()) {
        return absl::InvalidArgumentError(absl::StrCat(
            "The `flange_t_tool` offset does not match the one specified in "
            "the ",
            constraint->Name(), ", with norm difference: ",
            (flange_t_tool.translation() - position_constraint->TipPPoint())
                .norm()));
      }

      if (!resolved_base_p_point.has_value()) {
        resolved_base_p_point = position_constraint->BasePPoint();
      } else {
        if (resolved_base_p_point.value() !=
            position_constraint->BasePPoint()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "The constraint is an intersection of multiple "
              "`GeometricConstraint`s with conflicting `base_p_point`s. The "
              "norm difference between the resolved `base_p_point` so far and "
              "the unresolved `base_p_point` of ",
              constraint->Name(), " is: ",
              (resolved_base_p_point.value() -
               position_constraint->BasePPoint())
                  .norm()));
        }
      }

      // At this point, we have a resolved `base_p_point` and we need to check
      // if it satisfies the previous `PositionBoundingBoxConstraint` (if there
      // is already one).
      if (position_bounding_box_constraint != nullptr) {
        INTR_RETURN_IF_ERROR(
            ValidatePositionBoundingBoxConstraintAgainstBasePPoint(
                *position_bounding_box_constraint,
                resolved_base_p_point.value()));
      }
    } else if (constraint->Name() == "PositionBoundingBoxConstraint") {
      if (position_bounding_box_constraint != nullptr) {
        // For now, we only support a single `PositionBoundingBoxConstraint`
        // constraint.
        return absl::InvalidArgumentError(
            "Multiple `PositionBoundingBoxConstraint` constraints are not "
            "supported.");
      }
      position_bounding_box_constraint = static_cast<
          const motion_planning::PositionBoundingBoxConstraint* const>(
          constraint.get());
      if (flange_t_tool.translation() !=
          position_bounding_box_constraint->TargetPPoint()) {
        return absl::InvalidArgumentError(
            absl::StrCat("The `flange_t_tool` offset does not match the one "
                         "specified in the ",
                         constraint->Name(), ", with norm difference: ",
                         (flange_t_tool.translation() -
                          position_bounding_box_constraint->TargetPPoint())
                             .norm()));
      }
      if (resolved_base_p_point.has_value()) {
        INTR_RETURN_IF_ERROR(
            ValidatePositionBoundingBoxConstraintAgainstBasePPoint(
                *position_bounding_box_constraint,
                resolved_base_p_point.value()));
      }
    } else if (constraint->Name() == "PointAtConstraint") {
      return absl::UnimplementedError(
          "PointAtConstraint is not supported yet.");
    } else {
      continue;
    }
  }
  if (!resolved_base_p_point.has_value()) {
    if (position_bounding_box_constraint != nullptr) {
      const eigenmath::Vector3d center_point_between_bounds =
          0.5 * (position_bounding_box_constraint->LowerBounds() +
                 position_bounding_box_constraint->UpperBounds());
      const eigenmath::Vector3d base_p_bbox_center =
          position_bounding_box_constraint->ReferenceTBbox() *
          center_point_between_bounds;
      return base_p_bbox_center;
    } else {
      return absl::InvalidArgumentError(
          "Neither `PositionEqualityConstraint` nor "
          "`PositionBoundingBoxConstraint` constraint was found.");
    }
  }
  return resolved_base_p_point.value();
}

}  // namespace intrinsic
