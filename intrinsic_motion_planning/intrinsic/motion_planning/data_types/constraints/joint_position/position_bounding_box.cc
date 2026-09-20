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

#include "intrinsic/motion_planning/data_types/constraints/joint_position/position_bounding_box.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "google/protobuf/message.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/kinematics_builder.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace motion_planning {

using eigenmath::MatrixXd;
using eigenmath::Vector3d;
using eigenmath::VectorXd;
using ::intrinsic_proto::motion_planning::v1::PositionBoundingBox;

// Project a point into the bounding box (by clamping each dimension)
Vector3d PositionBoundingBoxConstraint::ClosestPointInBox(
    const Vector3d& point) const {
  Vector3d clamped(point);
  const bool result =
      eigenmath::ClampVector(lower_bounds_, upper_bounds_, clamped);
  CHECK(result);
  return clamped;
}

absl::StatusOr<std::unique_ptr<PositionBoundingBoxConstraint>>
PositionBoundingBoxConstraint::Create(const object_world::ObjectWorld& world,
                                      const PositionBoundingBox& constraint,
                                      const double tolerance) {
  if (tolerance < 0.0) {
    return absl::InvalidArgumentError("Tolerance cannot be negative.");
  }

  INTR_ASSIGN_OR_RETURN(
      const object_world::TransformNode* transform_node,
      GetTransformNodeByReference(world, constraint.moving_frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId target_id,
                        transform_node->GetTransformOriginEntityId());

  INTR_ASSIGN_OR_RETURN(transform_node, GetTransformNodeByReference(
                                            world, constraint.target_frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId reference_id,
                        transform_node->GetTransformOriginEntityId());

  if (target_id == reference_id) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Moving (%s) and target (%s) frames seem to be identical for the "
        "bounding box constraint. This is currently not supported. The target "
        "frame cannot be connected to the moving frame.",
        google::protobuf::ShortFormat(constraint.moving_frame()),
        google::protobuf::ShortFormat(constraint.target_frame())));
  }

  const World& entity_world = world.GetEntityWorld();

  // Construct the kinematic chain needed to evaluate forward kinematics.
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<kinematics::ModelInterface> kinematic_model,
      entity_world.BuildChainSkeleton(reference_id, target_id));

  // Figure out whether the reference or target are the base of the chain
  INTR_ASSIGN_OR_RETURN(const WorldEntity* reference_entity,
                        entity_world.GetEntityById(reference_id));
  const std::string reference_element_name =
      CreateKinematicElementName(reference_id, *reference_entity);
  INTR_ASSIGN_OR_RETURN(const WorldEntity* target_entity,
                        entity_world.GetEntityById(target_id));
  const std::string target_element_name =
      CreateKinematicElementName(reference_id, *target_entity);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const kinematics::Element* base_element,
      kinematic_model->GetElement(kinematic_model->GetBaseId()));
  auto base_name = base_element->GetName();

  if (base_name == target_element_name) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "The target frame (%s) seems to be connected to the moving frame (%s). "
        "This functionality is currently not supported for the bounding box "
        "constraint.",
        google::protobuf::ShortFormat(constraint.target_frame()),
        google::protobuf::ShortFormat(constraint.moving_frame())));
  }
  if (base_name != reference_element_name) {
    return absl::InternalError(absl::StrFormat(
        "Kinematic chain base %s doesn't equal target frame %s or moving frame "
        "%s. Please contact support.",
        base_name, reference_element_name, target_element_name));
  }

  Vector3d moving_frame_offset = Vector3d::Zero();
  if (constraint.has_moving_frame_offset()) {
    moving_frame_offset = FromProto(constraint.moving_frame_offset());
  }

  Pose3d reference_t_bbox_center = Pose3d::Identity();
  if (constraint.has_target_bounding_box_center()) {
    INTR_ASSIGN_OR_RETURN(
        reference_t_bbox_center,
        FromProtoNormalized(constraint.target_bounding_box_center()));
  }

  Vector3d lower_bounds =
      Vector3d::Constant(-std::numeric_limits<double>::infinity());
  if (constraint.has_lower_bounds()) {
    lower_bounds = FromProto(constraint.lower_bounds());
  }

  Vector3d upper_bounds =
      Vector3d::Constant(std::numeric_limits<double>::infinity());
  if (constraint.has_upper_bounds()) {
    upper_bounds = FromProto(constraint.upper_bounds());
  }

  if ((upper_bounds - lower_bounds).minCoeff() < 0) {
    return absl::InvalidArgumentError(
        "At least one of the lower bounds is greater than the upper bounds.");
  }

  return absl::WrapUnique(new PositionBoundingBoxConstraint(
      std::move(kinematic_model), moving_frame_offset, reference_t_bbox_center,
      lower_bounds, upper_bounds, tolerance));
}

PositionBoundingBoxConstraint::PositionBoundingBoxConstraint(
    std::unique_ptr<kinematics::ModelInterface> kinematic_model,
    const Vector3d& target_p_point, const Pose3d& reference_t_bbox,
    const Vector3d& lower_bounds, const Vector3d& upper_bounds,
    const double tolerance)
    : kinematic_model_(std::move(kinematic_model)),
      kinematic_state_(kinematics::State(kinematic_model_.get())),
      tip_element_id_(kinematic_model_->GetTipIds().front()),
      target_p_point_(target_p_point),
      reference_t_bbox_(reference_t_bbox),
      bbox_t_reference_(reference_t_bbox.inverse()),
      lower_bounds_(lower_bounds),
      upper_bounds_(upper_bounds),
      tolerance_(VectorXd::Constant(kConstraintDim, tolerance)) {}

// The cost is the squared distance from the target point to the closest
// point in the bounding box
absl::StatusOr<VectorXd> PositionBoundingBoxConstraint::Evaluate(
    const VectorXd& joint_positions) {
  INTR_ASSIGN_OR_RETURN(const Pose3d reference_t_moving,
                        GetReferenceTTarget(joint_positions));
  const Vector3d reference_p_point = reference_t_moving * target_p_point_;
  return EvaluatePoint3d(reference_p_point);
}

absl::StatusOr<eigenmath::VectorXd>
PositionBoundingBoxConstraint::EvaluatePoint3d(
    const eigenmath::Vector3d& reference_p_point) const {
  const Vector3d bbox_p_point = bbox_t_reference_ * reference_p_point;
  const Vector3d bbox_p_closest = ClosestPointInBox(bbox_p_point);

  const double cost = (bbox_p_point - bbox_p_closest).squaredNorm();

  return eigenmath::VectorXd::Constant(kConstraintDim, cost);
}

absl::StatusOr<MatrixXd> PositionBoundingBoxConstraint::Gradient(
    const VectorXd& joint_positions) {
  eigenmath::Matrix6Nd jacobian;
  INTRINSIC_RT_RETURN_IF_ERROR(kinematic_state_.SetDofPositions(
      joint_positions, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(jacobian, kinematic_state_.ComputeJacobian());

  INTR_ASSIGN_OR_RETURN(const Pose3d reference_t_moving,
                        GetReferenceTTarget(joint_positions));
  const Vector3d bbox_p_point =
      bbox_t_reference_ * reference_t_moving * target_p_point_;
  const Vector3d bbox_p_closest = ClosestPointInBox(bbox_p_point);

  // The cost function increases if you move directly away from the closest
  // point in the bbox.  So find that direction (in the base frame)
  // and multiply by the Jacobian to find the gradient wrt joint angles.
  const Vector3d bbox_v_point_to_closest = bbox_p_closest - bbox_p_point;
  const Vector3d reference_v_point_to_closest =
      reference_t_bbox_.rotationMatrix() * bbox_v_point_to_closest;

  return -2 * reference_v_point_to_closest.transpose() * jacobian.topRows(3);
}

absl::StatusOr<bool> PositionBoundingBoxConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_positions) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_positions));
  return (residual.array() <= tolerance_.array()).all();
}

absl::StatusOr<bool> PositionBoundingBoxConstraint::IsSatisfiedPoint3d(
    const eigenmath::Vector3d& reference_p_point) const {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        EvaluatePoint3d(reference_p_point));
  return (residual.array() <= tolerance_.array()).all();
}

absl::StatusOr<Pose3d> PositionBoundingBoxConstraint::GetReferenceTTarget(
    const VectorXd& joint_positions) const {
  using kinematics::ElementId;
  INTRINSIC_RT_RETURN_IF_ERROR(kinematic_state_.SetDofPositions(
      joint_positions, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d base_t_target,
                                kinematic_state_.GetTransform(tip_element_id_));

  return base_t_target;
}

}  // namespace motion_planning
}  // namespace intrinsic
