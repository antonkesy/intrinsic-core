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

#include "intrinsic/motion_planning/data_types/constraints/joint_position/position_equality.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
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

namespace intrinsic {
namespace motion_planning {

using ::intrinsic_proto::motion_planning::v1::PositionEquality;

absl::StatusOr<std::unique_ptr<PositionEqualityConstraint>>
PositionEqualityConstraint::Create(const object_world::ObjectWorld& world,
                                   const PositionEquality& constraint,
                                   const double tolerance) {
  if (tolerance < 0.0) {
    return absl::InvalidArgumentError("Tolerance cannot be negative.");
  }

  INTR_ASSIGN_OR_RETURN(
      const object_world::TransformNode* transform_node,
      GetTransformNodeByReference(world, constraint.moving_frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId moving_id,
                        transform_node->GetTransformOriginEntityId());

  INTR_ASSIGN_OR_RETURN(transform_node, GetTransformNodeByReference(
                                            world, constraint.target_frame()));
  INTR_ASSIGN_OR_RETURN(AttachmentEntityId reference_id,
                        transform_node->GetTransformOriginEntityId());

  // When reference and moving are the same, check if a robot exists between
  // the world root and the moving. If a robot exists, update the reference and
  // reference offset.
  Pose3d root_t_reference = Pose3d::Identity();
  if (reference_id == moving_id) {
    INTR_ASSIGN_OR_RETURN(
        auto robot_ids,
        world.GetEntityWorld().GetRobotIdsInChain(kRootEntityId, moving_id));

    if (robot_ids.empty()) {
      return absl::NotFoundError(
          "Reference and moving frames are the same but no robot exists "
          "between the World root and the moving frame.");
    }

    reference_id = kRootEntityId;
    root_t_reference =
        world.GetEntityWorld().GetTransform(kRootEntityId, moving_id);
  }

  // Construct the kinematic chain needed to evaluate forward kinematics.
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<kinematics::ModelInterface> kinematic_model,
      world.GetEntityWorld().BuildChainSkeleton(reference_id, moving_id));

  // Assign the points to the frames of the kinematic chain.
  // The frames could be swapped in the model to preserve the order of joints
  // in DOF views of the `world`.
  eigenmath::Vector3d base_p_point;
  eigenmath::Vector3d tip_p_point;

  INTR_ASSIGN_OR_RETURN(const WorldEntity* reference_entity,
                        world.GetEntityWorld().GetEntityById(reference_id));
  const std::string reference_element_name =
      CreateKinematicElementName(reference_id, *reference_entity);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const kinematics::Element* base_element,
      kinematic_model->GetElement(kinematic_model->GetBaseId()));

  // TODO(b/261875923): Remove this if/else once World::BuildChainSkeleton
  // preserves frame orders.
  if (base_element->GetName() == reference_element_name) {
    tip_p_point = FromProto(constraint.moving_frame_offset());
    base_p_point =
        root_t_reference * FromProto(constraint.target_frame_offset());
  } else {
    tip_p_point = FromProto(constraint.target_frame_offset());
    base_p_point = FromProto(constraint.moving_frame_offset());
  }

  // Use `WrapUnique` because the constructor is private.
  return absl::WrapUnique(new PositionEqualityConstraint(
      std::move(kinematic_model), base_p_point, tip_p_point, tolerance));
}

PositionEqualityConstraint::PositionEqualityConstraint(
    std::unique_ptr<kinematics::ModelInterface> kinematic_model,
    const eigenmath::Vector3d& base_p_point,
    const eigenmath::Vector3d& tip_p_point, const double tolerance)
    : kinematic_model_(std::move(kinematic_model)),
      kinematic_state_(kinematics::State(kinematic_model_.get())),
      tip_element_id_(kinematic_model_->GetTipIds().front()),
      base_p_point_(base_p_point),
      tip_p_point_(tip_p_point),
      tolerance_(eigenmath::VectorXd::Constant(kConstraintDim, tolerance)) {}

absl::StatusOr<eigenmath::VectorXd> PositionEqualityConstraint::Evaluate(
    const eigenmath::VectorXd& joint_positions) {
  INTRINSIC_RT_RETURN_IF_ERROR(kinematic_state_.SetDofPositions(
      joint_positions, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d base_t_tip,
                                kinematic_state_.GetTransform(tip_element_id_));

  const eigenmath::Vector3d base_p_tip_point = base_t_tip * tip_p_point_;

  // The squared distance is faster to compute, particularly within the
  // `Gradient` method. Benchmarks as of January 2023 indicate that returning
  // a 1-d residual containing the squared distance is preferable to returning
  // a 3-d residual with the distance from each position dimension.
  return eigenmath::VectorXd::Constant(
      kConstraintDim, (base_p_tip_point - base_p_point_).squaredNorm());
}

absl::StatusOr<eigenmath::MatrixXd> PositionEqualityConstraint::Gradient(
    const eigenmath::VectorXd& joint_positions) {
  INTRINSIC_RT_RETURN_IF_ERROR(kinematic_state_.SetDofPositions(
      joint_positions, /*check_limits=*/false));

  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d base_t_tip,
                                kinematic_state_.GetTransform(tip_element_id_));
  const eigenmath::Vector3d base_p_tip_point = base_t_tip * tip_p_point_;

  const eigenmath::Vector3d point_difference = base_p_tip_point - base_p_point_;

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const eigenmath::Matrix6Nd jacobian,
      kinematic_state_.ComputeJacobian(tip_element_id_, tip_p_point_));

  return (2 * point_difference.transpose()) * jacobian.topRows(3);

  // For future benchmark comparisons, this is the code for when `Evaluate`
  // returns the distance between the points instead of the squared distance:
  // const double residual = point_difference.norm();
  // return point_difference.transpose() * jacobian.topRows(3) / residual;
}

absl::StatusOr<bool> PositionEqualityConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_positions) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_positions));
  return (residual.array() <= tolerance_.array()).all();
}

}  // namespace motion_planning
}  // namespace intrinsic
