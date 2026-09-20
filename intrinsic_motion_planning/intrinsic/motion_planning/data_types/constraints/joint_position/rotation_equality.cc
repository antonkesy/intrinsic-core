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

#include "intrinsic/motion_planning/data_types/constraints/joint_position/rotation_equality.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/numopt/function_linearizer_numdiff.h"
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

using ::intrinsic_proto::motion_planning::v1::RotationEquality;

absl::StatusOr<std::unique_ptr<RotationEqualityConstraint>>
RotationEqualityConstraint::Create(const object_world::ObjectWorld& world,
                                   const RotationEquality& constraint,
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
  // the world root and the moving frame. If a robot exists, update the
  // reference and reference offset.
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

  INTR_ASSIGN_OR_RETURN(const WorldEntity* reference_entity,
                        world.GetEntityWorld().GetEntityById(reference_id));
  const std::string reference_element_name =
      CreateKinematicElementName(reference_id, *reference_entity);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const kinematics::Element* base_element,
      kinematic_model->GetElement(kinematic_model->GetBaseId()));

  eigenmath::Quaterniond base_r_tip;
  if (constraint.has_rotation_offset()) {
    base_r_tip = FromProto(constraint.rotation_offset());
  } else {
    base_r_tip = eigenmath::Quaterniond::Identity();
  }

  // The frames could be swapped in the model to preserve the order of joints
  // in DOF views of the `world`.
  // TODO(b/261875923): Remove this if/else once World::BuildChainSkeleton
  // preserves frame orders.
  if (base_element->GetName() == reference_element_name) {
    base_r_tip = root_t_reference.quaternion() * base_r_tip;
  } else {
    base_r_tip = base_r_tip.inverse();
  }

  // Use `WrapUnique` because the constructor is private.
  return absl::WrapUnique(new RotationEqualityConstraint(
      std::move(kinematic_model), base_r_tip, tolerance));
}

RotationEqualityConstraint::RotationEqualityConstraint(
    std::unique_ptr<kinematics::ModelInterface> kinematic_model,
    const eigenmath::Quaterniond& base_r_tip, const double tolerance)
    : kinematic_model_(std::move(kinematic_model)),
      kinematic_state_(kinematics::State(kinematic_model_.get())),
      tip_element_id_(kinematic_model_->GetTipIds().front()),
      base_r_tip_(base_r_tip),
      tolerance_(eigenmath::VectorXd::Constant(kConstraintDim, tolerance)) {}

absl::StatusOr<eigenmath::VectorXd> RotationEqualityConstraint::Evaluate(
    const eigenmath::VectorXd& joint_positions) {
  INTRINSIC_RT_RETURN_IF_ERROR(kinematic_state_.SetDofPositions(
      joint_positions, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d base_t_tip,
                                kinematic_state_.GetTransform(tip_element_id_));

  return eigenmath::VectorXd::Constant(
      kConstraintDim, base_r_tip_.angularDistance(base_t_tip.quaternion()));
}

absl::StatusOr<eigenmath::MatrixXd> RotationEqualityConstraint::Gradient(
    const eigenmath::VectorXd& joint_positions) {
  return ComputeDerivative(
      // The num-diff linearizer does not support status codes, so Evaluate
      // must be wrapped in a lambda.
      [this](const eigenmath::VectorXd& inner_joint_positions) {
        auto value_or = this->Evaluate(inner_joint_positions);
        CHECK_OK(value_or.status());
        return *value_or;
      },
      joint_positions,
      /*double_sided_derivative=*/false);
}

absl::StatusOr<bool> RotationEqualityConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_positions) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_positions));
  // No need to compute the absolute value of the residual because angular
  // distances are always positive.
  return (residual.array() <= tolerance_.array()).all();
}

}  // namespace motion_planning
}  // namespace intrinsic
