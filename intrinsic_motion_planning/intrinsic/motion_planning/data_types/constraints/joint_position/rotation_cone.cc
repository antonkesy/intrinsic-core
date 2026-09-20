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

#include "intrinsic/motion_planning/data_types/constraints/joint_position/rotation_cone.h"

#include <cmath>
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
#include "intrinsic/math/numopt/constraint_interface.h"
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

using ::intrinsic_proto::motion_planning::v1::RotationCone;

absl::StatusOr<std::unique_ptr<RotationConeConstraint>>
RotationConeConstraint::Create(const object_world::ObjectWorld& world,
                               const RotationCone& constraint,
                               const double tolerance) {
  if (tolerance < 0.0) {
    return absl::InvalidArgumentError("Tolerance cannot be negative.");
  }

  const double cone_opening_half_angle = constraint.cone_opening_half_angle();
  if (cone_opening_half_angle < 0.0) {
    return absl::InvalidArgumentError("Cone opening angle cannot be negative.");
  }

  INTR_ASSIGN_OR_RETURN(
      const object_world::TransformNode* transform_node,
      GetTransformNodeByReference(world, constraint.moving_frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId moving_id,
                        transform_node->GetTransformOriginEntityId());

  INTR_ASSIGN_OR_RETURN(transform_node, GetTransformNodeByReference(
                                            world, constraint.target_frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId reference_id,
                        transform_node->GetTransformOriginEntityId());

  // Construct the kinematic chain needed to evaluate forward kinematics.
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<kinematics::ModelInterface> kinematic_model,
      world.GetEntityWorld().BuildChainSkeleton(reference_id, moving_id));

  // Normalize the axes.
  eigenmath::Vector3d moving_p_axis = FromProto(constraint.moving_axis());
  const double moving_axis_norm = moving_p_axis.norm();
  if (moving_axis_norm < Eigen::NumTraits<double>::dummy_precision()) {
    return absl::InvalidArgumentError(
        "The norm of the moving axis is zero. Please provide an axis with "
        "nonzero norm.");
  }
  moving_p_axis /= moving_axis_norm;

  eigenmath::Vector3d reference_p_axis;
  if (constraint.has_target_axis()) {
    reference_p_axis = FromProto(constraint.target_axis());
    const double target_axis_norm = reference_p_axis.norm();
    if (target_axis_norm < Eigen::NumTraits<double>::dummy_precision()) {
      return absl::InvalidArgumentError(
          "The norm of the reference axis is zero. Please provide an axis with "
          "nonzero norm.");
    }
    reference_p_axis /= target_axis_norm;
  } else {
    reference_p_axis = moving_p_axis;
  }

  // Assign the axes to the frames of the kinematic chain.
  // The axes could be swapped in the skeleton to preserve the order of joints
  // in DOF views of the `world`.
  eigenmath::Vector3d base_p_axis;
  eigenmath::Vector3d tip_p_axis;

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
    tip_p_axis = moving_p_axis;
    base_p_axis = reference_p_axis;
  } else {
    tip_p_axis = reference_p_axis;
    base_p_axis = moving_p_axis;
  }

  // Use `WrapUnique` because the constructor is private.
  return absl::WrapUnique(new RotationConeConstraint(
      std::move(kinematic_model), base_p_axis, tip_p_axis,
      cone_opening_half_angle, tolerance));
}

RotationConeConstraint::RotationConeConstraint(
    std::unique_ptr<kinematics::ModelInterface> kinematic_model,
    const eigenmath::Vector3d& base_p_axis,
    const eigenmath::Vector3d& tip_p_axis, const double cone_opening_half_angle,
    const double tolerance)
    : kinematic_model_(std::move(kinematic_model)),
      kinematic_state_(kinematics::State(kinematic_model_.get())),
      tip_element_id_(kinematic_model_->GetTipIds().front()),
      base_p_axis_(base_p_axis),
      tip_p_axis_(tip_p_axis),
      cos_cone_opening_half_angle_(std::cos(cone_opening_half_angle)),
      tolerance_(eigenmath::VectorXd::Constant(kConstraintDim, tolerance)) {}

ConstraintInterface::ConstraintType RotationConeConstraint::Type() const {
  if (cos_cone_opening_half_angle_ == 1.0) {
    return GENERAL_EQUALITY;
  }
  return GENERAL_INEQUALITY;
}

absl::StatusOr<eigenmath::VectorXd> RotationConeConstraint::Evaluate(
    const eigenmath::VectorXd& joint_positions) {
  INTRINSIC_RT_RETURN_IF_ERROR(kinematic_state_.SetDofPositions(
      joint_positions, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const Pose3d base_t_tip,
                                kinematic_state_.GetTransform(tip_element_id_));

  const eigenmath::Vector3d base_p_tip_axis =
      base_t_tip.rotationMatrix() * tip_p_axis_;

  // TODO(b/263188336): Check that the dot product is preferred over the angle
  // between axes.
  const double cosine_of_axis_angle = base_p_tip_axis.dot(base_p_axis_);
  return eigenmath::VectorXd::Constant(
      kConstraintDim, cos_cone_opening_half_angle_ - cosine_of_axis_angle);
}

absl::StatusOr<eigenmath::MatrixXd> RotationConeConstraint::Gradient(
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

absl::StatusOr<bool> RotationConeConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_positions) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_positions));
  return (residual.array() <= tolerance_.array()).all();
}

}  // namespace motion_planning
}  // namespace intrinsic
