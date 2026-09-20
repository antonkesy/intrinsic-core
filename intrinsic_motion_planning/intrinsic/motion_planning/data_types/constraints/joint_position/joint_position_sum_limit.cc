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

#include "intrinsic/motion_planning/data_types/constraints/joint_position/joint_position_sum_limit.h"

#include <memory>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"

namespace intrinsic {
namespace motion_planning {

namespace {

static constexpr absl::string_view kNumberOfJointsError =
    "The number of given joint positions does not match the number of joints "
    "in this constraint.";
}

using ::intrinsic_proto::motion_planning::v1::JointPositionSumLimit;

absl::StatusOr<std::unique_ptr<JointPositionSumLimitConstraint>>
JointPositionSumLimitConstraint::Create(const object_world::ObjectWorld& world,
                                        const JointPositionSumLimit& constraint,
                                        const double tolerance) {
  std::vector<JointEntityId> joint_ids;
  if (constraint.has_joint_ids()) {
    for (const auto& joint_id : constraint.joint_ids().joint_ids()) {
      joint_ids.push_back(JointEntityId(joint_id));
    }
  } else {
    INTR_ASSIGN_OR_RETURN(
        const object_world::KinematicObject* kinematic_object,
        GetKinematicObjectByReference(world, constraint.object_id()));
    INTR_ASSIGN_OR_RETURN(joint_ids, kinematic_object->GetJointEntityIds());
  }

  // Validate the number of joints in all fields of the `constraint`.
  if (joint_ids.empty()) {
    return absl::InvalidArgumentError("At least one joint must be specified.");
  }

  if (constraint.joint_sum_limit() < 0.0) {
    return absl::InvalidArgumentError("Joint sum limit must be non-negative.");
  }

  if (constraint.joint_signs_size() != joint_ids.size()) {
    return absl::InvalidArgumentError(
        "The number of joints does not equal the number of joint signs.");
  }

  // Extract the joint position weights.
  eigenmath::VectorXd weights = eigenmath::VectorXd::Zero(joint_ids.size());
  for (int i = 0; i < joint_ids.size(); i++) {
    switch (constraint.joint_signs(i)) {
      case intrinsic_proto::motion_planning::v1::JointPositionSumLimit::
          UNSPECIFIED:
        weights(i) = 0.0;
        break;
      case intrinsic_proto::motion_planning::v1::JointPositionSumLimit::
          POSITIVE:
        weights(i) = 1.0;
        break;
      case intrinsic_proto::motion_planning::v1::JointPositionSumLimit::
          NEGATIVE:
        weights(i) = -1.0;
        break;
      default:
        weights(i) = 0.0;
        break;
    }
  }

  // Use `WrapUnique` because the constructor is private.
  return absl::WrapUnique(new JointPositionSumLimitConstraint(
      weights, constraint.joint_sum_limit(), tolerance));
}

JointPositionSumLimitConstraint::JointPositionSumLimitConstraint(
    const eigenmath::VectorXd& weights, double joint_sum_limit,
    double tolerance)
    : weights_(weights),
      joint_sum_limit_(joint_sum_limit),
      tolerance_(eigenmath::VectorXd::Constant(kConstraintDim, tolerance)) {}

double
JointPositionSumLimitConstraint::ComputeWeightedJointPositionsCombination(
    const eigenmath::VectorXd& joint_positions) const {
  return weights_.transpose() * joint_positions;
}

absl::StatusOr<eigenmath::VectorXd> JointPositionSumLimitConstraint::Evaluate(
    const eigenmath::VectorXd& joint_positions) {
  if (joint_positions.size() != weights_.size()) {
    return absl::InvalidArgumentError(kNumberOfJointsError);
  }
  const double weighted_joint_positions_combination =
      ComputeWeightedJointPositionsCombination(joint_positions);
  eigenmath::VectorXd residual = eigenmath::VectorXd::Constant(
      kConstraintDim, intrinsic::IPow(weighted_joint_positions_combination, 2) -
                          intrinsic::IPow(joint_sum_limit_, 2));
  return residual;
}

absl::StatusOr<eigenmath::MatrixXd> JointPositionSumLimitConstraint::Gradient(
    const eigenmath::VectorXd& joint_positions) {
  if (joint_positions.size() != weights_.size()) {
    return absl::InvalidArgumentError(kNumberOfJointsError);
  }
  const double weighted_joint_positions_combination =
      ComputeWeightedJointPositionsCombination(joint_positions);
  return 2.0 * weighted_joint_positions_combination * weights_.transpose();
}

absl::StatusOr<bool> JointPositionSumLimitConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_positions) {
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd residual,
                        Evaluate(joint_positions));
  return (residual.array() <= tolerance_.array()).all();
}

}  // namespace motion_planning
}  // namespace intrinsic
