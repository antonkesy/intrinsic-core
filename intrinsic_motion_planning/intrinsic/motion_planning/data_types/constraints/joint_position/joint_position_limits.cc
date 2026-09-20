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

#include "intrinsic/motion_planning/data_types/constraints/joint_position/joint_position_limits.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/math/numopt/constraint_interface.h"
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

using ::intrinsic_proto::motion_planning::v1::JointPositionLimits;

absl::StatusOr<std::unique_ptr<JointPositionLimitsConstraint>>
JointPositionLimitsConstraint::Create(const object_world::ObjectWorld& world,
                                      const JointPositionLimits& constraint,
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

  if (constraint.lower_limits_size() > 0 &&
      constraint.lower_limits_size() != joint_ids.size()) {
    return absl::InvalidArgumentError(
        "The number of joints does not equal the number of lower limits.");
  }

  if (constraint.upper_limits_size() > 0 &&
      constraint.upper_limits_size() != joint_ids.size()) {
    return absl::InvalidArgumentError(
        "The number of joints does not equal the number of upper limits.");
  }

  INTR_ASSIGN_OR_RETURN(const auto dof_view,
                        world.GetEntityWorld().GetDofKinematicView(joint_ids));
  const auto [world_lower_limits, world_upper_limits] =
      dof_view->GetDofValueApplicationLimits();

  eigenmath::VectorXd lower_limits = world_lower_limits;
  eigenmath::VectorXd upper_limits = world_upper_limits;

  // Merge the `JointPositionLimits` with the joint limits stored in the world.
  for (int i = 0; i < joint_ids.size(); i++) {
    if (constraint.lower_limits_size() > 0) {
      if (world_upper_limits(i) < constraint.lower_limits(i)) {
        return absl::InvalidArgumentError(
            absl::StrCat("The lower limit (", constraint.lower_limits(i),
                         ") exceeds the world's upper limit (",
                         world_upper_limits(i), ") for the joint with index ",
                         i, " and ID = ", joint_ids[i].value()));
      }

      lower_limits(i) =
          std::max(world_lower_limits(i), constraint.lower_limits(i));
    }

    if (constraint.upper_limits_size() > 0) {
      if (world_lower_limits(i) > constraint.upper_limits(i)) {
        return absl::InvalidArgumentError(
            absl::StrCat("The upper limit (", constraint.upper_limits(i),
                         ") exceeds the world's lower limit (",
                         world_lower_limits(i), ") for the joint with index ",
                         i, " and ID = ", joint_ids[i].value()));
      }

      upper_limits(i) =
          std::min(world_upper_limits(i), constraint.upper_limits(i));
    }

    if (lower_limits(i) > upper_limits(i)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "The lower limit (", lower_limits(i), ") exceeds the upper limit (",
          upper_limits(i), ") for the joint with index ", i,
          " and ID = ", joint_ids[i].value()));
    }
  }

  // Use `WrapUnique` because the constructor is private.
  return absl::WrapUnique(
      new JointPositionLimitsConstraint(lower_limits, upper_limits, tolerance));
}

absl::StatusOr<std::unique_ptr<JointPositionLimitsConstraint>>
JointPositionLimitsConstraint::Create(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::JointPositionEquality&
        constraint,
    const double tolerance) {
  // Convert the JointPositionEquality into a JointPositionLimits to reuse the
  // constructor above.
  JointPositionLimits limits_constraint;
  if (constraint.has_object_id()) {
    *limits_constraint.mutable_object_id() = constraint.object_id();
  } else {
    *limits_constraint.mutable_joint_ids() = constraint.joint_ids();
  }
  auto positions = constraint.joint_positions().joints();
  *limits_constraint.mutable_lower_limits() = positions;
  *limits_constraint.mutable_upper_limits() =
      constraint.joint_positions().joints();
  return Create(world, limits_constraint, tolerance);
}

JointPositionLimitsConstraint::JointPositionLimitsConstraint(
    const eigenmath::VectorXd& lower_limits,
    const eigenmath::VectorXd& upper_limits, const double tolerance)
    : lower_limits_(lower_limits),
      upper_limits_(upper_limits),
      tolerance_vector_(
          eigenmath::VectorXd::Constant(lower_limits.size() * 2, tolerance)),
      tolerance_(tolerance) {
  const int ndof = lower_limits_.size();
  const eigenmath::MatrixXd identity =
      eigenmath::MatrixXd::Identity(ndof, ndof);
  gradient_ = eigenmath::MatrixXd(2 * ndof, ndof);
  gradient_ << -identity, identity;

  // isApprox returns false positives when the limits contain infinities.
  if ((upper_limits_ - lower_limits).maxCoeff() <
      Eigen::NumTraits<double>::dummy_precision()) {
    type_ = ConstraintInterface::GENERAL_EQUALITY;
  } else {
    type_ = ConstraintInterface::GENERAL_INEQUALITY;
  }
}

absl::StatusOr<eigenmath::VectorXd> JointPositionLimitsConstraint::Evaluate(
    const eigenmath::VectorXd& joint_positions) {
  if (joint_positions.size() != lower_limits_.size()) {
    return absl::InvalidArgumentError(kNumberOfJointsError);
  }
  eigenmath::VectorXd residual(ConstraintDimension());
  residual << (lower_limits_.array() - joint_positions.array()),
      (joint_positions.array() - upper_limits_.array());
  return residual;
}

absl::StatusOr<eigenmath::MatrixXd> JointPositionLimitsConstraint::Gradient(
    const eigenmath::VectorXd& joint_positions) {
  if (joint_positions.size() != lower_limits_.size()) {
    return absl::InvalidArgumentError(kNumberOfJointsError);
  }
  return gradient_;
}

absl::StatusOr<bool> JointPositionLimitsConstraint::IsSatisfied(
    const eigenmath::VectorXd& joint_positions) {
  if (joint_positions.size() != lower_limits_.size()) {
    return absl::InvalidArgumentError(kNumberOfJointsError);
  }
  return (
      (lower_limits_.array() - tolerance_ <= joint_positions.array()).all() &&
      (joint_positions.array() <= upper_limits_.array() + tolerance_).all());
}

std::vector<eigenmath::VectorXd>
JointPositionLimitsConstraint::SampleJointPositions(int num_samples) {
  // Only return one sample if there is only one solution.
  if (type_ == ConstraintInterface::GENERAL_EQUALITY) {
    return {lower_limits_};
  }

  std::vector<eigenmath::VectorXd> samples(num_samples);
  for (int k = 0; k < num_samples; ++k) {
    // The limits were verified in the constructor, so no need to verify them
    // again with every sample.
    samples[k] = eigenmath::UnverifiedGetUniformRandomVector(
        lower_limits_, upper_limits_, gen_);
  }
  return samples;
}

}  // namespace motion_planning
}  // namespace intrinsic
