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

#ifndef INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_JOINT_POSITION_SUM_LIMIT_H_
#define INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_JOINT_POSITION_SUM_LIMIT_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {
namespace motion_planning {

// A `JointPositionSumLimit` constraint that applies to a particular `World`.
class JointPositionSumLimitConstraint : public ConstraintInterface {
 public:
  static constexpr absl::string_view kConstraintName =
      "JointPositionSumLimitConstraint";
  static constexpr int kConstraintDim = 1;
  static constexpr double kDefaultTolerance = 1e-4;

  // Binds a world and a proto specification of a joint position limits
  // constraint to construct an instance of this class.
  static absl::StatusOr<std::unique_ptr<JointPositionSumLimitConstraint>>
  Create(const object_world::ObjectWorld& world,
         const intrinsic_proto::motion_planning::v1::JointPositionSumLimit&
             constraint,
         double tolerance = kDefaultTolerance);

  ConstraintType Type() const override { return GENERAL_INEQUALITY; }

  int ConstraintDimension() const override { return kConstraintDim; }

  int DomainDimension() const override { return weights_.size(); }

  absl::string_view Name() const override { return kConstraintName; }

  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_positions) override;

  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_positions) override;

  const eigenmath::VectorXd& Tolerance() const override { return tolerance_; }

  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_positions) override;

 private:
  JointPositionSumLimitConstraint(const eigenmath::VectorXd& weights,
                                  double joint_sum_limit, double tolerance);

  double ComputeWeightedJointPositionsCombination(
      const eigenmath::VectorXd& joint_positions) const;

  eigenmath::VectorXd weights_;  // A vector with its elements being either 0,
                                 // +1, or -1. Used as the weight vector for the
                                 // weighted sum of the joint positions.
  double joint_sum_limit_;
  eigenmath::VectorXd tolerance_;
};

}  // namespace motion_planning
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_JOINT_POSITION_SUM_LIMIT_H_
