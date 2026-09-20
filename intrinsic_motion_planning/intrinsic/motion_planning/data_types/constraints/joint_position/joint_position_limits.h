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

#ifndef INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_JOINT_POSITION_LIMITS_H_
#define INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_JOINT_POSITION_LIMITS_H_

#include <memory>
#include <vector>

#include "absl/random/random.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position_sampler.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {
namespace motion_planning {

// A `JointPositionLimits` constraint that applies to a particular `World`.
class JointPositionLimitsConstraint : public ConstraintInterface,
                                      public JointPositionSamplerInterface {
 public:
  static constexpr absl::string_view kConstraintName =
      "JointPositionLimitsConstraint";
  static constexpr double kDefaultTolerance = 1e-4;

  // Binds a world and a proto specification of a joint position limits
  // constraint to construct an instance of this class.
  static absl::StatusOr<std::unique_ptr<JointPositionLimitsConstraint>> Create(
      const object_world::ObjectWorld& world,
      const intrinsic_proto::motion_planning::v1::JointPositionLimits&
          constraint,
      double tolerance = kDefaultTolerance);

  // Binds a world and a proto specification of a joint position equality
  // constraint to construct an instance of this class.
  static absl::StatusOr<std::unique_ptr<JointPositionLimitsConstraint>> Create(
      const object_world::ObjectWorld& world,
      const intrinsic_proto::motion_planning::v1::JointPositionEquality&
          constraint,
      double tolerance = kDefaultTolerance);

  ConstraintType Type() const override { return type_; }

  int ConstraintDimension() const override { return lower_limits_.size() * 2; }

  int DomainDimension() const override { return lower_limits_.size(); }

  absl::string_view Name() const override { return kConstraintName; }

  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_positions) override;

  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_positions) override;

  const eigenmath::VectorXd& Tolerance() const override {
    return tolerance_vector_;
  }

  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_positions) override;

  std::vector<eigenmath::VectorXd> SampleJointPositions(
      int num_samples) override;

  eigenmath::VectorXd GetUpperLimits() const { return upper_limits_; }
  eigenmath::VectorXd GetLowerLimits() const { return lower_limits_; }

 private:
  JointPositionLimitsConstraint(const eigenmath::VectorXd& lower_limits,
                                const eigenmath::VectorXd& upper_limits,
                                double tolerance);

  ConstraintType type_;
  eigenmath::VectorXd lower_limits_;
  eigenmath::VectorXd upper_limits_;
  eigenmath::VectorXd tolerance_vector_;
  double tolerance_;
  eigenmath::MatrixXd gradient_;

  // A bit generator used for sampling.
  absl::BitGen gen_;
};

}  // namespace motion_planning
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_JOINT_POSITION_LIMITS_H_
