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

#ifndef INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_POSITION_EQUALITY_H_
#define INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_POSITION_EQUALITY_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {
namespace motion_planning {

// A `PositionEquality` constraint that applies to Cartesian positions in a
// particular `World`.
class PositionEqualityConstraint : public ConstraintInterface {
 public:
  static constexpr int kConstraintDim = 1;
  static constexpr absl::string_view kConstraintName =
      "PositionEqualityConstraint";
  static constexpr double kDefaultTolerance = 1e-6;  // Units: squared meters

  // Binds a world and a proto specification of a position equality constraint
  // to construct an instance of this class.
  //
  // The returned constraint applies to all joints that lie between the moving
  // and reference frames in the `constraint` proto, and joints are in the same
  // order as in the robot components in the `world`. The `GetKinematicModel`
  // method returns the kinematic model used within this constraint, and it
  // contains the names of all joints in the order that their positions must
  // be provided to methods of this class.
  //
  // The `Evaluate` function returns the squared distance between the points
  // defined in the `constraint`, so `tolerance` has units of squared meters.
  //
  // Returns `kInvalidArgumentError` if the tolerance is negative.
  static absl::StatusOr<std::unique_ptr<PositionEqualityConstraint>> Create(
      const object_world::ObjectWorld& world,
      const intrinsic_proto::motion_planning::v1::PositionEquality& constraint,
      double tolerance = kDefaultTolerance);

  ConstraintType Type() const override { return GENERAL_EQUALITY; }

  int ConstraintDimension() const override { return kConstraintDim; }

  int DomainDimension() const override {
    return kinematic_model_->GetNumberDegreesOfFreedom();
  }

  absl::string_view Name() const override { return kConstraintName; }

  absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& joint_positions) override;

  absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& joint_positions) override;

  const eigenmath::VectorXd& Tolerance() const override { return tolerance_; }

  absl::StatusOr<bool> IsSatisfied(
      const eigenmath::VectorXd& joint_positions) override;

  // Returns the kinematic model that contains all joints to which this
  // constraint applies. Joints in the model are in the order that their
  // positions must be provided to other methods of this class, like `Evaluate`.
  const std::unique_ptr<kinematics::ModelInterface>& GetKinematicModel() const {
    return kinematic_model_;
  }

  const eigenmath::Vector3d& BasePPoint() const { return base_p_point_; }
  const eigenmath::Vector3d& TipPPoint() const { return tip_p_point_; }

 private:
  PositionEqualityConstraint(
      std::unique_ptr<kinematics::ModelInterface> kinematic_model,
      const eigenmath::Vector3d& base_p_point,
      const eigenmath::Vector3d& tip_p_point, double tolerance);

  std::unique_ptr<kinematics::ModelInterface> kinematic_model_;
  kinematics::State kinematic_state_;
  kinematics::ElementId tip_element_id_;
  const eigenmath::Vector3d base_p_point_;
  const eigenmath::Vector3d tip_p_point_;
  const eigenmath::VectorXd tolerance_;
};

}  // namespace motion_planning
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_POSITION_EQUALITY_H_
