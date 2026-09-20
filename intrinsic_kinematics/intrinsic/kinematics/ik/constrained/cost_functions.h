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

#ifndef INTRINSIC_KINEMATICS_IK_CONSTRAINED_COST_FUNCTIONS_H_
#define INTRINSIC_KINEMATICS_IK_CONSTRAINED_COST_FUNCTIONS_H_

#include <math.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/manifolds.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik_solvers/ik_solver_utils.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/numopt/costfunction_interface.h"
#include "intrinsic/math/numopt/function_linearizer_numdiff.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kinematics {

// Implements a quadratic cost function of the form 'J(x) = 0.5 * dx.transpose()
// * Q * dx' where 'dx' is defined as 'x - x_ref'.
class JointPositionCost : public CostFunctionInterface {
 public:
  // Creates JointPositionCost from a 'chain', a reference joint position
  // 'x_ref' and a single scalar value as cost 'weight'. Returns
  // 'kFailedPrecondition' in case of dimension mismatch or negative cost
  // weight.
  static absl::StatusOr<std::unique_ptr<JointPositionCost>> Create(
      const Chain* chain, const JointStateP& x_ref, double weight = 1.0);

  // Creates JointPositionCost from a 'chain', a reference joint position
  // 'x_ref' and an n x n positive semi-definite matrix as cost function
  // 'weight'. Returns 'kFailedPrecondition' in case of dimension mismatch or
  // negative definite cost weight.
  static absl::StatusOr<std::unique_ptr<JointPositionCost>> Create(
      const Chain* chain, const JointStateP& x_ref,
      const eigenmath::MatrixXd& weight);

  absl::StatusOr<double> Evaluate(
      const eigenmath::VectorXd& joint_angles) override;

  absl::StatusOr<eigenmath::VectorXd> Gradient(
      const eigenmath::VectorXd& joint_angles) override;

 private:
  explicit JointPositionCost(const eigenmath::MatrixXd& weight,
                             const JointStateP& x_ref);

  eigenmath::VectorXd x_ref_;
  eigenmath::MatrixXd weight_;
};

// Implements a manipulability cost function which maximizes the manipulability
// 'sqrt(det(J.transpose()*J))', where 'J' is the Jacobian matrix. Since cost
// terms need to be designed for minimization instead of maximization, we
// minimize 'exp(-sqrt(det(J.transpose()*J)))' instead. In general the
// manipulability metric tends to favour extremal joint configurations far away
// from "neutral" joint configurations. In order to remain distant from joint
// limits, the cost function is augmented with a penalty for proximity to joint
// limits.
class ManipulabilityCost : public CostFunctionInterface {
 public:
  // Creates a 'ManipulabilityCost' from a 'chain' and a 'weight' scaling
  // factor, evaluating manipulability at the robot frame specified by
  // 'robot_frame_id'. Returns kFailedPrecondition in case of negative weight.
  static absl::StatusOr<std::unique_ptr<ManipulabilityCost>> Create(
      const Chain* chain, ElementId robot_frame_id, double weight = 1.0);

  absl::StatusOr<double> Evaluate(
      const eigenmath::VectorXd& joint_angles) override;

  absl::StatusOr<eigenmath::VectorXd> Gradient(
      const eigenmath::VectorXd& joint_angles) override;

 private:
  static constexpr double kDefaultSoftminAlpha = -10.0;  // must be negative.

  explicit ManipulabilityCost(const Chain* chain, ElementId robot_frame_id,
                              double weight = 1.0);

  // state_ keeps a pointer to the original Chain, which must outlive the state.
  State state_;
  JointLimits limits_;
  double weight_;
  const ElementId robot_frame_id_;
};

}  // namespace intrinsic::kinematics

#endif  // INTRINSIC_KINEMATICS_IK_CONSTRAINED_COST_FUNCTIONS_H_
