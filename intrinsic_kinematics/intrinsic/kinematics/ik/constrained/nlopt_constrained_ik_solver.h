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

#ifndef INTRINSIC_KINEMATICS_IK_CONSTRAINED_NLOPT_CONSTRAINED_IK_SOLVER_H_
#define INTRINSIC_KINEMATICS_IK_CONSTRAINED_NLOPT_CONSTRAINED_IK_SOLVER_H_

#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "absl/random/random.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_interface.h"
#include "intrinsic/kinematics/ik/constrained/nlopt/nlopt_constrained_ik.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/numopt/costfunction_interface.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::kinematics {

class NLOptConstrainedIKSolver final : public ConstrainedIKInterface {
 public:
  using InverseKinematicsInterface::ComputeIK;

  static constexpr char kSolverName[] = "NLOptConstrainedIKSolver";

  explicit NLOptConstrainedIKSolver(Chain chain)
      : ConstrainedIKInterface(chain) {}

  int GetNumDof() const override { return chain().GetNumberDegreesOfFreedom(); }

  const std::string_view GetName() const override { return kSolverName; }

  bool ImplementsSameBranchIK() const override { return false; }

  absl::Status SetProblem(
      std::vector<std::unique_ptr<ConstraintInterface>>&& constraints,
      std::vector<std::unique_ptr<CostFunctionInterface>>&& costs) override;

  absl::Status SetProblem(const intrinsic_proto::kinematics::ConstrainedGoal&
                              constrained_goal) override;

  absl::StatusOr<ConstrainedIKInterface::Result> Solve(
      const ConstrainedIKInterface::Options& options) const override;

  absl::StatusOr<IKResult> ComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      absl::Span<JointStateP> solutions) const override;

  // We always return true, regardless of the actual chain.
  bool IsCompatibleWithChain(const Chain& chain) const override { return true; }

  // We always return nullopt.
  std::optional<ElementId> RetrieveMatchingStartJoint(
      const ModelInterface& model, ElementId end_joint) const override {
    return std::nullopt;
  }

 private:
  std::unique_ptr<NLOptConstrainedIK> nlopt_constrained_ik_ = nullptr;
};

}  // namespace intrinsic::kinematics

#endif  // INTRINSIC_KINEMATICS_IK_CONSTRAINED_NLOPT_CONSTRAINED_IK_SOLVER_H_
