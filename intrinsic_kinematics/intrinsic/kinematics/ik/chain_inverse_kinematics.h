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

#ifndef INTRINSIC_KINEMATICS_IK_CHAIN_INVERSE_KINEMATICS_H_
#define INTRINSIC_KINEMATICS_IK_CHAIN_INVERSE_KINEMATICS_H_

#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include "absl/flags/declare.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/math/pose3.h"

ABSL_DECLARE_FLAG(double, chain_ik_random_seed_solver_timeout_s);

namespace intrinsic {
namespace kinematics {

// The ChainInverseKinematics class implements the
// ManipulatorKinematicsInterface to allow for arbitrary kinematic systems to
// have an inverse kinematic solver. This is achieved using the kinematic chain
// solvers. As long as the intrinsic::kinematics can be described as a chain we
// can solve inverse kinematics for it. The solvers currently implemented here
// are not intended for real time use.
class ChainInverseKinematics : public InverseKinematicsInterfaceNonRT {
 public:
  using InverseKinematicsInterface::ComputeIK;

  static constexpr char kSolverName[] = "kinematic_chain";

  static absl::StatusOr<std::unique_ptr<ChainInverseKinematics>> Create(
      Chain chain, const InverseKinematicsInterface::Options& options);

  explicit ChainInverseKinematics(Chain chain)
      : InverseKinematicsInterfaceNonRT(std::move(chain)) {}

  absl::Status Init();

  int GetNumDof() const override;

  const std::string_view GetName() const override { return kSolverName; }

  bool ImplementsSameBranchIK() const override { return false; }

  absl::StatusOr<IKResult> ComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      absl::Span<JointStateP> solutions) const override;

  std::optional<ElementId> RetrieveMatchingStartJoint(
      const ModelInterface& model, ElementId end_joint) const override {
    LOG(FATAL) << "Not implemented.";
    return std::nullopt;
  }

  bool IsCompatibleWithChain(const Chain& chain) const override {
    // The chain solver is compatible with any Chain.
    return true;
  }

 private:
  std::unique_ptr<intrinsic::kinematics::KinematicChainIKSolver> ik_solver_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_CHAIN_INVERSE_KINEMATICS_H_
