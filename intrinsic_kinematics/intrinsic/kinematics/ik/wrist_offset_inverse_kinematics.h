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

#ifndef INTRINSIC_KINEMATICS_IK_WRIST_OFFSET_INVERSE_KINEMATICS_H_
#define INTRINSIC_KINEMATICS_IK_WRIST_OFFSET_INVERSE_KINEMATICS_H_

#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/wrist_offset_geometric_ik_solver.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// Inverse kinematics implementation that uses the WristOffsetGeometricIKSolver
// (which is the offset wrist IK solver for automatically generated Assemblies).
// TODO(efernan): While this solver is meant to be real-time safe, it is
// currently not: the underlying WristOffsetGeometricIKSolver needs to be fixed.
class WristOffsetInverseKinematics : public InverseKinematicsInterfaceNonRT {
 public:
  using InverseKinematicsInterface::ComputeIK;

  static constexpr char kSolverName[] = "wrist_offset";

  explicit WristOffsetInverseKinematics(Chain chain)
      : InverseKinematicsInterfaceNonRT(std::move(chain)) {}

  static absl::StatusOr<std::unique_ptr<WristOffsetInverseKinematics>> Create(
      Chain chain, const InverseKinematicsInterface::Options& options);

  // Prepares the underlying solver. Needs to be called before attempting to
  // compute IK.
  absl::Status Init();

  int GetNumDof() const override { return 6; }

  bool ImplementsSameBranchIK() const override { return false; }

  const std::string_view GetName() const override { return kSolverName; }

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
    LOG(FATAL) << "Not implemented.";
    return false;
  }

 private:
  std::unique_ptr<WristOffsetGeometricIKSolver> ik_solver_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_WRIST_OFFSET_INVERSE_KINEMATICS_H_
