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

#ifndef INTRINSIC_KINEMATICS_IK_UR_INVERSE_KINEMATICS_H_
#define INTRINSIC_KINEMATICS_IK_UR_INVERSE_KINEMATICS_H_

#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/ur_ik_solver.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// This class provides inverse kinematics for Universal Robotics' robots.
//
// There is partial validation of the passed model, but care should be taken to
// confirm that the model conforms to the required structure.
class UrInverseKinematics : public InverseKinematicsInterfaceRT {
 public:
  using InverseKinematicsInterface::ComputeIK;
  using InverseKinematicsInterface::RealtimeComputeIK;

  static constexpr char kSolverName[] = "ur";

  static absl::StatusOr<std::unique_ptr<UrInverseKinematics>> Create(
      Chain chain, const InverseKinematicsInterface::Options& options);

  explicit UrInverseKinematics(Chain chain)
      : InverseKinematicsInterfaceRT(std::move(chain)) {}

  // Prepares the underlying solver. Needs to be called before attempting to
  // compute IK.
  absl::Status Init();

  int GetNumDof() const override { return 6; }

  const std::string_view GetName() const override { return kSolverName; }

  bool ImplementsSameBranchIK() const override { return true; }

  absl::StatusOr<size_t> ComputeBranch(
      const JointStateP& joint_state) const override;

  icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      absl::Span<JointStateP> solutions) const override;

  icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits, bool ensure_same_branch,
      JointStateP* solution) const override;

  std::optional<ElementId> RetrieveMatchingStartJoint(
      const ModelInterface& model, ElementId end_joint) const override {
    LOG(FATAL) << "Not implemented.";
    return std::nullopt;
  }

  bool IsCompatibleWithChain(const Chain& chain) const override;

 private:
  std::unique_ptr<intrinsic::kinematics::URIKSolver> ik_solver_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_UR_INVERSE_KINEMATICS_H_
