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

#ifndef INTRINSIC_KINEMATICS_IK_FAKE_INVERSE_KINEMATICS_H_
#define INTRINSIC_KINEMATICS_IK_FAKE_INVERSE_KINEMATICS_H_

#include <array>
#include <optional>
#include <string_view>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

class FakeInverseKinematics : public InverseKinematicsInterfaceRT {
 public:
  static constexpr int kMaxNumFakeSolutions = 8;
  using InverseKinematicsInterface::ComputeIK;
  using InverseKinematicsInterface::RealtimeComputeIK;

  explicit FakeInverseKinematics(int njoints)
      : InverseKinematicsInterfaceRT(Chain()), njoints_(njoints) {}

  static constexpr char kSolverName[] = "fake_inverse_kinematics";

  const std::string_view GetName() const override { return kSolverName; }

  int GetNumDof() const override { return njoints_; }

  bool ImplementsSameBranchIK() const override { return true; }

  icon::RealtimeStatus SetIKResult(
      absl::Span<const JointStateP> fake_ik_joint_solutions,
      const IKResult& fake_ik_result);

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

  bool IsCompatibleWithChain(const Chain& chain) const override {
    LOG(FATAL) << "Not implemented.";
    return false;
  }

 private:
  int njoints_;
  std::array<JointStateP, kMaxNumFakeSolutions> fake_ik_joint_solutions_;
  std::optional<IKResult> fake_ik_result_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_FAKE_INVERSE_KINEMATICS_H_
