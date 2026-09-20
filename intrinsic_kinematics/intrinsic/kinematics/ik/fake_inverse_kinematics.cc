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

#include "intrinsic/kinematics/ik/fake_inverse_kinematics.h"

#include <array>

#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

icon::RealtimeStatus FakeInverseKinematics::SetIKResult(
    absl::Span<const JointStateP> fake_ik_joint_solutions,
    const IKResult& fake_ik_result) {
  if (fake_ik_joint_solutions.size() > kMaxNumFakeSolutions) {
    return icon::InvalidArgumentError(
        "The size of fake_ik_joint_solutions should be less than or equal "
        "to kMaxNumFakeSolutions.");
  }
  if (fake_ik_result.number_of_solutions > 0 &&
      fake_ik_joint_solutions.size() != fake_ik_result.number_of_solutions) {
    return icon::InvalidArgumentError(
        "The size of fake_ik_joint_solutions should be equal to the number "
        "of solutions in fake_ik_result.");
  }
  fake_ik_result_ = fake_ik_result;
  for (int i = 0; i < fake_ik_joint_solutions.size(); ++i) {
    fake_ik_joint_solutions_[i] = fake_ik_joint_solutions[i];
  }
  return icon::OkStatus();
}

icon::RealtimeStatusOr<InverseKinematicsInterface::IKResult>
FakeInverseKinematics::RealtimeComputeIK(
    const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
    const JointLimits& limits, absl::Span<JointStateP> solutions) const {
  if (!fake_ik_result_.has_value()) {
    return icon::FailedPreconditionError(
        "Fake IK result not set. Did you call SetIKResult()?");
  }
  if (solutions.empty()) {
    return icon::InvalidArgumentError(
        "The size of solutions should be greater or equal than 1.");
  }

  if (solutions.size() < fake_ik_result_->number_of_solutions) {
    return icon::InternalError(
        "The size of `solutions` should be greater or equal than the number of "
        "solutions in fake_ik_result.");
  }
  for (int i = 0; i < fake_ik_result_->number_of_solutions; ++i) {
    solutions[i] = fake_ik_joint_solutions_[i];
  }

  return *fake_ik_result_;
}

icon::RealtimeStatusOr<FakeInverseKinematics::IKResult>
FakeInverseKinematics::RealtimeComputeIK(const JointStateP& hint_joint_state,
                                         const Pose3d& desired_base_t_tip,
                                         const JointLimits& limits,
                                         bool ensure_same_branch,
                                         JointStateP* solution) const {
  std::array<JointStateP, kMaxNumFakeSolutions> solutions;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      IKResult result, RealtimeComputeIK(hint_joint_state, desired_base_t_tip,
                                         limits, absl::MakeSpan(solutions)));
  *solution = solutions[0];
  result.number_of_solutions = 1;
  return result;
}

}  // namespace kinematics
}  // namespace intrinsic
