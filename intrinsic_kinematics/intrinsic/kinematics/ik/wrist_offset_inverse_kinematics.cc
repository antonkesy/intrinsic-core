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

#include "intrinsic/kinematics/ik/wrist_offset_inverse_kinematics.h"

#include <algorithm>
#include <array>
#include <memory>
#include <new>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "intrinsic/eigenmath/pose3_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/sanitize_ik.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/units.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

absl::StatusOr<std::unique_ptr<WristOffsetInverseKinematics>>
WristOffsetInverseKinematics::Create(
    Chain chain, const InverseKinematicsInterface::Options& options) {
  auto ik_solver =
      std::make_unique<WristOffsetInverseKinematics>(std::move(chain));
  INTR_RETURN_IF_ERROR(ik_solver->Init());
  return std::move(ik_solver);
}

absl::Status WristOffsetInverseKinematics::Init() {
  ik_solver_ =
      std::make_unique<intrinsic::kinematics::WristOffsetGeometricIKSolver>();
  return ik_solver_->Init(chain());
}

absl::StatusOr<InverseKinematicsInterface::IKResult>
WristOffsetInverseKinematics::ComputeIK(
    const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
    const JointLimits& limits, absl::Span<JointStateP> solutions) const {
  if (!ik_solver_) {
    return absl::InternalError(
        "The solver has not been properly initialized. Did you call Init?");
  }
  const int num_requested_solutions = solutions.size();
  if (num_requested_solutions < 1) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "The solutions span needs to have size 1 or bigger, but it is: %d",
        num_requested_solutions));
  }

  if (hint_joint_state.size() != GetNumDof()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "The provided hint vector is of size %d but the chain has %d DOFs",
        hint_joint_state.size(), chain().GetNumberDegreesOfFreedom()));
  }

  // Invoke the wrist offset solver.
  std::array<
      intrinsic::eigenmath::Vector6dAligned,
      intrinsic::kinematics::WristOffsetGeometricIKSolver::kSolutionBufferSize>
      results;
  int num_results = ik_solver_->Solve(desired_base_t_tip, &results);

  intrinsic::kinematics::State kinematics_state(&chain());

  // Copy the results from std::array<intrinsic::eigenmath::Vector6dAligned>
  // into the solutions span.
  for (int i = 0; i < num_results && i < num_requested_solutions; i++) {
    // We ignore limits for now, since they're handled below.
    INTR_RETURN_IF_ERROR(
        kinematics_state.SetDofPositions(results[i],
                                         /*check_limits=*/false));

    // Log a warning if the solution's cartesian error is > 1 millimeter or >
    // 1 degree.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const intrinsic::Pose3d base_t_tip,
        kinematics_state.GetTransform(chain().GetTipId()));
    intrinsic::eigenmath::PoseError err =
        intrinsic::eigenmath::PoseErrorBetween(base_t_tip, desired_base_t_tip);

    if (err.translation > 0.001) {
      INTRINSIC_RT_LOG(WARNING)
          << "Warning: High IK translation error: " << err.translation;
    }
    if (err.rotation > intrinsic::DegToRad(1.0)) {
      INTRINSIC_RT_LOG(WARNING)
          << "Warning: High IK angular error: " << err.rotation;
    }

    JointStateP& solution = solutions[i];
    INTR_RETURN_IF_ERROR(solution.SetSize(GetNumDof()));
    solution.position = results[i];
  }

  // TODO(keegang,prisament) Remove any duplicate solutions.
  const int num_candidate_solutions =
      std::min(num_results, num_requested_solutions);
  const int num_wrapped_solutions = sanitize_ik::WrapToLimits(
      solutions.subspan(0, num_candidate_solutions), limits);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const int num_final_solutions,
      sanitize_ik::ExpandWindings(solutions, limits, num_wrapped_solutions));

  return InverseKinematicsInterface::IKResult(
      {.status = num_final_solutions > 0
                     ? InverseKinematicsInterface::IKResult::OK
                     : InverseKinematicsInterface::IKResult::NO_SOLUTION,
       .number_of_solutions = num_final_solutions});
}

}  // namespace kinematics
}  // namespace intrinsic
