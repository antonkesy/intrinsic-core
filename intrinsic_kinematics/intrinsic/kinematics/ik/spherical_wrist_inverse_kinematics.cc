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

#include "intrinsic/kinematics/ik/spherical_wrist_inverse_kinematics.h"

#include <cstddef>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/spherical_wrist_ik_solver.h"
#include "intrinsic/kinematics/joint_wrapping.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

/*static*/
absl::StatusOr<std::unique_ptr<SphericalWristInverseKinematics>>
SphericalWristInverseKinematics::Create(
    Chain chain, const InverseKinematicsInterface::Options& options) {
  auto ik_solver =
      std::make_unique<SphericalWristInverseKinematics>(std::move(chain));

  INTR_RETURN_IF_ERROR(ik_solver->Init());
  return std::move(ik_solver);
}

absl::Status SphericalWristInverseKinematics::Init() {
  if (chain().GetNumberDegreesOfFreedom() != GetNumDof()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("The chain needs to have %d dofs, but has %d.",
                        GetNumDof(), chain().GetNumberDegreesOfFreedom()));
  }

  ik_solver_ = std::make_unique<SphericalWristIKSolver>(&chain());

  return ik_solver_->Init();
}

icon::RealtimeStatusOr<InverseKinematicsInterface::IKResult>
SphericalWristInverseKinematics::RealtimeComputeIK(
    const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
    const JointLimits& limits, absl::Span<JointStateP> solutions) const {
  if (!ik_solver_) {
    return icon::InternalError(
        "The solver has not been properly initialized. Did you call Init?");
  }
  const int num_requested_solutions = solutions.size();
  if (num_requested_solutions < 1) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The solutions span needs to have size 1 or bigger, but it is: ",
        num_requested_solutions));
  }

  if (hint_joint_state.size() != GetNumDof()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The provided hint vector is of size ", hint_joint_state.size(),
        " but the chain has num DOFs: ", chain().GetNumberDegreesOfFreedom()));
  }

  // Calculate all IK solutions
  SphericalWristIKSolutionSet candidate_solution_set;
  ik_solver_->SetNearestJointStateHint(hint_joint_state.position);
  int candidate_solution_count =
      ik_solver_->Solve(desired_base_t_tip, &candidate_solution_set);
  if (candidate_solution_count == 0) {
    return InverseKinematicsInterface::IKResult(
        {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
         .number_of_solutions = 0});
  }

  auto candidate_solution_span = absl::MakeSpan(candidate_solution_set)
                                     .subspan(0, candidate_solution_count);

  // Sort solutions according to distance to hint_joint_state and kinematic
  // branch.
  INTRINSIC_RT_RETURN_IF_ERROR(ik_solver_->WrapAndSort(
      chain(), limits, hint_joint_state.position, candidate_solution_span));

  int valid_solution_count = 0;
  for (int i = 0; valid_solution_count < num_requested_solutions &&
                  i < candidate_solution_count;
       i++) {
    const JointStateP& candidate_solution = candidate_solution_span[i];

    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto limit_check,
                                  IsWithinLimits(candidate_solution, limits));
    if (!limit_check.p_ok) {
      continue;
    }
    solutions[valid_solution_count] = candidate_solution;
    valid_solution_count++;
  }

  return InverseKinematicsInterface::IKResult(
      {.status = valid_solution_count > 0
                     ? InverseKinematicsInterface::IKResult::OK
                     : InverseKinematicsInterface::IKResult::NO_SOLUTION,
       .number_of_solutions = valid_solution_count});
}

icon::RealtimeStatusOr<InverseKinematicsInterface::IKResult>
SphericalWristInverseKinematics::RealtimeComputeIK(
    const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
    const JointLimits& limits, bool ensure_same_branch,
    JointStateP* solution) const {
  // If same branch IK is not enabled, return the default IK solution.
  if (!ensure_same_branch) {
    return RealtimeComputeIK(hint_joint_state, desired_base_t_tip, limits,
                             solution);
  }

  // Sanity checks for user-provided input and solver.
  if (solution == nullptr) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided with a null pointer to `solution`. Please provide a valid "
        "pointer to a JointStateP object."));
  }
  if (!ik_solver_) {
    return icon::InternalError(
        "The solver has not been properly initialized. Did you call Init?");
  }
  if (hint_joint_state.size() != GetNumDof()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The provided hint vector is of size ", hint_joint_state.size(),
        " but the chain has num DOFs: ", chain().GetNumberDegreesOfFreedom()));
  }

  // Calculate all IK solutions
  SphericalWristIKSolutionSet candidate_solution_set;
  ik_solver_->SetNearestJointStateHint(hint_joint_state.position);
  const int candidate_solution_count =
      ik_solver_->Solve(desired_base_t_tip, &candidate_solution_set);
  if (candidate_solution_count == 0) {
    return InverseKinematicsInterface::IKResult(
        {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
         .number_of_solutions = 0});
  }

  // Get the branch of the hint joint state.
  const size_t hint_joint_state_branch =
      ik_solver_->GetBranch(hint_joint_state.position);

  // Return the first IK solution with the same branch as the hint joint state
  // since sometimes the underlying spherical wrist IK solver returns multiple
  // of the same IK solution. This also avoids any memory allocation.
  // [http://b/250526143].
  for (int i = 0; i < candidate_solution_count; i++) {
    const size_t branch = ik_solver_->GetBranch(candidate_solution_set[i]);
    if (branch == hint_joint_state_branch) {
      // Wrap the solution close to the hint joint state before returning.
      INTRINSIC_RT_RETURN_IF_ERROR(Wrap(chain(), limits,
                                        hint_joint_state.position,
                                        &candidate_solution_set[i])
                                       .status());
      *solution = candidate_solution_set[i];
      return InverseKinematicsInterface::IKResult(
          {.status = InverseKinematicsInterface::IKResult::OK,
           .number_of_solutions = 1});
    }
  }

  // Return result with no solution since none of the IK solutions had the same
  // branch as the hint joint state.
  return InverseKinematicsInterface::IKResult(
      {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
       .number_of_solutions = 0});
}

absl::StatusOr<size_t> SphericalWristInverseKinematics::ComputeBranch(
    const JointStateP& joint_state) const {
  return ik_solver_->GetBranch(joint_state.position);
}

absl::StatusOr<double> SphericalWristInverseKinematics::GetMaximumArmLength()
    const {
  return ik_solver_->GetMaximumArmLength();
};

}  // namespace kinematics
}  // namespace intrinsic
