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

#include "intrinsic/kinematics/ik/calibrated_ur_inverse_kinematics.h"

#include <array>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_newton_raphson_ik_solver.h"
#include "intrinsic/kinematics/ik_solvers/ur_ik_solver.h"
#include "intrinsic/kinematics/joint_wrapping.h"
#include "intrinsic/kinematics/proto/kinematics_conversion.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/path_resolver/path_resolver.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/helpers.h"
#include "ortools/base/options.h"

namespace intrinsic {
namespace kinematics {

absl::StatusOr<std::unique_ptr<CalibratedUrInverseKinematics>>
CalibratedUrInverseKinematics::Create(
    Chain chain, const UrRobotType& arm_type,
    const InverseKinematicsInterface::Options& options) {
  auto ik_solver =
      absl::WrapUnique(new CalibratedUrInverseKinematics(std::move(chain)));
  INTR_RETURN_IF_ERROR(ik_solver->Init(arm_type));
  return std::move(ik_solver);
}

absl::Status CalibratedUrInverseKinematics::Init(const UrRobotType& arm_type) {
  // (b/264577959) Get the default chain for constructing a UR analytical IK
  // solver from an SDF. This causes a circular dependency on the world when
  // adding the registry for this solver. Instead we load the skeleton from a
  // proto file to avoid the circular dependency.
  if (!arm_type_to_skeleton_proto_.contains(arm_type)) {
    return absl::InvalidArgumentError(
        "Arm type is not supported by the calibrated UR IK solver.");
  }
  const auto& skeleton_proto_file_path =
      arm_type_to_skeleton_proto_.at(arm_type);
  const auto skeleton_proto_file =
      PathResolver::ResolveRunfilesPath(skeleton_proto_file_path);
  INTR_ASSIGN_OR_RETURN(const auto skeleton_proto,
                        file::GetTextProto<intrinsic_proto::Skeleton>(
                            skeleton_proto_file, file::Defaults()));
  INTR_ASSIGN_OR_RETURN(nominal_skeleton_, FromProto(skeleton_proto));

  INTR_ASSIGN_OR_RETURN(nominal_chain_,
                        ExtractNonBranchingChain(*nominal_skeleton_));
  if (nominal_chain_.GetNumberDegreesOfFreedom() !=
      this->chain().GetNumberDegreesOfFreedom()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Cannot build calibrated UR solver. Nominal robot model has ",
        nominal_chain_.GetNumberDegreesOfFreedom(),
        " degrees of freedom, but the solver got a model with ",
        chain().GetNumberDegreesOfFreedom()));
  }

  // Construct a UR analytical IK solver with the default (nominal) chain.
  ur_ik_solver_ = std::make_unique<URIKSolver>();
  INTR_RETURN_IF_ERROR(ur_ik_solver_->Init(&nominal_chain_));

  // Construct an iterative IK solver required for the kinematic calibration
  // refinement.
  chain_ik_solver_ = std::make_unique<
      intrinsic::kinematics::KinematicChainNewtonRaphsonIKSolver>(&chain());

  return absl::OkStatus();
}

absl::StatusOr<InverseKinematicsInterface::IKResult>
CalibratedUrInverseKinematics::ComputeIK(
    const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
    const JointLimits& limits, absl::Span<JointStateP> solutions) const {
  const int num_requested_solutions = solutions.size();
  if (num_requested_solutions < 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The solutions span needs to have size 1 or bigger, but it is: ",
        num_requested_solutions));
  }

  if (hint_joint_state.size() != GetNumDof()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The provided hint vector is of size ", hint_joint_state.size(),
        " but the chain has num DOFs: ", chain().GetNumberDegreesOfFreedom()));
  }

  std::array<intrinsic::eigenmath::VectorNd, URIKSolver::kSolutionBufferSize>
      candidate_solutions;

  const auto candidate_solution_count = ur_ik_solver_->Solve(
      desired_base_t_tip, hint_joint_state.position, &candidate_solutions);

  // If no IK solution can be found, return early. This is likely due to an
  // unreachable pose.
  if (candidate_solution_count == 0) {
    return InverseKinematicsInterface::IKResult(
        {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
         .number_of_solutions = 0});
  }

  auto candidate_solutions_span =
      absl::MakeSpan(candidate_solutions).subspan(0, candidate_solution_count);

  INTRINSIC_RT_RETURN_IF_ERROR(
      WrapAndSort(chain(), chain().GetDofSystemLimits(),
                  hint_joint_state.position, &candidate_solutions_span));

  intrinsic::kinematics::IKValidator limits_validator =
      [&limits](const intrinsic::eigenmath::VectorXd& q_test) {
        // The solution must be within the joint limits.
        JointStateP joint_state;
        joint_state.position = q_test;
        INTRINSIC_RT_ASSIGN_OR_DIE(auto is_within_limits,
                                   IsWithinLimits(joint_state, limits));
        return is_within_limits;
      };

  DVLOG(1) << "Number of analytical IK solutions: " << candidate_solution_count;

  // Perform kinematic calibration refinement and add valid solutions.
  int valid_solution_count = 0;
  for (int i = 0; valid_solution_count < num_requested_solutions &&
                  i < candidate_solution_count;
       i++) {
    DVLOG(1) << "Candidate IK solution: "
             << toString(candidate_solutions_span[i]);

    // Check that the analytical IK solution is within limits.
    if (!limits_validator(candidate_solutions_span[i])) {
      continue;
    }

    // Use the solution from the analytical solver as the seed of the iterative
    // solver, used for the refinement step.
    chain_ik_solver_->setSeed(candidate_solutions_span[i]);
    std::vector<intrinsic::eigenmath::VectorXd> calibrated_solutions =
        chain_ik_solver_->compute(desired_base_t_tip, limits_validator);

    if (calibrated_solutions.empty()) {
      continue;
    }

    DVLOG(1) << "Number of Calibrated IK solution: "
             << calibrated_solutions.size();
    DVLOG(1) << "Calibrated IK solution: " << toString(calibrated_solutions[0]);

    solutions[valid_solution_count].position = calibrated_solutions[0];
    valid_solution_count++;
  }

  return InverseKinematicsInterface::IKResult(
      {.status = valid_solution_count > 0
                     ? InverseKinematicsInterface::IKResult::OK
                     : InverseKinematicsInterface::IKResult::NO_SOLUTION,
       .number_of_solutions = valid_solution_count});
}

absl::StatusOr<InverseKinematicsInterface::IKResult>
CalibratedUrInverseKinematics::ComputeIK(const JointStateP& hint_joint_state,
                                         const Pose3d& desired_base_t_tip,
                                         const JointLimits& limits,
                                         bool ensure_same_branch,
                                         JointStateP* solution) const {
  // If same branch IK is not requested, return the default IK solution.
  if (!ensure_same_branch)
    return ComputeIK(hint_joint_state, desired_base_t_tip, limits, solution);

  // Sanity checks for user-provided input and solver.
  if (solution == nullptr) {
    return absl::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Provided with a null pointer to `solution`. Please provide a valid "
        "pointer to a JointStateP object."));
  }
  if (hint_joint_state.size() != GetNumDof()) {
    return absl::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The provided hint vector is of size ", hint_joint_state.size(),
        " but the chain has num DOFs: ", chain().GetNumberDegreesOfFreedom()));
  }

  // Calculate all IK solutions
  std::array<intrinsic::eigenmath::VectorNd, URIKSolver::kSolutionBufferSize>
      candidate_solutions;

  const auto candidate_solution_count = ur_ik_solver_->Solve(
      desired_base_t_tip, hint_joint_state.position, &candidate_solutions);

  // If no IK solution can be found, return early. This is likely due to an
  // unreachable pose.
  if (candidate_solution_count == 0) {
    return InverseKinematicsInterface::IKResult(
        {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
         .number_of_solutions = 0});
  }

  intrinsic::kinematics::IKValidator limits_validator =
      [&limits](const intrinsic::eigenmath::VectorXd& q_test) {
        // The solution must be within the joint limits.
        JointStateP joint_state;
        joint_state.position = q_test;
        INTRINSIC_RT_ASSIGN_OR_DIE(auto is_within_limits,
                                   IsWithinLimits(joint_state, limits));
        return is_within_limits == true;
      };

  // Identify the branch of the hint joint state.
  const size_t hint_joint_state_branch =
      ur_ik_solver_->GetBranch(hint_joint_state.position);

  // Return the first IK solution with the same branch as the hint joint
  // state.
  for (int i = 0; i < candidate_solution_count; i++) {
    const size_t branch = ur_ik_solver_->GetBranch(candidate_solutions[i]);
    if (branch != hint_joint_state_branch) {
      continue;
    }
    // Wrap the solution close to the hint joint state before returning.
    INTRINSIC_RT_RETURN_IF_ERROR(Wrap(chain(), limits,
                                      hint_joint_state.position,
                                      &candidate_solutions[i])
                                     .status());

    // Ensure that the analytical IK solution is within the joint limits.
    if (!limits_validator(candidate_solutions[i])) {
      return InverseKinematicsInterface::IKResult(
          {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
           .number_of_solutions = 0});
    }

    // Use the solution from the analytical solver on the desired branch as
    // the seed of the iterative solver, used for the refinement step.
    chain_ik_solver_->setSeed(candidate_solutions[i]);
    std::vector<intrinsic::eigenmath::VectorXd> calibrated_solutions =
        chain_ik_solver_->compute(desired_base_t_tip, limits_validator);

    // Return early if kinematic calibration refinement failed.
    if (calibrated_solutions.empty()) {
      return InverseKinematicsInterface::IKResult(
          {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
           .number_of_solutions = 0});
    }

    *solution = calibrated_solutions[0];
    return InverseKinematicsInterface::IKResult(
        {.status = InverseKinematicsInterface::IKResult::OK,
         .number_of_solutions = 1});
  }

  // Return result with no solution since none of the IK solutions had the
  // same branch as the hint joint state.
  return InverseKinematicsInterface::IKResult(
      {.status = InverseKinematicsInterface::IKResult::NO_SOLUTION,
       .number_of_solutions = 0});
}

absl::StatusOr<size_t> CalibratedUrInverseKinematics::ComputeBranch(
    const JointStateP& joint_state) const {
  return ur_ik_solver_->GetBranch(joint_state.position);
}

}  // namespace kinematics
}  // namespace intrinsic
