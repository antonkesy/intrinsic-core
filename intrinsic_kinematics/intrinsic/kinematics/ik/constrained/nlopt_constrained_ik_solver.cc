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

#include "intrinsic/kinematics/ik/constrained/nlopt_constrained_ik_solver.h"

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_interface.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_proto_conversions.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_util.h"
#include "intrinsic/kinematics/ik/constrained/constraints.h"
#include "intrinsic/kinematics/ik/constrained/cost_functions.h"
#include "intrinsic/kinematics/ik/constrained/nlopt/nlopt_constrained_ik.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/numopt/costfunction_interface.h"
#include "intrinsic/math/numopt/nlopt_nonlinear_program.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kinematics {

absl::Status NLOptConstrainedIKSolver::SetProblem(
    std::vector<std::unique_ptr<ConstraintInterface>>&& constraints,
    std::vector<std::unique_ptr<CostFunctionInterface>>&& costs) {
  INTRINSIC_ASSERT_NON_REALTIME();

  auto nlopt_problem =
      std::make_unique<NLOptProblem>(chain().GetNumberDegreesOfFreedom());

  // Add all costs to the NLOptProblem.
  for (auto& cost : costs) {
    nlopt_problem->AddCost(std::move(cost));
  }

  // Add all constraints to the NLOptProblem.
  for (auto& constraint : constraints) {
    nlopt_problem->AddConstraint(std::move(constraint));
  }

  INTR_ASSIGN_OR_RETURN(
      nlopt_constrained_ik_,
      NLOptConstrainedIK::Create(std::move(nlopt_problem), &chain()));

  return absl::OkStatus();
}

absl::Status NLOptConstrainedIKSolver::SetProblem(
    const intrinsic_proto::kinematics::ConstrainedGoal& constrained_goal) {
  INTRINSIC_ASSERT_NON_REALTIME();

  auto nlopt_problem =
      std::make_unique<NLOptProblem>(chain().GetNumberDegreesOfFreedom());

  // Loop through cost terms and add them to the NLOptProblem.
  for (int i = 0; i < constrained_goal.costs_size(); ++i) {
    switch (constrained_goal.costs(i).cost_case()) {
      case intrinsic_proto::kinematics::Cost::kJointPositionCost: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<JointPositionCost> joint_position_cost,
            FromProto(constrained_goal.costs(i).joint_position_cost(),
                      &chain()));
        nlopt_problem->AddCost(std::move(joint_position_cost));
        break;
      }
      case intrinsic_proto::kinematics::Cost::kManipulabilityCost: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<ManipulabilityCost> manipulability_cost,
            FromProto(constrained_goal.costs(i).manipulability_cost(),
                      &chain()));
        nlopt_problem->AddCost(std::move(manipulability_cost));
        break;
      }
      case intrinsic_proto::kinematics::Cost::kJointVelocityCost: {
        return absl::UnimplementedError("Un-implemented cost type.");
      }
      case intrinsic_proto::kinematics::Cost::
          kManipulabilityTimesMinJointLimitDistanceCost: {
        return absl::UnimplementedError("Un-implemented cost type.");
      }
      case intrinsic_proto::kinematics::Cost::COST_NOT_SET: {
        return absl::FailedPreconditionError("Unknown cost type.");
      }
    }
  }

  // Loop through constraint terms and add them to the NLOptProblem.
  for (int i = 0; i < constrained_goal.constraints_size(); ++i) {
    switch (constrained_goal.constraints(i).constraint_case()) {
      case intrinsic_proto::kinematics::Constraint::
          kJointPositionLimitsConstraint: {
        JointStateP min_position;
        JointStateP max_position;
        INTR_RETURN_IF_ERROR(FromProto(
            constrained_goal.constraints(i).joint_position_limits_constraint(),
            min_position, max_position));
        INTR_RETURN_IF_ERROR(
            nlopt_problem->SetBoxConstraintLowerBound(min_position.position));
        INTR_RETURN_IF_ERROR(
            nlopt_problem->SetBoxConstraintUpperBound(max_position.position));
        break;
      }
      case intrinsic_proto::kinematics::Constraint::kPointConstraint: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<PointConstraint> point_constraint,
            FromProto(constrained_goal.constraints(i).point_constraint(),
                      &chain()));
        nlopt_problem->AddConstraint(std::move(point_constraint));
        break;
      }
      case intrinsic_proto::kinematics::Constraint::kPoseConstraint: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<PoseConstraint> pose_constraint,
            FromProto(constrained_goal.constraints(i).pose_constraint(),
                      &chain()));
        nlopt_problem->AddConstraint(std::move(pose_constraint));
        break;
      }
      case intrinsic_proto::kinematics::Constraint::
          kOrientationConeConstraint: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<OrientationConeConstraint> cone_constraint,
            FromProto(
                constrained_goal.constraints(i).orientation_cone_constraint(),
                &chain()));
        nlopt_problem->AddConstraint(std::move(cone_constraint));
        break;
      }
      case intrinsic_proto::kinematics::Constraint::
          kOrientationFreeAxisConstraint: {
        INTR_ASSIGN_OR_RETURN(std::unique_ptr<OrientationWithFreeAxisConstraint>
                                  free_axis_constraint,
                              FromProto(constrained_goal.constraints(i)
                                            .orientation_free_axis_constraint(),
                                        &chain()));
        nlopt_problem->AddConstraint(std::move(free_axis_constraint));
        break;
      }
      case intrinsic_proto::kinematics::Constraint::
          kPositionEllipsoidConstraint: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<EllipsoidConstraint> ellipsoid_constraint,
            FromProto(
                constrained_goal.constraints(i).position_ellipsoid_constraint(),
                &chain()));
        nlopt_problem->AddConstraint(std::move(ellipsoid_constraint));
        break;
      }
      case intrinsic_proto::kinematics::Constraint::kPlaneConstraint: {
        INTR_ASSIGN_OR_RETURN(
            std::unique_ptr<PlaneConstraint> plane_constraint,
            FromProto(constrained_goal.constraints(i).plane_constraint(),
                      &chain()));
        nlopt_problem->AddConstraint(std::move(plane_constraint));
        break;
      }
      case intrinsic_proto::kinematics::Constraint::
          kJointVelocityLimitsConstraint: {
        return absl::UnimplementedError("Un-implemented constraint type.");
      }
      case intrinsic_proto::kinematics::Constraint::
          kJointAccelerationLimitsConstraint: {
        return absl::UnimplementedError("Un-implemented constraint type.");
      }
      case intrinsic_proto::kinematics::Constraint::CONSTRAINT_NOT_SET: {
        return absl::FailedPreconditionError("Unknown constraint type.");
      }
    }
  }

  INTR_ASSIGN_OR_RETURN(
      nlopt_constrained_ik_,
      NLOptConstrainedIK::Create(std::move(nlopt_problem), &chain()));

  return absl::OkStatus();
}

absl::StatusOr<ConstrainedIKInterface::Result> NLOptConstrainedIKSolver::Solve(
    const ConstrainedIKInterface::Options& options) const {
  if (!nlopt_constrained_ik_) {
    return absl::FailedPreconditionError(
        "You must call SetProblem() before calling Solve()");
  }

  INTRINSIC_ASSERT_NON_REALTIME();

  // Check if user provided initial guess.
  std::optional<eigenmath::VectorNd> joint_position_initial_guess =
      std::nullopt;
  if (options.joint_position_initial_guess.has_value()) {
    joint_position_initial_guess =
        options.joint_position_initial_guess->position;
  }

  // TODO(giftthaler): populate NLOptoptions in Solve() call once required.
  INTR_ASSIGN_OR_RETURN(
      NLOptConstrainedIK::Result nlopt_result,
      nlopt_constrained_ik_->Solve(ConstrainedIKOptions{
          .joint_position_initial_guess = joint_position_initial_guess,
          .max_solver_attempts = options.max_number_of_attempts,
          .halton_sequence_index = options.halton_sequence_index,
      }));

  // Transcribe results from NLOpt into the interface result container.
  ConstrainedIKInterface::Result result{
      .status = ConstrainedIKInterface::Result::NO_SOLUTION,
      .number_of_attempts = nlopt_result.number_of_attempts,
      .elapsed_time = nlopt_result.measured_processing_time,
  };
  if (nlopt_result.status == NLOptSolver::Result::SOLVED) {
    result.status = ConstrainedIKInterface::Result::OK;
    if (nlopt_result.cost == std::nullopt) {
      return absl::InternalError("Optional cost should have a value.");
    }
    result.cost = nlopt_result.cost;
    if (nlopt_result.q_solution == std::nullopt) {
      return absl::InternalError("Optional q_solution should have a value.");
    }
    result.q_star = JointStateP(*nlopt_result.q_solution);
  }
  return result;
}

auto NLOptConstrainedIKSolver::ComputeIK(
    const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
    const JointLimits& limits, absl::Span<JointStateP> solutions) const
    -> absl::StatusOr<IKResult> {
  const int num_requested_solutions = solutions.size();
  if (num_requested_solutions < 1) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "The solutions span needs to have size 1 or bigger, but it is: %d",
        num_requested_solutions));
  }

  if (hint_joint_state.size() != chain().GetNumberDegreesOfFreedom()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "The provided hint vector is of size %d but the chain has %d DOFs",
        hint_joint_state.size(), chain().GetNumberDegreesOfFreedom()));
  }

  if (limits.size() != chain().GetNumberDegreesOfFreedom()) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "The provided 'limits' is of size %d but the chain has %d DOFs",
        limits.size(), chain().GetNumberDegreesOfFreedom()));
  }

  INTRINSIC_ASSERT_NON_REALTIME();

  // We define a new NLOptProblem
  auto nlopt_problem =
      std::make_unique<NLOptProblem>(chain().GetNumberDegreesOfFreedom());

  // ... which includes the provided joint limits.
  INTR_RETURN_IF_ERROR(
      nlopt_problem->SetBoxConstraintLowerBound(limits.min_position));
  INTR_RETURN_IF_ERROR(
      nlopt_problem->SetBoxConstraintUpperBound(limits.max_position));

  // ... and includes a full pose constraint for 'desired_base_t_tip'.
  INTR_ASSIGN_OR_RETURN(
      auto pose_constraint,
      PoseConstraint::Create(&chain(), desired_base_t_tip, chain().GetTipId()));
  nlopt_problem->AddConstraint(std::move(pose_constraint));

  // We add a joint position cost with default weights around the
  // 'hint_joint_state' robot joint configuration.
  INTR_ASSIGN_OR_RETURN(
      auto joint_position_cost,
      JointPositionCost::Create(&chain(), hint_joint_state.position));
  nlopt_problem->AddCost(std::move(joint_position_cost));

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<NLOptConstrainedIK> nlopt_solver,
      NLOptConstrainedIK::Create(std::move(nlopt_problem), &chain()));

  IKResult result{
      .status = IKResult::Status::NO_SOLUTION,
      .number_of_solutions = 0,
  };

  // The maximum number of attempts we take to find different
  // 'num_requested_solutions' (hard-coded).
  const int max_attempts = num_requested_solutions * 25;
  // When to terminate early after not having found a new (different) solution.
  const int max_trials_without_finding_new_solution = 25;
  int attempts_since_last_success = 0;
  int halton_sequence_index = 0;
  for (int i = 0; i < max_attempts; ++i) {
    // Only for the first run we use the provided hint joint state, otherwise we
    // sample randomly.
    eigenmath::VectorNd q_init;
    if (i == 0) {
      q_init = hint_joint_state.position;
    } else {
      INTR_ASSIGN_OR_RETURN(
          q_init, GetPseudoRandomConfiguration(chain(), halton_sequence_index));
    }

    // Attempt to solve IK problem.
    INTR_ASSIGN_OR_RETURN(auto nlopt_result,
                          nlopt_solver->Solve(ConstrainedIKOptions{
                              .joint_position_initial_guess = q_init,
                              .max_solver_attempts = 1,
                              .halton_sequence_index = halton_sequence_index,
                          }));
    attempts_since_last_success++;

    if (i > 0) {
      // For the first solver invocation we use the user-provided hint joint
      // state, so for that case the random number index does not need to be
      // incremented. Otherwise, we increment the random number seed by one.
      halton_sequence_index += 1;
    }

    if (nlopt_result.status == NLOptSolver::Result::SOLVED) {
      result.status = IKResult::Status::OK;
      JointStateP new_solution_candidate(*nlopt_result.q_solution);
      // Compare to previously recorded solutions and include new solution
      // only if it has not been found before. Note that hard-coded threshold.
      bool solution_is_new = true;
      for (int sol_idx = 0; sol_idx < result.number_of_solutions; ++sol_idx) {
        if (new_solution_candidate.position.isApprox(
                solutions[sol_idx].position, 1e-3)) {
          solution_is_new = false;
          break;
        }
      }
      if (solution_is_new) {
        solutions[result.number_of_solutions] = new_solution_candidate;
        result.number_of_solutions++;
        attempts_since_last_success = 0;
      }
    }

    // Terminate loop if we have found as many different solutions as
    // requested.
    if (result.number_of_solutions == num_requested_solutions) {
      break;
    }

    // Terminate early if we have not found a new solution during a given
    // number of runs.
    if (attempts_since_last_success >=
        max_trials_without_finding_new_solution) {
      break;
    }
  }

  return result;
}

}  // namespace intrinsic::kinematics
