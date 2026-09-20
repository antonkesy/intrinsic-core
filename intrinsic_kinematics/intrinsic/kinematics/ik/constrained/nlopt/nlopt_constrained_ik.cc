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

#include "intrinsic/kinematics/ik/constrained/nlopt/nlopt_constrained_ik.h"

#include <iomanip>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>

#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_util.h"
#include "intrinsic/math/numopt/nlopt_nonlinear_program.h"
#include "intrinsic/util/status/status_macros.h"
#include "magic_enum/magic_enum.hpp"
#include "nlopt.hpp"

namespace intrinsic::kinematics {

/*static*/
absl::StatusOr<std::unique_ptr<NLOptConstrainedIK>> NLOptConstrainedIK::Create(
    std::unique_ptr<NLOptProblem> problem, const Chain* chain,
    nlopt_algorithm algorithm) {
  if (!problem) {
    return absl::FailedPreconditionError("NLOptProblem pointer is nullptr.");
  }

  if (!chain) {
    return absl::FailedPreconditionError("Chain is nullptr.");
  }

  INTR_ASSIGN_OR_RETURN(auto nlopt_solver,
                        NLOptSolver::Create(std::move(problem), algorithm));

  // Using 'new' to access private constructor.
  return absl::WrapUnique(
      new NLOptConstrainedIK(std::move(nlopt_solver), chain));
}

absl::StatusOr<NLOptConstrainedIK::Result> NLOptConstrainedIK::Solve(
    const ConstrainedIKOptions& options, const NLOptOptions& nlopt_options) {
  if (options.max_solver_attempts < 1) {
    return absl::FailedPreconditionError(
        "max_solver_attempts option must be >= 1.");
  }

  // Used for random sampling of initial guesses. The Halton sequence is used
  // for deterministic random sampling.
  int halton_sequence_index = options.halton_sequence_index;

  eigenmath::VectorNd q_init_guess;
  if (options.joint_position_initial_guess.has_value()) {
    q_init_guess = options.joint_position_initial_guess.value();
  } else {
    INTR_ASSIGN_OR_RETURN(q_init_guess, GetPseudoRandomConfiguration(
                                            *chain_, halton_sequence_index));
  }

  Result result{.status = NLOptSolver::Result::NOT_SOLVED};

  while (result.status == NLOptSolver::Result::NOT_SOLVED &&
         result.number_of_attempts < options.max_solver_attempts) {
    // Use a random joint position as initial guess in all but the first
    // attempt.
    if (result.number_of_attempts > 0) {
      INTR_ASSIGN_OR_RETURN(q_init_guess, GetPseudoRandomConfiguration(
                                              *chain_, halton_sequence_index));
    }

    VLOG(2) << "ConstrainedIK - run optimization attempt "
            << result.number_of_attempts;

    INTR_ASSIGN_OR_RETURN(NLOptSolver::Result nlopt_result,
                          solver_->Solve(q_init_guess, nlopt_options));

    result.measured_processing_time += nlopt_result.measured_processing_time;
    result.number_of_attempts++;

    VLOG(2) << "Return code is:\t"
            << magic_enum::enum_name(nlopt_result.status);
    if (nlopt_result.status != NLOptSolver::Result::NOT_SOLVED) {
      VLOG(2) << "Min objective:\t" << *nlopt_result.cost;
      VLOG(2) << "Joint angles: " << std::endl;
      VLOG(2) << std::setprecision(3) << nlopt_result.x_solution->transpose();
    } else {
      VLOG(2) << "Solver did not find solution";
    }
    VLOG(2) << "Constraint residual norms: ";
    for (const auto& residuals : nlopt_result.named_constraint_residuals) {
      VLOG(2) << "'" << residuals.first << "':\t" << residuals.second.norm();
    }

    // In case of success, extract solution and exit loop.
    if (nlopt_result.status == NLOptSolver::Result::SOLVED) {
      result.status = NLOptSolver::Result::SOLVED;
      result.q_solution = nlopt_result.x_solution;
      result.cost = *nlopt_result.cost;
      break;
    }
  }

  return result;
}

}  // namespace intrinsic::kinematics
