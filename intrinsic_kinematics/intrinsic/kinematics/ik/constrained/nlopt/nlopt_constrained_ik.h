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

#ifndef INTRINSIC_KINEMATICS_IK_CONSTRAINED_NLOPT_NLOPT_CONSTRAINED_IK_H_
#define INTRINSIC_KINEMATICS_IK_CONSTRAINED_NLOPT_NLOPT_CONSTRAINED_IK_H_

#include <memory>
#include <optional>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/math/numopt/nlopt_nonlinear_program.h"
#include "nlopt.hpp"

namespace intrinsic::kinematics {

struct ConstrainedIKOptions {
  // Optional user-provided initial guess for the constrained IK problem.
  std::optional<eigenmath::VectorNd> joint_position_initial_guess;
  // Max solver attempts (with random re-initialization) in case of failure.
  int max_solver_attempts = 10;
  // Halton random sequence index, can optionally be configured to allow to
  // reproduce certain IK failure/success cases.
  int halton_sequence_index = 0;
};

// Implements constrained inverse kinematics using the nonlinear programming
// solver NLOpt.
// Note: this class is purposefully not inheriting from
// 'kinematics::ConstrainedIKInterface' in order to allow strict separation of
// interface and the implementation of solver functionality and details.
class NLOptConstrainedIK {
 public:
  struct Result {
    // NLOpt solver status according to solver's internal report.
    NLOptSolver::Result::SolveStatus status = NLOptSolver::Result::NOT_SOLVED;
    // Solution for the constrained IK problem, filled out only if a solution
    // was found.
    std::optional<eigenmath::VectorXd> q_solution;
    // Number of attempts taken until solution was found.
    int number_of_attempts = 0;
    // Optimal cost (cost functions evaluated at 'q_solution'), filled out only
    // if a solution was found.
    std::optional<double> cost;
    // Overall time spent in NLOpt across all 'number_of_attempts'.
    absl::Duration measured_processing_time = absl::ZeroDuration();
  };

  // Create an 'NLOptConstrainedIK' solver instance from a given NLOptProblem
  // 'problem' consisting of cost functions and constraints. Requires a
  // kinematic 'chain' for the robot employed in the problem. Optionally allows
  // to switch the underlying NLOpt optmization algorithm for special use-cases,
  // however it is recommended to stick to the default solver (NLOPT_LD_SLSQP)
  // wherever possible. The provided 'chain' must outlive the class.
  static absl::StatusOr<std::unique_ptr<NLOptConstrainedIK>> Create(
      std::unique_ptr<NLOptProblem> problem, const Chain* chain,
      nlopt_algorithm algorithm = NLOptSolver::kDefaultAlgorithm);

  // Solve constrained IK problem using 'options'. Returns an absl::Status in
  // case of internal failures, such as dimension mismatch, returns a 'Result'
  // struct otherwise, i.e. even if no solution was found. Use 'status' field
  // from Result to examine solver success and solution.
  absl::StatusOr<Result> Solve(
      const ConstrainedIKOptions& options = ConstrainedIKOptions(),
      const NLOptOptions& nlopt_options = NLOptOptions());

 private:
  NLOptConstrainedIK(std::unique_ptr<NLOptSolver> nlopt_solver,
                     const Chain* chain)
      : solver_(std::move(nlopt_solver)), chain_(chain) {}

  std::unique_ptr<NLOptSolver> solver_;

  const Chain* chain_;
};

}  // namespace intrinsic::kinematics

#endif  // INTRINSIC_KINEMATICS_IK_CONSTRAINED_NLOPT_NLOPT_CONSTRAINED_IK_H_
