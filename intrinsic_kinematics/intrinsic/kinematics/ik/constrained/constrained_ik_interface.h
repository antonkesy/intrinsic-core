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

#ifndef INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_INTERFACE_H_
#define INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_INTERFACE_H_

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/random/bit_gen_ref.h"
#include "absl/status/status.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_util.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/numopt/costfunction_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

// An interface to request IK solutions for its underlaying chain so that they
// satisfy the constraints and (locally) minimize the costs of the given problem
// in the form of a ConstrainedPoint.
// Note that while this class owns a Chain, such chain is only a view of the
// underlying Skeleton. Such Skeleton must be owned externally and outlive this
// class.
// NOTE : At this time this interface assumes that solvers providing this
// capability are non-RT safe. This can change in the future if we come up with
// RT-capable solvers.
class ConstrainedIKInterface : public InverseKinematicsInterfaceNonRT {
 public:
  explicit ConstrainedIKInterface(Chain chain)
      : InverseKinematicsInterfaceNonRT(std::move(chain)) {}

  // Sets the constrained IK problem to be solved by the Solve function.
  // This moves ownership of all unique pointers for constraints and costs into
  // this class.
  virtual absl::Status SetProblem(
      std::vector<std::unique_ptr<ConstraintInterface>>&& constraints,
      std::vector<std::unique_ptr<CostFunctionInterface>>&& costs) = 0;

  // Equivalent to the above, but limited to constraints and cost functions
  // with protos defined in `constrained_ik.proto`.
  virtual absl::Status SetProblem(
      const intrinsic_proto::kinematics::ConstrainedGoal& problem) = 0;

  // Describes the results of the Solve function.
  struct Result {
    enum Status {
      // Solution found and everything OK.
      OK,
      // No solution found satisfying all the constraints.
      NO_SOLUTION,
    };
    Status status = NO_SOLUTION;
    // The solution found by the solver when the status is OK. Otherwise,
    // nullopt.
    std::optional<JointStateP> q_star;
    // The cost achieved by the returned solution, if any.
    std::optional<double> cost;
    // The number of times that the solver attempted to find a solution from
    // some initial configuration.
    int number_of_attempts = 0;
    // Time that the solver took to find a solution or give up.
    absl::Duration elapsed_time = absl::ZeroDuration();
  };

  // Optional parameters that the solver will use with the Solve function.
  struct Options {
    // If provided, the initial guess that the solver will use to start the
    // search (for the first attempt).
    std::optional<JointStateP> joint_position_initial_guess;
    // The maximum number of times that the solver will try to solve the problem
    // from scratch using a different starting guess.
    int max_number_of_attempts = 10;
    // Halton random sequence index, can optionally be configured to allow to
    // reproduce certain IK failure/success cases.
    int halton_sequence_index = 0;
  };

  // Attempts to solve the constrained IK problem previously specified with
  // SetProblem by finding a configuration that satisfies the constraints and
  // minimizes the weighted sum of the costs.
  virtual absl::StatusOr<Result> Solve(const Options& options) const = 0;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_INTERFACE_H_
