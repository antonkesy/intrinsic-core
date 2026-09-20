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

#include "intrinsic/kinematics/ik/sanitize_ik.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "Eigen/Core"
#include "absl/log/check.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace kinematics {
namespace sanitize_ik {

int WrapToLimits(absl::Span<JointStateP> solutions, const JointLimits& limits) {
  const int num_original_solutions = solutions.size();
  int num_removed_solutions = 0;
  for (int i = 0, ix = 0; i < num_original_solutions; i++) {
    bool valid_solution = true;
    for (int dof_index = 0; dof_index < solutions[ix].position.size();
         dof_index++) {
      int num_windings = floor(
          (solutions[ix].position[dof_index] - limits.min_position[dof_index]) /
          (2 * M_PI));
      solutions[ix].position[dof_index] -= num_windings * 2 * M_PI;
      if (solutions[ix].position[dof_index] > limits.max_position[dof_index]) {
        // Since removing from a std::vector takes linear time, we instead swap
        // this element with the last element, decrease the number of
        // `solutions`, and maintain the index `ix` at the same value.  This
        // takes O(1) time but does not preserve the ordering of elements (a
        // worthwhile tradeoff for us).
        std::swap(
            solutions[ix],
            solutions[num_original_solutions - 1 - num_removed_solutions]);
        num_removed_solutions++;
        valid_solution = false;
        break;
      }
    }
    if (valid_solution) {
      // This solution is valid. Proceed to the next value in the span.
      ix++;
    }
  }
  return num_original_solutions - num_removed_solutions;
}

namespace {
// Recursive routine that appends the cartesian product of possible windings to
// `solutions`.  The recursion is on `joint_idx`.  The joints with index (<
// joint_idx) are fixed, and all combinations of windings for joints with index
// (>= joint_idx) are generated and appended to `solutions`.
void ExpandWindingsForSolutionJointRecursive(JointStateP* scratch,
                                             absl::Span<JointStateP> solutions,
                                             int provided_solution_idx,
                                             int dofs, int joint_idx,
                                             const JointLimits& limits,
                                             int* num_current_solutions) {
  CHECK(num_current_solutions != nullptr);
  CHECK(scratch != nullptr);
  if (*num_current_solutions >= solutions.size()) {
    // Base case: no more space for further solutions.
    return;
  }
  if (joint_idx == dofs) {
    // Base case: we've set values for all joints in `scratch`.  Append it to
    // `solutions`.

    // It is possible that we've duplicated solutions[provided_solution_idx] by
    // picking k=0 for every joint, so skip in that case.
    if (scratch->position.isApprox(solutions[provided_solution_idx].position)) {
      return;
    }
    solutions[*num_current_solutions] = *scratch;
    (*num_current_solutions)++;
    return;
  }

  // Recursion on joint_idx.

  // Try subtracting multiples of TWO_PI (starting from 1).
  double v = solutions[provided_solution_idx].position[joint_idx] - 2 * M_PI;
  while (v > limits.min_position[joint_idx]) {
    scratch->position[joint_idx] = v;
    ExpandWindingsForSolutionJointRecursive(
        scratch, solutions, provided_solution_idx, dofs, joint_idx + 1, limits,
        num_current_solutions);
    v -= 2 * M_PI;
  }

  // Try adding multiples of TWO_PI (starting from 0).
  v = solutions[provided_solution_idx].position[joint_idx];
  while (v < limits.max_position[joint_idx]) {
    scratch->position[joint_idx] = v;
    ExpandWindingsForSolutionJointRecursive(
        scratch, solutions, provided_solution_idx, dofs, joint_idx + 1, limits,
        num_current_solutions);
    v += 2 * M_PI;
  }
}
}  // namespace

icon::RealtimeStatusOr<int> ExpandWindings(absl::Span<JointStateP> solutions,
                                           const JointLimits& limits,
                                           int num_provided_solutions) {
  const int max_solutions = solutions.size();
  if (num_provided_solutions > max_solutions) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "The number of provided solutions ", num_provided_solutions,
        " needs to be >= than the size of the span, ", max_solutions));
  }
  if (num_provided_solutions == 0) {
    return num_provided_solutions;
  }
  int dofs = solutions[0].position.size();

  JointStateP scratch;
  INTRINSIC_RT_RETURN_IF_ERROR(scratch.SetSize(dofs));
  int num_current_solutions = num_provided_solutions;
  for (int i = 0; i < num_provided_solutions; i++) {
    ExpandWindingsForSolutionJointRecursive(&scratch, solutions, i, dofs, 0,
                                            limits, &num_current_solutions);
  }
  return num_current_solutions;
}

}  // namespace sanitize_ik
}  // namespace kinematics
}  // namespace intrinsic
