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

#ifndef INTRINSIC_WORLD_UTIL_KINEMATIC_WORLD_UTIL_H_
#define INTRINSIC_WORLD_UTIL_KINEMATIC_WORLD_UTIL_H_

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/world/dof_kinematic_view.h"

namespace intrinsic {

// Returns true if the given dof values are within the limits from the provided
// kinematic view.
absl::StatusOr<bool> IsWithinDofLimits(const DofKinematicView& view,
                                       const eigenmath::VectorXd& dof_values);

// Gets a quasirandom joint configuration (using a Halton Sequence) within the
// joint limits defined by the given view.
eigenmath::VectorXd GetQuasiRandomDofConfiguration(const DofKinematicView& view,
                                                   int* seed);

// Get a random joint configuration from a uniform distribution within the
// joint limits defined by the given view. The Generator is expected to be a
// absl::BitGen.
template <typename Generator>
eigenmath::VectorXd GetUniformRandomDofConfiguration(
    const DofKinematicView& view, Generator& gen) {
  const auto [lower_limits, upper_limits] = view.GetDofValueApplicationLimits();
  ASSIGN_OR_DIE(
      eigenmath::VectorXd random_q,
      eigenmath::GetUniformRandomVectorXd(lower_limits, upper_limits, gen));
  return random_q;
}

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_KINEMATIC_WORLD_UTIL_H_
