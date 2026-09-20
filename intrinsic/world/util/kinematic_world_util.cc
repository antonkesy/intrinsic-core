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

#include "intrinsic/world/util/kinematic_world_util.h"

#include <cstddef>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/world/dof_kinematic_view.h"

namespace intrinsic {

absl::StatusOr<bool> IsWithinDofLimits(const DofKinematicView& view,
                                       const eigenmath::VectorXd& dof_values) {
  const auto [lower_limits, upper_limits] = view.GetDofValueApplicationLimits();
  if (dof_values.size() != upper_limits.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Mismatched number of dofs (got ", dof_values.size(),
        " dofs, but kinematic view expects ", upper_limits.size(), ")"));
  }

  for (size_t i = 0; i < dof_values.size(); ++i) {
    if (lower_limits[i] > dof_values[i]) {
      return false;
    }

    if (upper_limits[i] < dof_values[i]) {
      return false;
    }
  }

  return true;
}

eigenmath::VectorXd GetQuasiRandomDofConfiguration(const DofKinematicView& view,
                                                   int* seed) {
  const auto [lower_limits, upper_limits] = view.GetDofValueApplicationLimits();
  ASSIGN_OR_DIE(
      eigenmath::VectorXd random_q,
      eigenmath::GetQuasiRandomVectorXd(lower_limits, upper_limits, seed));
  return random_q;
}

}  // namespace intrinsic
