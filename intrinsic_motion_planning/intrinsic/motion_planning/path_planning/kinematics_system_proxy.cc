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

#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"

#include "absl/container/flat_hash_set.h"

namespace intrinsic {

absl::StatusOr<std::vector<int>>
KinematicsSystemProxy::FindInvalidConfigurations(
    absl::Span<const eigenmath::VectorXd> configurations,
    const std::optional<int> max_invalid_results) const {
  std::vector<int> invalid_indices;
  const int max_invalid_results_value =
      max_invalid_results.value_or(std::numeric_limits<int>::max());

  for (int ii = 0; ii < configurations.size(); ++ii) {
    if (invalid_indices.size() >= max_invalid_results_value) break;

    INTR_ASSIGN_OR_RETURN(
        const JointConfigurationValidationResult result,
        IsValid(configurations[ii], /*collision_debug=*/nullptr));
    if (!static_cast<bool>(result)) {
      invalid_indices.push_back(ii);
    }
  }
  return invalid_indices;
}

void KinematicsSystemProxy::SetDistanceCheckStatistics(
    DistanceCheckStatistics* const statistics) {
  const std::shared_ptr<CollisionChecker> collision_checker =
      GetCollisionChecker();
  if (collision_checker != nullptr) {
    collision_checker->SetDistanceCheckStatistics(statistics);
  }
}

}  // namespace intrinsic
