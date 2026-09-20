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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TRAJECTORY_WITH_OPTIONAL_PATH_SAMPLES_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TRAJECTORY_WITH_OPTIONAL_PATH_SAMPLES_H_

#include <optional>
#include <vector>

#include "intrinsic/motion_planning/trajectory_planning/topp/path_sample.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_trajectory_result.h"

namespace intrinsic::topp {

// A structure that bundles the results of a trajectory optimization request.
// It contains the generated time-optimal trajectory, and optionally, the path
// samples generated during path refinement (which can be used for debugging or
// visualization).
struct TrajectoryWithOptionalPathSamples {
  ToppTrajectoryResult topp_trajectory_result;
  std::optional<std::vector<PathSample>> path_samples = std::nullopt;
};

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TRAJECTORY_WITH_OPTIONAL_PATH_SAMPLES_H_
