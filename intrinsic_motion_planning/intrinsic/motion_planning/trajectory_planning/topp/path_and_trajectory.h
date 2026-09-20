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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PATH_AND_TRAJECTORY_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PATH_AND_TRAJECTORY_H_

#include "intrinsic/motion_planning/trajectory_planning/path_refinement/spline_based_path_refinement.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_trajectory_result.h"

namespace intrinsic::topp {

// Struct to hold the path samples and the trajectory resulting from a path
// refinement and a trajectory optimization methods.
struct PathAndTrajectory {
  ToppTrajectoryResult trajectory_result;
  SplineBasedPathRefinementResult path_result;
};

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PATH_AND_TRAJECTORY_H_
