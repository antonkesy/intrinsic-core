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

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/spline/uniform_between_knots_bspline_sampler.h"
#include "intrinsic/motion_planning/motion_planner/acceleration_limited_trajectory_parameterizer.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"

namespace intrinsic {

absl::StatusOr<std::unique_ptr<MotionPlanner>> MotionPlanner::Create(
    const MotionPlannerFlags& flags) {
  static const UniformBetweenKnotsBSplineSampler uniform_between_knots_sampler;

  auto parameterizer =
      std::make_unique<AccelerationLimitedTrajectoryParameterizer>(
          uniform_between_knots_sampler);

  return absl::WrapUnique(new MotionPlanner(flags, std::move(parameterizer)));
}

}  // namespace intrinsic
