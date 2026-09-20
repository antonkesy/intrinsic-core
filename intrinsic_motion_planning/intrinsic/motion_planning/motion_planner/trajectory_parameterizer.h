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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_PARAMETERIZER_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_PARAMETERIZER_H_

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/trajectory_with_optional_path_samples.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {

// Interface for trajectory parameterization.
//
// Implementations of this interface are responsible for path refinement (e.g.,
// joint-level blending) and time-parameterization to compute a feasible
// trajectory from a series of path segments.
class TrajectoryParameterizer {
 public:
  virtual ~TrajectoryParameterizer() = default;

  // Refines the path segments and computes the time-parameterized trajectory.
  //
  // Given a sequence of path segments, this method performs optimization (such
  // as fitting splines and running time-parameterization) to generate a
  // trajectory. The computed trajectory must respect kinematic limits,
  // collision constraints, and other parameters specified by the motion
  // planner flags.
  //
  // `world` represents the environment (e.g. for collision checking).
  // `robot` is the kinematic robot object to plan the trajectory for.
  // `path_segments` represents the sequence of geometric path segments to
  // optimize.
  // `flags` contains configuration flags for the motion planner.
  //
  // Returns the optimized trajectory along with optional path samples on
  // success, or an error if the path optimization fails.
  virtual absl::StatusOr<topp::TrajectoryWithOptionalPathSamples>
  ComputeTrajectory(const object_world::ObjectWorld& world,
                    const object_world::KinematicObject& robot,
                    absl::Span<const PathSegment> path_segments,
                    const MotionPlannerFlags& flags) const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_PARAMETERIZER_H_
