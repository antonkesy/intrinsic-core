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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_ACCELERATION_LIMITED_TRAJECTORY_PARAMETERIZER_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_ACCELERATION_LIMITED_TRAJECTORY_PARAMETERIZER_H_

#include "absl/base/attributes.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/math/spline/bspline_sampler.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"
#include "intrinsic/motion_planning/motion_planner/trajectory_parameterizer.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/trajectory_with_optional_path_samples.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {

// An implementation of `TrajectoryParameterizer` that uses Time-Optimal Path
// Parameterization via Reachability Analysis (TOPP-RA) to compute
// acceleration-limited trajectories.
//
// This optimizer refines path segments using the provided `SplineSampler`
// and computes the trajectory assuming joint velocity/acceleration and
// Cartesian velocity/acceleration limits, ignoring jerk limits.
class AccelerationLimitedTrajectoryParameterizer
    : public TrajectoryParameterizer {
 public:
  // Constructs the optimizer with a specific sampler used for path refinement.
  // The sampler must outlive this optimizer instance.
  explicit AccelerationLimitedTrajectoryParameterizer(
      const BSplineSampler& sampler ABSL_ATTRIBUTE_LIFETIME_BOUND);
  ~AccelerationLimitedTrajectoryParameterizer() override = default;

  // Computes a time-optimal, acceleration-limited trajectory using TOPP-RA.
  //
  // Refines the path using the spline sampler and optimizes it subject to
  // joint velocity and acceleration limits, and Cartesian velocity and
  // acceleration limits.
  absl::StatusOr<topp::TrajectoryWithOptionalPathSamples> ComputeTrajectory(
      const object_world::ObjectWorld& world,
      const object_world::KinematicObject& robot,
      absl::Span<const PathSegment> path_segments,
      const MotionPlannerFlags& flags) const override;

 private:
  const BSplineSampler& sampler_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_ACCELERATION_LIMITED_TRAJECTORY_PARAMETERIZER_H_
