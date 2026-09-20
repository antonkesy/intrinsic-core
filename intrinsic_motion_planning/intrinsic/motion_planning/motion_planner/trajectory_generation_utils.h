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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_GENERATION_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_GENERATION_UTILS_H_

#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/path_sample.h"

namespace intrinsic {

inline constexpr double kMinimumLowLevelJointBlendingDeviationRad = 1e-5;

// Contains the two waypoints of a line-segment inside a `PathSegment`, whose
// blending radius may be locally adjusted. NOTE: This is a line segment of two
// consecutive waypoints (not to be confused with a `PathSegment` that may
// contain many waypoints). It should hold that `first_point_index <=
// second_point_index` with equality if it is a degenarate line (single point).
struct AdjustedLineSegment {
  // The segment index in the `PathSegment`.
  int segment_index;
  // The indices of the first and second waypoints in
  // `PathSegment[segment_index].joint_configurations`.
  int first_point_index;
  int second_point_index;
};

// Sets the blending radius of the specified waypoint to the minimum value.
// If the waypoint is at a segment boundary, also updates the adjacent
// segment's shared waypoint to maintain boundary consistency.
absl::Status SetWaypointBlendingToMinimum(
    int waypoint_segment_index, int waypoint_joint_index,
    absl::Span<PathSegment> path_segments);

// Locally reduces the blending radius of waypoints near the collision samples
// in `invalid_sample_indices` (assumed sorted).
// Maps each collision sample to the closest line segment between consecutive
// waypoints, and uses a heuristic (based on relative position) to tighten the
// start, end, or both waypoints of that line segment.
// Returns the modified line segments so that downstream logic can apply
// a stricter fallback (tightening both endpoints) if this heuristic fails.
absl::StatusOr<std::vector<AdjustedLineSegment>> AdjustPathBlendingLocally(
    absl::Span<PathSegment> path_segments,
    absl::Span<const topp::PathSample> spline_path_samples,
    absl::Span<const int> invalid_sample_indices);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_GENERATION_UTILS_H_
