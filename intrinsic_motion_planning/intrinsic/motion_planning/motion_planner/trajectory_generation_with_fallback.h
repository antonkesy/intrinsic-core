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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_GENERATION_WITH_FALLBACK_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_GENERATION_WITH_FALLBACK_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_interface.h"
#include "intrinsic/motion_planning/motion_planner/motion_planning_utils.h"
#include "intrinsic/motion_planning/motion_planner/trajectory_parameterizer.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/trajectory_with_optional_path_samples.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {

// Return value of GenerateTrajectoryWithFallback. Combines a trajectory and
// some useful metadata.
struct GenerateTrajectoryResult {
  topp::TrajectoryWithOptionalPathSamples trajectory_with_samples;

  // Counts only the time spent in path refinement and trajectory optimization.
  absl::Duration trajectory_generation_duration;

  // Counts the time spent creating the proxies for validation.
  absl::Duration validation_proxy_creation_duration;

  // Counts the time spent executing validation on the trajectory.
  absl::Duration validation_duration;

  // If we generated a fallback trajectory, this will be populated with the
  // initial, non-fallback trajectory.
  std::optional<topp::TrajectoryWithOptionalPathSamples>
      non_fallback_trajectory;

  // If this trajectory did not require fallback, this will be
  // `kNone`. Otherwise, it will contain the fallback stage that successfully
  // generated this trajectory.
  MotionPlannerInterface::FallbackStage fallback_stage =
      MotionPlannerInterface::FallbackStage::kNone;
};

// Generates a trajectory from the Path Planning output (`path_segments`).
// Performs validation of the output trajectory if enabled in `flags`. If
// validation fails and fallback is enabled in `flags`, this will generate a
// fallback trajectory and re-run validation.
//
// Note: `path_segments` are modified in-place if fallback is triggered (their
// joint blending parameters are set to minimum low-level blending deviation),
// so that the resulting path segments in the caller's output reflect the actual
// parameterization used for the fallback trajectory.
absl::StatusOr<GenerateTrajectoryResult> GenerateTrajectoryWithFallback(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::MotionPlannerConfiguration&
        motion_config,
    absl::Span<const PathSegment> path_segments,
    const TrajectoryParameterizer& trajectory_parameterizer,
    const MotionPlannerFlags& flags,
    const std::vector<ProxyCreateInfo>& proxy_set,
    const absl::flat_hash_map<std::string, std::vector<int>>&
        path_segment_to_validation_proxies_id_map,
    bool force_strict_fallback_for_testing = false);

// Generates a trajectory where, for some of the path segments, the trajectory
// should strictly follow the geometric path described by the path segment. If
// `path_segment_is_strict[i]` is true, then `path_segments[i]` will be strictly
// followed by the trajectory.
absl::StatusOr<topp::TrajectoryWithOptionalPathSamples>
GenerateTrajectoryWithStrictPathFollowingSegments(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    absl::Span<const PathSegment> path_segments,
    const TrajectoryParameterizer& trajectory_parameterizer,
    const MotionPlannerFlags& flags,
    const std::vector<bool>& path_segment_is_strict);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_TRAJECTORY_GENERATION_WITH_FALLBACK_H_
