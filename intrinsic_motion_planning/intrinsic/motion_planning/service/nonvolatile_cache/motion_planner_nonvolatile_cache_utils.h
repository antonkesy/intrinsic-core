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

#ifndef INTRINSIC_MOTION_PLANNING_SERVICE_NONVOLATILE_CACHE_MOTION_PLANNER_NONVOLATILE_CACHE_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_SERVICE_NONVOLATILE_CACHE_MOTION_PLANNER_NONVOLATILE_CACHE_UTILS_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_config.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/motion_planning/service/nonvolatile_cache/motion_planner_nonvolatile_cache.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"

namespace intrinsic {

// Get LockMotionConfiguration from the given MotionPlanningRequest if exists.
std::optional<intrinsic_proto::motion_planning::v1::LockMotionConfiguration>
GetLockMotionConfiguration(
    const intrinsic_proto::motion_planning::v1::MotionPlanningRequest& request);

// Try to load and return a motion from the nonvolatile cache if it is
// requested. If the request also specifies motion segments to replan, the
// function will try to construct a new MotionSpecification based on the
// MotionSpecification in the request and the loaded path segments. Then it will
// call MotionPlanner::PlanTrajectory to plan a trajectory for the new
// MotionSpecification.
// Optionally, for Assetized MotionPlannerService (MPS), mps_asset_major_version
// can be supplied for the MotionPlannerNonvolatileCacheKey creation.
// Return std::nullopt if the request does not specify a motion to load.
// Return an InvalidArgumentError if the request misses the motion id or if
// the given cache is a nullptr.
// Return a NotFoundError if the request specifies a motion to load but a
// matching motion cannot be found in the cache.
// Return the motion once it is successfully loaded/planned.
absl::StatusOr<std::optional<MotionPlanner::PlanTrajectoryResult>>
LoadMotionFromNonvolatileCacheIfRequested(
    const MotionPlanner& motion_planner,
    MotionPlannerNonvolatileCache* plan_trajectory_nonvolatile_cache,
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::MotionPlanningRequest& request,
    double collision_check_spacing,
    std::optional<std::string> mps_asset_major_version = std::nullopt);

// Try to save a motion to the nonvolatile cache if it is requested and return
// its id for future reference.
// Optionally, for Assetized MotionPlannerService (MPS), mps_asset_major_version
// can be supplied for the MotionPlannerNonvolatileCacheKey creation.
// Return std::nullopt if the request does not specify a motion to save.
// Return an InvalidArgumentError if the given cache is a nullptr.
// Return the motion id once the motion is successfully saved.
absl::StatusOr<std::optional<std::string>>
SaveMotionToNonvolatileCacheIfRequested(
    MotionPlannerNonvolatileCache* plan_trajectory_nonvolatile_cache,
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::MotionPlanningRequest& request,
    const MotionPlanner::PlanTrajectoryResult& plan_trajectory_result,
    std::optional<std::string> mps_asset_major_version = std::nullopt);

// Construct and return a list of JointPositionEquality constraints from the
// given PathSegment and robot id. Each joint configuration in the PathSegment
// will be converted to a JointPositionEquality constraint.
std::vector<intrinsic_proto::motion_planning::v1::JointPositionEquality>
GetJointPositionEqualityConstraintsFromPathSegment(
    const PathSegment& path_segment,
    const intrinsic_proto::world::ObjectReference& robot_id);

// Construct and return a list of MotionSegments from the given
// original_motion_segment and PathSegment. Each joint configuration in the
// PathSegment will be converted to a MotionSegment with a
// JointPositionEquality constraint.
std::vector<intrinsic_proto::motion_planning::v1::MotionSegment>
GetMotionSegmentsFromPathSegment(
    const intrinsic_proto::motion_planning::v1::MotionSegment&
        original_motion_segment,
    const PathSegment& path_segment,
    const intrinsic_proto::world::ObjectReference& robot_id);

// Construct and return a MotionSpecification from the given
// original_motion_specification, motion_segment_ids_to_keep, path_segments and
// robot id. The motion_segment_ids_to_keep are the ids of the motion segments
// to keep from the original_motion_specification. The rest of motion segments
// will be replaced by new segments created from the corresponding path
// segments. The motion target of the new motion segments are
// JointPositionEquality constraints constructed from the corresponding path
// segments.
// Returns an InvalidArgumentError if the number of motion segments in
// the original_motion_specification does not match the number of path segments.
// Returns an InvalidArgumentError if the max motion segment id in the
// motion_segment_ids_to_keep is greater or equal to the number of motion
// segments in the original_motion_specification.
absl::StatusOr<intrinsic_proto::motion_planning::v1::MotionSpecification>
GetMotionSpecificationFromPathSegments(
    const intrinsic_proto::motion_planning::v1::MotionSpecification&
        original_motion_specification,
    const absl::flat_hash_set<uint32_t>& motion_segment_ids_to_keep,
    absl::Span<const PathSegment> path_segments,
    const intrinsic_proto::world::ObjectReference& robot_id);

// Construct and return a PlanTrajectoryResult from the given args.
// This function first calls GetMotionSpecificationFromPathSegments to
// construct a new MotionSpecification based on the MotionSpecification in the
// given request and the given path segments. Then it calls
// MotionPlanner::PlanTrajectory to plan a trajectory for the new
// MotionSpecification.
absl::StatusOr<MotionPlanner::PlanTrajectoryResult>
ConstructNewMotionSpecificationAndPlanTrajectory(
    const MotionPlanner& motion_planner,
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::MotionPlanningRequest& request,
    const absl::flat_hash_set<uint32_t>& motion_segment_ids_to_keep,
    absl::Span<const PathSegment> path_segments);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_SERVICE_NONVOLATILE_CACHE_MOTION_PLANNER_NONVOLATILE_CACHE_UTILS_H_
