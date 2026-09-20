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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNER_TEST_UTIL_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNER_TEST_UTIL_H_

#include <optional>

#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/proto/v1/motion_blending_parameter.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/path_sample.h"

namespace intrinsic::testing {
// Returns OkStatus if every sample in `path_samples` is within a very small
// error tolerance of the path segments in `path_segments`. Returns an
// error otherwise. This is intended to test that strict fallback trajectory
// generation is indeed strict. If `every_sample_must_match_a_segment` is true,
// then if any path samples do not correspond to a segment in `path_segments`,
// we return an error; if `every_sample_must_match_a_segment` is false, then the
// samples that don't match any segments won't be checked for strictness.
absl::Status CheckThatPathSamplesStrictlyFollowPathSegments(
    absl::Span<const topp::PathSample> path_samples,
    absl::Span<const PathSegment> path_segments,
    bool every_sample_must_match_a_segment = false);

}  // namespace intrinsic::testing

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNER_TEST_UTIL_H_
