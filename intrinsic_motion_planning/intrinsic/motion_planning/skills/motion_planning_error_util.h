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

#ifndef INTRINSIC_MOTION_PLANNING_SKILLS_MOTION_PLANNING_ERROR_UTIL_H_
#define INTRINSIC_MOTION_PLANNING_SKILLS_MOTION_PLANNING_ERROR_UTIL_H_

#include <cstdint>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"

namespace intrinsic::skills {

// Error codes for motion planning errors, following the guideline in
// go/intrinsic-extended-status-guide#heading=h.8cqb9vpkhg6q .
// The spacings between error codes are based on the occurrence statistics in
// go/intrinsic-motion-planning-error-messages-stats .
constexpr uint32_t kCollisionErrorCode = 10201;
constexpr uint32_t kIKErrorCode = 10301;
// Generic error code for Linear Cartesian Path Planning errors.
constexpr uint32_t kLinearCartesianPathPlanningErrorCode = 10401;
// Error code for Linear Cartesian Path Planning errors which occur in the
// FinePathIK planner.
constexpr uint32_t kFinePathIKErrorCode = 10402;
constexpr uint32_t kJointLimitErrorCode = 10501;
// Generic error code for motion planning errors.
constexpr uint32_t kMotionPlanningErrorCode = 10601;

// Get the Motion Planning `ExtendedStatus` error message for a given status,
// and fill-in the external report with a user-friendly/user-readable message if
// the status payload has a `MotionPlanningError` or `MotionPipelineError` proto
// in it.
absl::Status GetMotionPlanningExtendedStatusErrorMessage(
    absl::Status status, bool use_rad = true,
    absl::Time timestamp = absl::Now(),
    absl::string_view component = "ai.intrinsic.move_robot");

// Get the Motion Planning `ExtendedStatus` error message for a given status,
// error code, component (optional),  user message and internal message
// (optional). If no component is provided, the default component is
// "ai.intrinsic.move_robot".
absl::Status GetMotionPlanningExtendedStatusErrorMessage(
    const absl::Status& status, uint32_t error_code,
    absl::string_view user_message, absl::string_view debug_message = "",
    absl::Time timestamp = absl::Now(),
    absl::string_view component = "ai.intrinsic.move_robot");

// Returns the joint positions in degrees. Assumes that the input joint
// positions are in radians.
eigenmath::VectorNd ConvertRadiansToDegrees(
    const eigenmath::VectorNd& joint_positions);

}  // namespace intrinsic::skills

#endif  // INTRINSIC_MOTION_PLANNING_SKILLS_MOTION_PLANNING_ERROR_UTIL_H_
