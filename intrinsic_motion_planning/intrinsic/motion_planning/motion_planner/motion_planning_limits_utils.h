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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNING_LIMITS_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNING_LIMITS_UTILS_H_

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_limits.pb.h"

namespace intrinsic {

// Returns the most relaxed `DynamicCartesianLimits` proto combination of
// `dynamic_cartesian_limits_1` and `dynamic_cartesian_limits_2`.
absl::StatusOr<intrinsic_proto::motion_planning::v1::DynamicCartesianLimits>
GetMostRelaxedDynamicCartesianLimitsCombination(
    const intrinsic_proto::motion_planning::v1::DynamicCartesianLimits&
        dynamic_cartesian_limits_1,
    const intrinsic_proto::motion_planning::v1::DynamicCartesianLimits&
        dynamic_cartesian_limits_2);

// Returns the most relaxed `JointLimitsUpdate` proto combination of
// `joint_limits_update_1` and `joint_limits_update_2`.
absl::StatusOr<intrinsic_proto::motion_planning::v1::JointLimitsUpdate>
GetMostRelaxedJointLimitsUpdateCombination(
    const intrinsic_proto::motion_planning::v1::JointLimitsUpdate&
        joint_limits_update_1,
    const intrinsic_proto::motion_planning::v1::JointLimitsUpdate&
        joint_limits_update_2);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNING_LIMITS_UTILS_H_
