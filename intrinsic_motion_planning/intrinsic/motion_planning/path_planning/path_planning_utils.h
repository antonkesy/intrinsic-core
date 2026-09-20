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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNING_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNING_UTILS_H_

#include <cstddef>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"

namespace intrinsic {

// Computes the ratio of point/edge violation for a given path based on
// counting:
//
//       (number of point violation) + (number of edge violation)
// r =  ----------------------------------------------------------
//          (total number of points) + (total number of edges)
//
absl::StatusOr<double> ComputeViolationRatio(
    const PointPath& path, const PointValidator& point_validator,
    const EdgeValidator& edge_validator);

// Computes the ratio of violation based on edge length:
//
//          total length of invalid edges
// r = ----------------------------------------
//            total length of all edges
//
absl::StatusOr<double> ComputeViolationLengthRatio(
    const PointPath& path, const EdgeValidator& edge_validator);

absl::StatusOr<JointLimitsXd> GetSamplingLimitsForPlanningProblem(
    const PointPath& path, const JointLimitsXd& limits);

// For a given `manipulator_kinematics` and `joint_configuration`, this function
// returns the smallest singular value of the corresponding Jacobian.
icon::RealtimeStatusOr<double> GetJacobianSmallestSingularValue(
    const icon::ManipulatorKinematics* kinematics,
    const eigenmath::VectorNd& joint_configuration);

// For a given kinematics `chain` and `joint_configuration`, this function
// returns the smallest singular value of the corresponding Jacobian.
icon::RealtimeStatusOr<double> GetJacobianSmallestSingularValue(
    const kinematics::Chain& chain,
    const eigenmath::VectorNd& joint_configuration);
// Computes the total length of a piecewise linear path of configuration
// waypoints.
double ComputePointPathLength(const PointPath& path);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNING_UTILS_H_
