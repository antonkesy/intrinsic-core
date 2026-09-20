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

#ifndef INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_POSE_EQUALITY_H_
#define INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_POSE_EQUALITY_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {
namespace motion_planning {

// Binds a world to a proto specification of a pose equality constraint, and
// returns a vector containing two constraints: a `PositionEqualityConstraint`
// and then a `RotationEqualityConstraint` whose intersection of solutions
// corresponds to the solutions of the pose equality constraint.
//
// `position_tolerance` defines the tolerance of the returned position equality
// constraint, and its units are squared meters.
// `rotation_tolerance` defines the tolerance of the returned rotation equality
// constraint, and its units are radians.
absl::StatusOr<std::vector<std::unique_ptr<ConstraintInterface>>>
CreatePoseEquality(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::PoseEquality& proto_constraint,
    double position_tolerance = 1e-2, double rotation_tolerance = 1e-4);

}  // namespace motion_planning
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_JOINT_POSITION_POSE_EQUALITY_H_
