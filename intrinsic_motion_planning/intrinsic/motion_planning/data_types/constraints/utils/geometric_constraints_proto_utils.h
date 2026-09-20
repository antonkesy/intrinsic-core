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

#ifndef INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_UTILS_GEOMETRIC_CONSTRAINTS_PROTO_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_UTILS_GEOMETRIC_CONSTRAINTS_PROTO_UTILS_H_

#include <string_view>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {

// Generates a geometric_constraint proto of type JointPositionLimits given the
// robot and the lower and upper limits.
intrinsic_proto::motion_planning::v1::JointPositionLimits
ToJointPositionLimitProto(const object_world::KinematicObject& robot,
                          const eigenmath::VectorNd& min_position,
                          const eigenmath::VectorNd& max_position);

absl::StatusOr<eigenmath::VectorNd> FromProto(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::JointPositionEquality&
        joint_position_proto);

// Extracts all geometric constraints of type `constraint_case` from
// `geometric_constraints` and returns them as a vector of GeometricConstraints.
std::vector<intrinsic_proto::motion_planning::v1::GeometricConstraint>
ExtractConstraintType(
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint,
    intrinsic_proto::motion_planning::v1::GeometricConstraint::ConstraintCase
        constraint_case);

// Returns the AttachmentEntityId for a TransformNodeReference 'reference'. In
// case of en error occurring, the methods offers an optional message prepend.
absl::StatusOr<AttachmentEntityId> GetOriginEntityIdForTransformNodeReference(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::TransformNodeReference& reference,
    std::string_view error_message_prepend = "",
    std::string_view error_message_append = "");

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_DATA_TYPES_CONSTRAINTS_UTILS_GEOMETRIC_CONSTRAINTS_PROTO_UTILS_H_
