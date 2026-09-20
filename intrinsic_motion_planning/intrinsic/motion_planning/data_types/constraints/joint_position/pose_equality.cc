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

#include "intrinsic/motion_planning/data_types/constraints/joint_position/pose_equality.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/proto/point.pb.h"
#include "intrinsic/math/proto/pose.pb.h"
#include "intrinsic/math/proto/quaternion.pb.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/position_equality.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/rotation_equality.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"

namespace intrinsic {
namespace motion_planning {

using ::intrinsic_proto::motion_planning::v1::PositionEquality;
using ::intrinsic_proto::motion_planning::v1::RotationEquality;

absl::StatusOr<std::vector<std::unique_ptr<ConstraintInterface>>>
CreatePoseEquality(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::PoseEquality& proto_constraint,
    double position_tolerance, double rotation_tolerance) {
  // Deconstruct the pose equality constraint into a position equality and
  // rotation equality.
  PositionEquality proto_position_constraint;
  *proto_position_constraint.mutable_moving_frame() =
      proto_constraint.moving_frame();
  *proto_position_constraint.mutable_target_frame() =
      proto_constraint.target_frame();

  RotationEquality proto_rotation_constraint;
  *proto_rotation_constraint.mutable_moving_frame() =
      proto_constraint.moving_frame();
  *proto_rotation_constraint.mutable_target_frame() =
      proto_constraint.target_frame();

  if (proto_constraint.has_target_frame_offset()) {
    *proto_position_constraint.mutable_target_frame_offset() =
        proto_constraint.target_frame_offset().position();
    *proto_rotation_constraint.mutable_rotation_offset() =
        proto_constraint.target_frame_offset().orientation();
  }

  INTR_ASSIGN_OR_RETURN(
      auto position_constraint,
      PositionEqualityConstraint::Create(world, proto_position_constraint,
                                         position_tolerance));
  INTR_ASSIGN_OR_RETURN(
      auto rotation_constraint,
      RotationEqualityConstraint::Create(world, proto_rotation_constraint,
                                         rotation_tolerance));

  std::vector<std::unique_ptr<ConstraintInterface>> constraints;
  constraints.reserve(2);
  constraints.push_back(std::move(position_constraint));
  constraints.push_back(std::move(rotation_constraint));
  return constraints;
}

}  // namespace motion_planning
}  // namespace intrinsic
