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

#include "intrinsic/motion_planning/data_types/constraints/utils/geometric_constraints_proto_utils.h"

#include <queue>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"

namespace intrinsic {

using intrinsic_proto::motion_planning::v1::GeometricConstraint;
using GeometricConstraintCase =
    intrinsic_proto::motion_planning::v1::GeometricConstraint::ConstraintCase;

intrinsic_proto::motion_planning::v1::JointPositionLimits
ToJointPositionLimitProto(const object_world::KinematicObject& robot,
                          const eigenmath::VectorNd& min_position,
                          const eigenmath::VectorNd& max_position) {
  intrinsic_proto::motion_planning::v1::JointPositionLimits
      joint_position_proto;
  VectorNdToRepeatedDouble(min_position,
                           joint_position_proto.mutable_lower_limits());
  VectorNdToRepeatedDouble(max_position,
                           joint_position_proto.mutable_upper_limits());
  joint_position_proto.mutable_object_id()->set_id(robot.GetId().value());
  return joint_position_proto;
}

absl::StatusOr<eigenmath::VectorNd> FromProto(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::JointPositionEquality&
        joint_position_proto) {
  switch (joint_position_proto.joint_group_case()) {
    case intrinsic_proto::motion_planning::v1::JointPositionEquality::
        JointGroupCase::kObjectId: {
      INTR_ASSIGN_OR_RETURN(
          auto robot_for_limits,
          object_world::GetKinematicObjectByReference(
              object_world, joint_position_proto.object_id()));
      if (robot_for_limits == nullptr) {
        return absl::InvalidArgumentError(
            "Cannot parse kinematic object from joint limit proto.");
      }

      if (robot_for_limits->GetRobotEntityId() != robot.GetRobotEntityId()) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Robot for path limits (id = %i ) is different than the robot "
            "specified for planning (id = %i).",
            robot_for_limits->GetRobotEntityId().value(),
            robot.GetRobotEntityId().value()));
      }
      // Check dimensionality of parsed configs
      INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd config,
                            robot.GetJointPositions());
      if (config.size() !=
          joint_position_proto.joint_positions().joints().size()) {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Provided joint configuration does not contain the number of "
            "fields required by this robot. Desired number of joint position "
            "fields = %i, provided number of fields = %i",
            config.size(),
            joint_position_proto.joint_positions().joints().size()));
      }
      break;
    }
    case intrinsic_proto::motion_planning::v1::JointPositionEquality::
        JointGroupCase::kJointIds: {
      return absl::InvalidArgumentError(
          "Using individual JointIds for joint limit constraints not yet "
          "supported.");
    }
    case intrinsic_proto::motion_planning::v1::JointPositionEquality::
        JointGroupCase::JOINT_GROUP_NOT_SET: {
      return absl::InvalidArgumentError("Robot reference was unspecified.");
    }
  }

  return icon::RepeatedDoubleToVectorNd(
      joint_position_proto.joint_positions().joints());
}

std::vector<intrinsic_proto::motion_planning::v1::GeometricConstraint>
ExtractConstraintType(const GeometricConstraint& geometric_constraint,
                      GeometricConstraintCase constraint_case) {
  std::vector<GeometricConstraint> result;
  std::queue<GeometricConstraint> queue;
  queue.push(geometric_constraint);
  while (!queue.empty()) {
    const auto constraint = queue.front();
    queue.pop();
    if (constraint.has_constraint_intersection()) {
      for (const auto& element :
           constraint.constraint_intersection().constraints()) {
        queue.push(element);
        if (constraint.constraint_case() == constraint_case) {
          result.push_back(constraint);
        }
      }
      continue;
    }
    if (constraint.constraint_case() == constraint_case) {
      result.push_back(constraint);
    }
  }
  return result;
}

absl::StatusOr<AttachmentEntityId> GetOriginEntityIdForTransformNodeReference(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::TransformNodeReference& reference,
    std::string_view error_message_prepend,
    std::string_view error_message_append) {
  auto node_request_status =
      object_world::GetTransformNodeByReference(object_world, reference);
  if (node_request_status.ok()) {
    return node_request_status.value()->GetTransformOriginEntityId();
  }
  return absl::InvalidArgumentError(absl::StrFormat(
      "%s %s %s", error_message_prepend, node_request_status.status().message(),
      error_message_append));
}

}  // namespace intrinsic
