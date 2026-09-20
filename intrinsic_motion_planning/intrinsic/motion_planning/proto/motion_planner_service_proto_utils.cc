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

#include "intrinsic/motion_planning/proto/motion_planner_service_proto_utils.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/motion_planning/proto/v1/robot_specification.pb.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"

namespace intrinsic {

// TODO(keegang): While the switching below may be too much for just a single
// case, we want to remain flexible to handle other specification options in the
// future (for example, arm+rail). The return type may need to be changed to
// make this work.
absl::StatusOr<const object_world::KinematicObject*> GetRobot(
    const intrinsic_proto::motion_planning::v1::RobotReference& robot_reference,
    const object_world::ObjectWorld* object_world) {
  switch (robot_reference.robot_reference_case()) {
    case intrinsic_proto::motion_planning::v1::RobotReference::
        RobotReferenceCase::kObjectId: {
      return object_world::GetKinematicObjectByReference(
          *object_world, robot_reference.object_id());
    }
    case intrinsic_proto::motion_planning::v1::RobotReference::
        RobotReferenceCase::ROBOT_REFERENCE_NOT_SET: {
      return absl::InvalidArgumentError("Robot reference was unspecified.");
    }
  }
}

absl::StatusOr<object_world::KinematicObject*> GetRobot(
    const intrinsic_proto::motion_planning::v1::RobotReference& robot_reference,
    object_world::ObjectWorld* object_world) {
  switch (robot_reference.robot_reference_case()) {
    case intrinsic_proto::motion_planning::v1::RobotReference::
        RobotReferenceCase::kObjectId: {
      return object_world::GetKinematicObjectByReference(
          *object_world, robot_reference.object_id());
    }
    case intrinsic_proto::motion_planning::v1::RobotReference::
        RobotReferenceCase::ROBOT_REFERENCE_NOT_SET: {
      return absl::InvalidArgumentError("Robot reference was unspecified.");
    }
  }
}

intrinsic_proto::motion_planning::v1::RobotReference CreateRobotReferenceByName(
    absl::string_view robot_name) {
  intrinsic_proto::motion_planning::v1::RobotReference robot_reference;
  robot_reference.mutable_object_id()->mutable_by_name()->set_object_name(
      robot_name);
  return robot_reference;
}

}  // namespace intrinsic
