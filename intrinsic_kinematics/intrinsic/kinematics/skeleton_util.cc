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

#include "intrinsic/kinematics/skeleton_util.h"

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {
namespace kinematics {

absl::StatusOr<std::unique_ptr<kinematics::Skeleton>> GetSkeletonForRobot(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot) {
  INTR_ASSIGN_OR_RETURN(
      const RobotComponent* component,
      object_world.GetEntityWorld().GetComponentByEntityId<RobotComponent>(
          robot.GetRobotEntityId()));
  if (component->GetSolvableFrames().size() != 1) {
    return absl::InternalError(
        absl::StrCat("The expected number of solvable frames is 1, but got ",
                     component->GetSolvableFrames().size(), "."));
  }
  AttachmentEntityId robot_tool_tip_id =
      component->GetSolvableFrames().begin()->second;
  INTR_ASSIGN_OR_RETURN(
      LinkEntityId robot_base_id,
      object_world.GetEntityWorld().GetBaseLink(robot.GetRobotEntityId()));
  return object_world.GetEntityWorld().BuildChainSkeleton(robot_base_id,
                                                          robot_tool_tip_id);
}

}  // namespace kinematics
}  // namespace intrinsic
