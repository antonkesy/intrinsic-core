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

#include "intrinsic/world/aspects/entity_kinematic_utils.h"

#include <string>

#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {
namespace entity_kinematic_world_details {

absl::StatusOr<std::string> GetIKSolverKey(
    const entity_aspect_world_details::EntityWorld& world,
    const RobotCollectionsEntityId& robot_id) {
  INTR_ASSIGN_OR_RETURN(const WorldEntity* robot_ent,
                        world.GetEntityById(robot_id));
  INTR_ASSIGN_OR_RETURN(const RobotComponent* robot_component,
                        robot_ent->GetComponent<RobotComponent>());

  // Find the IDs of the robot's tip and base.
  INTR_ASSIGN_OR_RETURN(AttachmentEntityId tip_id,
                        world.GetFinalEntityOfRobotKinematicChain(robot_id));
  CHECK_NE(tip_id, kInvalidEntityId)
      << "cannot solve IK for non-linear chain robot with ID "
      << robot_id.value();
  INTR_ASSIGN_OR_RETURN(AttachmentEntityId base_id,
                        world.GetBaseLink(robot_id));

  // Look for an IK solver key in the following order:
  // 1) A specific solver for base_id to tip_id.
  // 2) A legacy solver key (see
  //    intrinsic/world/component/robot_component.h;rcl=314214976;l=83).
  // 3) Default of an empty string, which should map to the gradient descent
  //    solver.
  auto specific_solver_key_or =
      robot_component->GetSolverKeyForFrames(base_id, tip_id);
  if (specific_solver_key_or.ok()) {
    return specific_solver_key_or.value();
  } else {
    std::string solver_key;
    for (const auto& [frame_base_id, frame_tip_id] :
         robot_component->GetSolvableFrames()) {
      if (frame_base_id != kInvalidEntityId) {
        continue;
      }
      CHECK(solver_key.empty())
          << "Robot \"" << robot_ent->GetLocalName() << "\" (ID "
          << robot_id.value() << ") has multiple legacy solver keys";
      INTR_ASSIGN_OR_RETURN(solver_key, robot_component->GetSolverKeyForFrames(
                                            frame_base_id, frame_tip_id));
    }
    return solver_key;
  }
  return intrinsic::NotFoundErrorBuilder()
         << "could not find a solver key for robot_id=" << robot_id;
}

}  // namespace entity_kinematic_world_details
}  // namespace intrinsic
