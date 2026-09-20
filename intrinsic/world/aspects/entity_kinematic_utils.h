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

#ifndef INTRINSIC_WORLD_ASPECTS_ENTITY_KINEMATIC_UTILS_H_
#define INTRINSIC_WORLD_ASPECTS_ENTITY_KINEMATIC_UTILS_H_

#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {
namespace entity_kinematic_world_details {

// Helper function to bridge the gap between the Entity-based World V2 and the
// IK solvers, which need a solver to know which IK algorithm to use. Robot id
// are provided in order of attachment. Each robot will be attached to each
// other with fixed joints.

absl::StatusOr<std::string> GetIKSolverKey(
    const entity_aspect_world_details::EntityWorld& world,
    const RobotCollectionsEntityId& robot_id);

}  // namespace entity_kinematic_world_details
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_ASPECTS_ENTITY_KINEMATIC_UTILS_H_
