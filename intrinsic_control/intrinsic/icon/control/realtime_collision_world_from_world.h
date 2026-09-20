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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_COLLISION_WORLD_FROM_WORLD_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_COLLISION_WORLD_FROM_WORLD_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/geometry/storage/geometry_deserializer.h"
#include "intrinsic/icon/control/collision/collision_world.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/world.h"

namespace intrinsic::icon {

struct CollisionWorldWithLinkNames {
  collision::CollisionWorld collision_world;
  std::vector<std::string> link_names;
};

// Extracts collision data from `world` (geometry and exclusion pairs), and uses
// that data to create an instance of the realtime-safe CollisionWorld class.
// Links that are part of a robot *must* have primitive collision geometry.
//
// If `enable_environment_collision` is true, then this additionally extracts
// collision geometry for links that:
// * Are not part of a robot
// * Can collide with at least one robot link (i.e. have collision geometry and
//   do not have exclusions for all robot links)
//
// Note that this applies even to links that are attached to a robot! For
// example, if the world has a gripper attached to the robot's flange, then you
// must set `enable_environment_collision` to true if you want to consider
// collisions with the gripper.
//
// If `enable_environment_collision` is true, then the non-robot links must also
// have primitive collision geometry.
//
// Returns UnimplementedError if a robot link has non-primitive collision
// geometry.
// Returns AlreadyExistsError if multiple links in `world` have the same local
// name.
absl::StatusOr<CollisionWorldWithLinkNames> RealtimeCollisionWorldFromWorld(
    const World& world,
    bool enable_environment_collision) INTRINSIC_NON_REALTIME_ONLY;

absl::StatusOr<CollisionWorldWithLinkNames> RealtimeCollisionWorldFromWorld(
    const world::ObjectWorldClient& world_client,
    const GeometryDeserializer& geometry_deserializer,
    bool enable_environment_collision) INTRINSIC_NON_REALTIME_ONLY;

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_COLLISION_WORLD_FROM_WORLD_H_
