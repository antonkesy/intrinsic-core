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

#ifndef INTRINSIC_ICON_UTILS_WORLD_UTILS_H_
#define INTRINSIC_ICON_UTILS_WORLD_UTILS_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/world.h"

namespace intrinsic::icon {

struct SkeletonAndIdMap {
  std::unique_ptr<kinematics::Skeleton> skeleton;
  absl::flat_hash_map<std::string, kinematics::ElementId>
      world_id_to_skeleton_id;
  std::vector<world::WorldObject> kinematic_objects_in_skeleton;
};

// Attempts to get a unique DofKinematicView from an object in `world`,
// optionally filtered by `resource_instance_name`. Unique here means that it is
// the only World object that provides a DofKinematicView.
//
// When the caller runs as a resource instance, `resource_instance_name` is
// available via the RuntimeContext proto
// (intrinsic/resources/proto/runtime_context.proto).
//
// Returns
//   * NotFoundError if there are no RobotCollections in `world`
//   * NotFoundError if `resource_instance_name` is provided, and none of
//     the RobotCollection entities have a PprComponent with
//     `resource_instance_name`
//    * FailedPreconditionError if there is more than one RobotCollection
//      (after optionally filtering for `resource_instance_name`)
absl::StatusOr<std::unique_ptr<const DofKinematicView>>
GetUniqueDofKinematicView(const World& world,
                          std::optional<std::string> resource_instance_name);

// Attempts to get a DofKinematicView for an entity with `local_name` from
// `world`, optionally filtered by `resource_instance_name`.
//
// When the caller runs as a resource instance `resource_instance_name` is
// available via the RuntimeContext proto
// (intrinsic/resources/proto/runtime_context.proto).
//
// Returns
//   * NotFoundError if there are no RobotCollections with `local_name` in
//     `world`
//   * NotFoundError if `resource_instance_name` is provided, and none of
//     the RobotCollection entities with `local_name` have a PprComponent with
//     `resource_instance_name`
//    * FailedPreconditionError if there is more than one RobotCollection with
//      `local_name` (after optionally filtering for `resource_instance_name`)
absl::StatusOr<std::unique_ptr<const DofKinematicView>>
GetDofKinematicViewByLocalName(
    const World& world, absl::string_view local_name,
    std::optional<std::string> resource_instance_name);

// Attempts to get a DofKinematicView for an entity with `alias` from
// `world`.
//
// This does not take a `resource_instance_name` like the others above, because
// an alias is already unique.
//
// Returns NotFoundError if there is no RobotCollection with `alias` in `world`.
absl::StatusOr<std::unique_ptr<const DofKinematicView>>
GetDofKinematicViewByAlias(const World& world, absl::string_view alias);

absl::StatusOr<SkeletonAndIdMap> GetSkeletonForObject(
    const world::ObjectWorldClient& world_client, WorldObjectName object_name,
    bool include_children = false);

absl::StatusOr<SkeletonAndIdMap> GetWholeWorldTreeSkeleton(
    const world::ObjectWorldClient& world_client);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_UTILS_WORLD_UTILS_H_
