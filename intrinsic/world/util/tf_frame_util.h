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

#ifndef INTRINSIC_WORLD_UTIL_TF_FRAME_UTIL_H_
#define INTRINSIC_WORLD_UTIL_TF_FRAME_UTIL_H_

#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto/transform_stamped.pb.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {

// Finds a world object and entity filter for a given TF frame ID in the given
// world. The entity filter identifies an underlying link or frame.
// Supported frame ID formats:
// - For an asset instance: 'asset_name/object_name/entity_name'
// - For a non asset object: 'object_name/entity_name'
// - 'root' for the root object
absl::StatusOr<
    std::pair<const object_world::WorldObject*, world::ObjectEntityFilter>>
GetWorldObjectAndEntityFilterFromFrameId(const object_world::ObjectWorld& world,
                                         absl::string_view frame_id);

absl::StatusOr<std::pair<object_world::WorldObject*, world::ObjectEntityFilter>>
GetWorldObjectAndEntityFilterFromFrameId(object_world::ObjectWorld& world,
                                         absl::string_view frame_id);

// Returns a map of entity ID to TF frame ID for all entities specified by the
// given ObjectEntityFilter. Requires `world_object` to have been retrieved
// with ObjectView::FULL.
//
// Filter behavior:
// - For the root object: returns a map containing {"root": "root"}.
// - IncludesAllEntities filter: returns mappings for all entities in the
//   object.
// - Only IncludesBaseEntity filter: returns a single entry mapping the object's
//   base entity ID to 'object_name'.
// - Explicit entity names or IDs filter: returns mappings for matching entities
//   only.
//
// Returned frame ID format for non-root objects:
// - If only IncludesBaseEntity is set: 'object_name' for base entity
// - In all other cases: 'object_name/entity_name' (returns an
// InvalidArgumentError
//   if entity_name is empty)
absl::StatusOr<absl::flat_hash_map<std::string, std::string>>
GetFrameIdByEntityIdFromWorldObjectAndEntityFilter(
    const world::WorldObject& world_object,
    const world::ObjectEntityFilter& entity_filter);

// Returns the TF frame ID for the base entity of the given WorldObject.
// Returned frame ID format:
// - 'root' for the root object
// - For non-root objects: 'object_name'
absl::StatusOr<std::string> GetFrameIdFromWorldObject(
    const world::WorldObject& world_object);

// Updates the world transform for the given TransformStamped message.
// If 'bypass_movable_check' is true, the check whether the target node is
// movable is bypassed.
absl::Status UpdateWorldTransform(
    object_world::ObjectWorld& world,
    const intrinsic_proto::TransformStamped& transform_stamped,
    bool bypass_movable_check = false);

// Gets the transform between two TF frame IDs.
absl::StatusOr<Pose3d> GetFrameTransform(const object_world::ObjectWorld& world,
                                         absl::string_view parent_frame_id,
                                         absl::string_view child_frame_id);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_TF_FRAME_UTIL_H_
