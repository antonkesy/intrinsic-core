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

#include "intrinsic/world/objects/root_object.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world_data.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {
namespace object_world {

RootObject::RootObject(ObjectWorldData& data)
    : WorldObject(RootObjectId(), RootObjectName(), {kRootEntityId}, data) {}

absl::Status RootObject::SetName(const WorldObjectName& name,
                                 bool name_is_global_alias) {
  return absl::InvalidArgumentError(
      "Cannot change the name of the root object.");
}

absl::Status RootObject::ReparentTo(WorldObject& new_parent) {
  return absl::InvalidArgumentError("Cannot reparent the root object.");
}

absl::Status RootObject::ReparentTo(WorldObject& new_parent,
                                    const world::ObjectEntityFilter& filter) {
  return absl::InvalidArgumentError("Cannot reparent the root object.");
}

absl::Status RootObject::ReparentToFinalEntityOf(WorldObject& new_parent) {
  return ReparentTo(new_parent);
}

absl::Status RootObject::ToggleCollisionsWith(
    WorldObject& other, bool enable_collisions,
    const world::ObjectEntityFilter& this_entity_filter,
    const world::ObjectEntityFilter& other_entity_filter) {
  return absl::InvalidArgumentError(
      "Cannot enable/disable collisions with the root object.");
}

absl::Status RootObject::Delete(bool delete_child_objects) {
  return absl::InvalidArgumentError("Cannot delete the root object.");
}

absl::StatusOr<Pose3d> RootObject::GetParentTThis() const {
  return absl::InternalError(
      "GetParentTThis() cannot be called on the root object.");
}

absl::StatusOr<AttachmentEntityId>
RootObject::FinalEntityIfKinematicObjectOrElseRootEntity() const {
  return kRootEntityId;
}

absl::StatusOr<WorldHashSet<AttachmentEntityId>> RootObject::FinalEntities()
    const {
  return WorldHashSet<AttachmentEntityId>{kRootEntityId};
}

}  // namespace object_world
}  // namespace intrinsic
