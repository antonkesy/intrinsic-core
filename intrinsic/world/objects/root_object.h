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

#ifndef INTRINSIC_WORLD_OBJECTS_ROOT_OBJECT_H_
#define INTRINSIC_WORLD_OBJECTS_ROOT_OBJECT_H_

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

// The root object in the object-based view onto a world (see ObjectWorld).
//
// Corresponds to the root entity in a World and is always present, even in an
// otherwise empty world.
class RootObject : public WorldObject {
 public:
  explicit RootObject(ObjectWorldData& data);

  absl::StatusOr<bool> NameIsGlobalAlias() const override { return true; }

  absl::Status SetName(const WorldObjectName& name,
                       bool name_is_global_alias) override;

  absl::Status ReparentTo(WorldObject& new_parent) override;
  absl::Status ReparentTo(WorldObject& new_parent,
                          const world::ObjectEntityFilter& filter) override;
  absl::Status ReparentToFinalEntityOf(WorldObject& new_parent) override;

  absl::StatusOr<Pose3d> GetParentTThis() const override;

  absl::Status Accept(WorldObjectVisitor& visitor) override {
    return visitor.Visit(*this);
  }

  absl::Status Accept(WorldObjectConstVisitor& visitor) const override {
    return visitor.Visit(*this);
  }

  absl::StatusOr<AttachmentEntityId>
  FinalEntityIfKinematicObjectOrElseRootEntity() const override;

  absl::StatusOr<WorldHashSet<AttachmentEntityId>> FinalEntities()
      const override;

 protected:
  absl::Status ToggleCollisionsWith(
      WorldObject& other, bool enable_collisions,
      const world::ObjectEntityFilter& this_entity_filter,
      const world::ObjectEntityFilter& other_entity_filter) override;

  absl::Status Delete(bool delete_child_objects) override;
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_ROOT_OBJECT_H_
