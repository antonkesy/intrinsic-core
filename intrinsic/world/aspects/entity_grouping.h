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

#ifndef INTRINSIC_WORLD_ASPECTS_ENTITY_GROUPING_H_
#define INTRINSIC_WORLD_ASPECTS_ENTITY_GROUPING_H_

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/grouping.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {
namespace entity_grouping_details {

class ABSL_DEPRECATED(
    "Grouping aspect is deprecated; use the world instance directly instead.")
    EntityGrouping : public Grouping {
 public:
  // Initialize the Aspect from the given world interface.
  explicit EntityGrouping(entity_aspect_world_details::EntityWorld* world);

  // Copy and move are disabled to match other Aspects.
  EntityGrouping(EntityGrouping&& other) = delete;
  EntityGrouping(const EntityGrouping& other) = delete;
  EntityGrouping operator=(EntityGrouping&& other) = delete;
  EntityGrouping operator=(const EntityGrouping& other) = delete;

  void UpdateEntityWorld(entity_aspect_world_details::EntityWorld* world);

  // Adds a new group with the given name and object ids, the name must be
  // unique within this world.
  absl::Status AddGroup(
      GroupId group_id,
      const WorldHashSet<PhysicalEntityId>& object_ids) override;

  // Removes the given group from the world.
  void RemoveGroup(GroupId group_id) override;

  // Returns true if the given group exists in the world.
  bool CheckGroupExists(GroupId group_id) const override;

  // Returns the set of group names contained in the world.
  WorldHashSet<GroupId> GetGroupIds() const override;

  // Returns the set of objects associated with this group name. The group must
  // exist.
  WorldHashSet<PhysicalEntityId> GetObjectIds(GroupId group_id) const override;

  // Returns the physical objects associated with the given group and label, if
  // one exists, if there are multiple objects with that label it will fail. The
  // group must already exist.
  WorldHashSet<PhysicalEntityId> GetPhysicalObjectsForGroup(
      GroupId group_id, const LabelId& label_id) const override;

  // Returns the unique physical object id that is associated with both the
  // group id and label id. It fails if such object does not exist or there are
  // more than one object match.
  PhysicalEntityId GetSingleObject(GroupId group_id,
                                   const LabelId& label_id) const override;

 private:
  // The reference to the world that this Aspect belongs to.
  entity_aspect_world_details::EntityWorld* world_ = nullptr;
};

}  // namespace entity_grouping_details
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_ASPECTS_ENTITY_GROUPING_H_
