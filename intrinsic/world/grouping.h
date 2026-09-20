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

#ifndef INTRINSIC_WORLD_GROUPING_H_
#define INTRINSIC_WORLD_GROUPING_H_

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "intrinsic/util/string_type.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {

INTRINSIC_DEFINE_STRING_TYPE_AS(GroupId,
                                intrinsic::SharedPtrStringRepresentation);

// A grouping is a collection of physical objects that have been named. We can
// then search within a group for a specific label or get the list of objects in
// the group.
class ABSL_DEPRECATED(
    "Grouping aspect is deprecated; use the world instance directly instead.")
    Grouping {
 public:
  virtual ~Grouping() = default;

  // Adds a new group with the given name and object ids, the name must be
  // unique within this world.
  ABSL_DEPRECATED(
      "Grouping aspect is deprecated; use the world instance directly instead.")
  virtual absl::Status AddGroup(
      GroupId group_id, const WorldHashSet<PhysicalEntityId>& object_ids) = 0;

  // Removes the given group from the world.
  ABSL_DEPRECATED(
      "Grouping aspect is deprecated; use the world instance directly instead.")
  virtual void RemoveGroup(GroupId group_id) = 0;

  // Returns true if the given group exists in the world.
  ABSL_DEPRECATED(
      "Grouping aspect is deprecated; use the world instance directly instead.")
  virtual bool CheckGroupExists(GroupId group_id) const = 0;

  // Returns the set of group names contained in the world.
  ABSL_DEPRECATED(
      "Grouping aspect is deprecated; use the world instance directly instead.")
  virtual WorldHashSet<GroupId> GetGroupIds() const = 0;

  // Returns the set of objects associated with this group name. The group must
  // exist.
  ABSL_DEPRECATED(
      "Grouping aspect is deprecated; use the world instance directly instead.")
  virtual WorldHashSet<PhysicalEntityId> GetObjectIds(
      GroupId group_id) const = 0;

  // Returns the physical objects associated with the given group and label, if
  // one exists, if there are multiple objects with that label it will fail. The
  // group must already exist.
  ABSL_DEPRECATED(
      "Grouping aspect is deprecated; use the world instance directly instead.")
  virtual WorldHashSet<PhysicalEntityId> GetPhysicalObjectsForGroup(
      GroupId group_id, const LabelId& label_id) const = 0;

  // Returns the unique physical object id that is associated with both the
  // group id and label id. It fails if such object does not exist or there are
  // more than one object match.
  ABSL_DEPRECATED(
      "Grouping aspect is deprecated; use the world instance directly instead.")
  virtual PhysicalEntityId GetSingleObject(GroupId group_id,
                                           const LabelId& label_id) const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_GROUPING_H_
