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

#ifndef INTRINSIC_WORLD_COMPONENT_COLLISION_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_COLLISION_COMPONENT_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collision_component.pb.h"

namespace intrinsic {

// A component to hold collision related information.
class CollisionComponent {
 public:
  virtual ~CollisionComponent() = default;

  // Returns a new CollisionComponent instance.
  static std::unique_ptr<CollisionComponent> Create();

  // Returns a new CollisionComponent instance derived from the given proto.
  static absl::StatusOr<std::unique_ptr<CollisionComponent>> FromProto(
      const intrinsic_proto::world::CollisionComponent& proto);

  // Returns a new CollisionComponent instance that is a copy of this one.
  virtual std::unique_ptr<CollisionComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::CollisionComponent> ToProto()
      const = 0;

  // Adds the given id to the exclusion list for this component. Duplicates will
  // be ignored.
  virtual void AddExclusionId(PhysicalEntityId other_handle) = 0;

  // Removes the exclusion id from the exclusion list if it was there.
  virtual void RemoveExclusionId(PhysicalEntityId other_handle) = 0;

  // Returns true if the entity is collidable, but should not produce a physical
  // response.
  virtual bool HasCollisionResponse() const = 0;

  // Adds or removes the property that the associated entity will generate a
  // physical response when collided with.
  virtual void SetCollisionResponse(bool flag) = 0;

  // Removes all exclusions.
  virtual void ClearExclusions() = 0;

  // Returns the set of exclusion ids for this entity
  virtual const WorldHashSet<PhysicalEntityId>& GetExclusions() const = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::CollisionComponent& proto) = 0;

  // Updates all ids within the collection using the mapping. Fails if the
  // mapping doesn't include all entities referenced.
  virtual absl::Status RekeyIds(
      const WorldHashMap<EntityId, EntityId>& id_mapping,
      bool drop_unknown_ids) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_COLLISION_COMPONENT_H_
