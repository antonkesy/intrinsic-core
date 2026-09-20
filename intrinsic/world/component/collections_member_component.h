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

#ifndef INTRINSIC_WORLD_COMPONENT_COLLECTIONS_MEMBER_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_COLLECTIONS_MEMBER_COMPONENT_H_

#include <memory>

#include "absl/container/node_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collections_component.pb.h"
#include "intrinsic/world/proto/collections_member_component.pb.h"

namespace intrinsic {

// A Component class for data related to members of a collection.
class CollectionsMemberComponent {
 public:
  virtual ~CollectionsMemberComponent() = default;

  // Returns a new CollectionsMemberComponent instance.
  static std::unique_ptr<CollectionsMemberComponent> Create();

  // Returns a new CollectionsMemberComponent instance derived from the given
  // proto.
  static absl::StatusOr<std::unique_ptr<CollectionsMemberComponent>> FromProto(
      const intrinsic_proto::world::CollectionsMemberComponent& proto);

  // Returns a new CollectionsMemberComponent instance that is a copy of this
  // one.
  virtual std::unique_ptr<CollectionsMemberComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::CollectionsMemberComponent>
  ToProto() const = 0;

  // Returns whether this Entity is a member of collection identified by
  // (parent_collections_id, type).
  virtual bool IsMemberOfCollection(
      CollectionsEntityId parent_collections_id,
      intrinsic_proto::world::CollectionsComponent::CollectionType type)
      const = 0;

  // Returns whether this Entity is a member of a collection (of any type) under
  // the specified collections Entity.
  virtual bool IsMemberOfCollection(
      CollectionsEntityId parent_collections_id) const = 0;

  // Given the ID of a collections entity, returns the types of collections that
  // Entity belongs to. Returns an error if there are no such collections.
  virtual absl::StatusOr<const WorldHashSet<
      intrinsic_proto::world::CollectionsComponent::CollectionType>*>
  GetCollectionTypesByParentId(
      CollectionsEntityId parent_collections_id) const = 0;

  // Returns the map from parent collections Entity ID to collection types.
  virtual const absl::node_hash_map<
      CollectionsEntityId,
      WorldHashSet<
          intrinsic_proto::world::CollectionsComponent::CollectionType>>&
  GetParentCollectionsIdToTypesMap() const = 0;

  // Attempts to find exactly one parent collections entity among the parents of
  // the provided types. Returns an error if there are 0 or 2+ candidates.
  virtual absl::StatusOr<CollectionsEntityId> FindParentCollectionAmongTypes(
      const WorldHashSet<
          intrinsic_proto::world::CollectionsComponent::CollectionType>& types)
      const = 0;

  // Attempts to add the specified (parent collections ID, type) pair to the set
  // of memberships. Returns an error if the arguments are invalid or the
  // membership already exists.
  virtual absl::Status AddParentCollection(
      CollectionsEntityId parent_collections_id,
      intrinsic_proto::world::CollectionsComponent::CollectionType type) = 0;

  // Attempts to remove the specified (parent collections ID, type) pair from
  // the set of memberships. Returns an error if that membership does not exist.
  virtual absl::Status DeleteParentCollection(
      CollectionsEntityId parent_collections_id,
      intrinsic_proto::world::CollectionsComponent::CollectionType type) = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::CollectionsMemberComponent& proto) = 0;

  // Updates all ids within the collection using the mapping. Fails if the
  // mapping doesn't include all entities referenced.
  virtual absl::Status RekeyIds(
      const WorldHashMap<EntityId, EntityId>& id_mapping) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_COLLECTIONS_MEMBER_COMPONENT_H_
