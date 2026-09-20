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

#ifndef INTRINSIC_WORLD_COMPONENT_COLLECTIONS_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_COLLECTIONS_COMPONENT_H_

#include <memory>
#include <set>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collections_component.pb.h"

namespace intrinsic {

// A Component class for collection Entities which hold typed lists of
// references to other Entities.
class CollectionsComponent {
 public:
  // Short hand constants for proto enums.
  static constexpr intrinsic_proto::world::CollectionsComponent::CollectionType
      kLinks =
          intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_LINKS;
  static constexpr intrinsic_proto::world::CollectionsComponent::CollectionType
      kJoints =
          intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_JOINTS;
  static constexpr intrinsic_proto::world::CollectionsComponent::CollectionType
      kCoordinateFrames = intrinsic_proto::world::CollectionsComponent::
          COLLECTION_TYPE_COORDINATE_FRAMES;
  static constexpr intrinsic_proto::world::CollectionsComponent::CollectionType
      kSensors =
          intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_SENSORS;
  static constexpr intrinsic_proto::world::CollectionsComponent::CollectionType
      kAttachmentFrames = intrinsic_proto::world::CollectionsComponent::
          COLLECTION_TYPE_ATTACHMENT_FRAMES;
  static constexpr intrinsic_proto::world::CollectionsComponent::CollectionType
      kProjectors = intrinsic_proto::world::CollectionsComponent::
          COLLECTION_TYPE_PROJECTORS;

  // Short hand for the types that used to be stored in RobotPartComponent.
  static const WorldHashSet<
      intrinsic_proto::world::CollectionsComponent::CollectionType>&
  RobotPartTypes();

  virtual ~CollectionsComponent() = default;

  // Returns a new CollectionsComponent instance.
  static std::unique_ptr<CollectionsComponent> Create();

  // Returns a new CollectionsComponent instance derived from the given proto.
  static absl::StatusOr<std::unique_ptr<CollectionsComponent>> FromProto(
      const intrinsic_proto::world::CollectionsComponent& proto);

  // Returns a new CollectionsComponent instance that is a copy of this one.
  virtual std::unique_ptr<CollectionsComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::CollectionsComponent> ToProto()
      const = 0;

  // Returns the members of a specified collection type.
  virtual const std::vector<CollectionsMemberEntityId>& GetCollectionMembers(
      intrinsic_proto::world::CollectionsComponent::CollectionType type)
      const = 0;

  virtual std::set<CollectionsMemberEntityId> GetAllCollectionMembers()
      const = 0;

  // Sets the members of a specified collection type. Caller is responsible for
  // ensuring any type-specific ordering invariants (see
  // intrinsic/world/proto/collections_component.proto) are
  // maintained. If members is empty, the entry for the given type is deleted
  // instead.
  virtual absl::Status SetCollectionMembers(
      intrinsic_proto::world::CollectionsComponent::CollectionType type,
      const std::vector<CollectionsMemberEntityId>& members) = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::CollectionsComponent& proto) = 0;

  // Updates all ids within the collection using the mapping. Fails if the
  // mapping doesn't include all entities referenced.
  virtual absl::Status RekeyIds(
      const WorldHashMap<EntityId, EntityId>& id_mapping) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_COLLECTIONS_COMPONENT_H_
