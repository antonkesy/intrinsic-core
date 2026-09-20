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

#include "intrinsic/world/component/collections_member_component.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <ranges>
#include <utility>
#include <vector>

#include "absl/container/node_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collections_component.pb.h"
#include "intrinsic/world/proto/collections_member_component.pb.h"

using ::intrinsic_proto::world::CollectionsComponent;

namespace intrinsic {
namespace {

class CollectionsMemberComponentImpl : public CollectionsMemberComponent {
 public:
  CollectionsMemberComponentImpl();
  explicit CollectionsMemberComponentImpl(
      const absl::node_hash_map<
          CollectionsEntityId,
          WorldHashSet<CollectionsComponent::CollectionType>>&
          parent_ids_to_types);
  absl::StatusOr<intrinsic_proto::world::CollectionsMemberComponent> ToProto()
      const override;
  std::unique_ptr<CollectionsMemberComponent> Clone() const override;
  bool IsMemberOfCollection(
      CollectionsEntityId parent_collections_id,
      intrinsic_proto::world::CollectionsComponent::CollectionType type)
      const override;
  bool IsMemberOfCollection(
      CollectionsEntityId parent_collections_id) const override;
  absl::StatusOr<const WorldHashSet<
      intrinsic_proto::world::CollectionsComponent::CollectionType>*>
  GetCollectionTypesByParentId(
      CollectionsEntityId parent_collections_id) const override;
  const absl::node_hash_map<CollectionsEntityId,
                            WorldHashSet<CollectionsComponent::CollectionType>>&
  GetParentCollectionsIdToTypesMap() const override {
    return parent_ids_to_types_;
  }
  absl::StatusOr<CollectionsEntityId> FindParentCollectionAmongTypes(
      const WorldHashSet<
          intrinsic_proto::world::CollectionsComponent::CollectionType>& types)
      const override;
  absl::Status AddParentCollection(
      CollectionsEntityId parent_collections_id,
      CollectionsComponent::CollectionType type) override;
  absl::Status DeleteParentCollection(
      CollectionsEntityId parent_collections_id,
      CollectionsComponent::CollectionType type) override;

  absl::Status UpdateFromProto(
      const intrinsic_proto::world::CollectionsMemberComponent& proto) override;

  absl::Status RekeyIds(
      const WorldHashMap<EntityId, EntityId>& id_mapping) override;

 private:
  absl::node_hash_map<CollectionsEntityId,
                      WorldHashSet<CollectionsComponent::CollectionType>>
      parent_ids_to_types_;
};

CollectionsMemberComponentImpl::CollectionsMemberComponentImpl() = default;

CollectionsMemberComponentImpl::CollectionsMemberComponentImpl(
    const absl::node_hash_map<
        CollectionsEntityId,
        WorldHashSet<CollectionsComponent::CollectionType>>&
        parent_ids_to_types)
    : parent_ids_to_types_(parent_ids_to_types) {}

absl::StatusOr<intrinsic_proto::world::CollectionsMemberComponent>
CollectionsMemberComponentImpl::ToProto() const {
  intrinsic_proto::world::CollectionsMemberComponent ret;

  // Output the parent IDs in ascending order.
  std::vector<uint32_t> parent_ids;
  parent_ids.reserve(parent_ids_to_types_.size());
  for (auto parent_id : std::views::keys(parent_ids_to_types_)) {
    parent_ids.push_back(parent_id.value());
  }
  std::sort(parent_ids.begin(), parent_ids.end());
  for (auto parent_id : parent_ids) {
    // Output the types in order.
    const auto& types = parent_ids_to_types_.at(CollectionsEntityId(parent_id));
    std::vector<CollectionsComponent::CollectionType> sorted_types(
        types.begin(), types.end());
    std::sort(sorted_types.begin(), sorted_types.end());
    auto& types_proto = (*ret.mutable_parent_uids_to_types())[parent_id];
    for (auto type : sorted_types) {
      types_proto.add_types(type);
    }
  }
  return ret;
}

std::unique_ptr<CollectionsMemberComponent>
CollectionsMemberComponentImpl::Clone() const {
  return std::make_unique<CollectionsMemberComponentImpl>(parent_ids_to_types_);
}

bool CollectionsMemberComponentImpl::IsMemberOfCollection(
    CollectionsEntityId parent_collections_id,
    intrinsic_proto::world::CollectionsComponent::CollectionType type) const {
  auto types_or = GetCollectionTypesByParentId(parent_collections_id);
  return types_or.ok() && types_or.value()->contains(type);
}

bool CollectionsMemberComponentImpl::IsMemberOfCollection(
    CollectionsEntityId parent_collections_id) const {
  return GetCollectionTypesByParentId(parent_collections_id).ok();
}

absl::StatusOr<const WorldHashSet<
    intrinsic_proto::world::CollectionsComponent::CollectionType>*>
CollectionsMemberComponentImpl::GetCollectionTypesByParentId(
    CollectionsEntityId parent_collections_id) const {
  auto iter = parent_ids_to_types_.find(parent_collections_id);
  if (iter == parent_ids_to_types_.end()) {
    return intrinsic::NotFoundErrorBuilder()
           << "not a member of any collection under ID "
           << parent_collections_id.value();
  }
  return &iter->second;
}

absl::StatusOr<CollectionsEntityId>
CollectionsMemberComponentImpl::FindParentCollectionAmongTypes(
    const WorldHashSet<
        intrinsic_proto::world::CollectionsComponent::CollectionType>& types)
    const {
  CollectionsEntityId ret(kInvalidEntityId);
  for (const auto& [parent_id, parent_types] : parent_ids_to_types_) {
    bool match = false;
    for (auto iter = types.begin(); !match && iter != types.end(); iter++) {
      match = parent_types.contains(*iter);
    }
    if (!match) {
      continue;
    }
    if (ret != kInvalidEntityId) {
      return absl::InvalidArgumentError(
          "found multiple parent collection candidates among given types");
    }
    ret = parent_id;
  }
  if (ret == kInvalidEntityId) {
    return absl::NotFoundError(
        "found no parent collection candidates among given types");
  }
  return ret;
}

absl::Status CollectionsMemberComponentImpl::AddParentCollection(
    CollectionsEntityId parent_collections_id,
    CollectionsComponent::CollectionType type) {
  if (parent_collections_id == kInvalidEntityId ||
      !CollectionsComponent::CollectionType_IsValid(type) ||
      type == CollectionsComponent::COLLECTION_TYPE_UNDEFINED) {
    return absl::InvalidArgumentError("invalid parent_collections_id or type");
  }
  auto& collection_types = parent_ids_to_types_[parent_collections_id];
  if (collection_types.contains(type)) {
    return AlreadyExistsErrorBuilder()
           << "cannot add redundant (parent collection ID, type) pair of ("
           << parent_collections_id.value() << ", "
           << CollectionsComponent::CollectionType_Name(type) << ")";
  }
  collection_types.insert(type);
  return absl::OkStatus();
}

absl::Status CollectionsMemberComponentImpl::DeleteParentCollection(
    CollectionsEntityId parent_collections_id,
    CollectionsComponent::CollectionType type) {
  auto iter = parent_ids_to_types_.find(parent_collections_id);
  if (iter == parent_ids_to_types_.end() || iter->second.erase(type) != 1) {
    return intrinsic::NotFoundErrorBuilder()
           << "not a member of "
           << CollectionsComponent::CollectionType_Name(type)
           << " collection under ID " << parent_collections_id.value();
  }
  // If we just removed the final type for this parent, remove it.
  if (iter->second.empty()) {
    parent_ids_to_types_.erase(iter);
  }
  return absl::OkStatus();
}

absl::Status CollectionsMemberComponentImpl::UpdateFromProto(
    const intrinsic_proto::world::CollectionsMemberComponent& proto) {
  absl::node_hash_map<CollectionsEntityId,
                      WorldHashSet<CollectionsComponent::CollectionType>>
      updated_parent_ids_to_types;

  for (const auto& [raw_parent_id, raw_types] : proto.parent_uids_to_types()) {
    CollectionsEntityId parent_id(raw_parent_id);

    for (auto raw_type : raw_types.types()) {
      const auto type = CollectionsComponent::CollectionType(raw_type);

      if (parent_id == kInvalidEntityId ||
          !CollectionsComponent::CollectionType_IsValid(type) ||
          type == CollectionsComponent::COLLECTION_TYPE_UNDEFINED) {
        return absl::InvalidArgumentError(
            "invalid parent_collections_id or type");
      }
      auto& collection_types = updated_parent_ids_to_types[parent_id];
      if (collection_types.contains(type)) {
        return AlreadyExistsErrorBuilder()
               << "cannot add redundant (parent collection ID, type) pair of ("
               << raw_parent_id << ", "
               << CollectionsComponent::CollectionType_Name(type) << ")";
      }
      collection_types.emplace(type);
    }
  }

  // Once we know that both maps have parsed correctly we then update our fields
  parent_ids_to_types_ = updated_parent_ids_to_types;
  return absl::OkStatus();
}

absl::Status CollectionsMemberComponentImpl::RekeyIds(
    const WorldHashMap<EntityId, EntityId>& id_mapping) {
  absl::node_hash_map<CollectionsEntityId,
                      WorldHashSet<CollectionsComponent::CollectionType>>
      new_parent_ids_to_types;
  for (const auto& [parent_id, types] : parent_ids_to_types_) {
    if (id_mapping.contains(parent_id)) {
      new_parent_ids_to_types[CollectionsEntityId(id_mapping.at(parent_id))] =
          types;
    } else {
      return absl::InvalidArgumentError(absl::StrCat(
          "Entity collections member data references an entity with id ",
          parent_id.value(), " that was not provided."));
    }
  }

  parent_ids_to_types_ = std::move(new_parent_ids_to_types);
  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<CollectionsMemberComponent>
CollectionsMemberComponent::Create() {
  return std::make_unique<CollectionsMemberComponentImpl>();
}

absl::StatusOr<std::unique_ptr<CollectionsMemberComponent>>
CollectionsMemberComponent::FromProto(
    const intrinsic_proto::world::CollectionsMemberComponent& proto) {
  auto ret = std::make_unique<CollectionsMemberComponentImpl>();

  for (const auto& [parent_id, types] : proto.parent_uids_to_types()) {
    for (auto type : types.types()) {
      INTR_RETURN_IF_ERROR(
          ret->AddParentCollection(CollectionsEntityId(parent_id),
                                   CollectionsComponent::CollectionType(type)));
    }
  }
  return ret;
}

}  // namespace intrinsic
