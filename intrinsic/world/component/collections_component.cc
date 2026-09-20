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

#include "intrinsic/world/component/collections_component.h"

#include <memory>
#include <set>
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

namespace intrinsic {

const WorldHashSet<
    intrinsic_proto::world::CollectionsComponent::CollectionType>&
CollectionsComponent::RobotPartTypes() {
  static const auto* set = new WorldHashSet<
      intrinsic_proto::world::CollectionsComponent::CollectionType>({
      CollectionsComponent::kLinks,
      CollectionsComponent::kJoints,
      CollectionsComponent::kCoordinateFrames,
      // TODO(stoyang): This should include sensors but we get some build breaks
      // CollectionsComponent::kSensors,
      CollectionsComponent::kAttachmentFrames,
  });
  return *set;
}

namespace {

class CollectionsComponentImpl : public CollectionsComponent {
 public:
  CollectionsComponentImpl() = default;
  explicit CollectionsComponentImpl(
      const absl::node_hash_map<
          intrinsic_proto::world::CollectionsComponent::CollectionType,
          std::vector<CollectionsMemberEntityId>>& collections);
  absl::StatusOr<intrinsic_proto::world::CollectionsComponent> ToProto()
      const override;
  std::unique_ptr<CollectionsComponent> Clone() const override;

  const std::vector<CollectionsMemberEntityId>& GetCollectionMembers(
      intrinsic_proto::world::CollectionsComponent::CollectionType type)
      const override;
  std::set<CollectionsMemberEntityId> GetAllCollectionMembers() const override;

  absl::Status SetCollectionMembers(
      intrinsic_proto::world::CollectionsComponent::CollectionType type,
      const std::vector<CollectionsMemberEntityId>& members) override;

  absl::Status UpdateFromProto(
      const intrinsic_proto::world::CollectionsComponent& proto) override;

  absl::Status RekeyIds(
      const WorldHashMap<EntityId, EntityId>& id_mapping) override;

 private:
  absl::node_hash_map<
      intrinsic_proto::world::CollectionsComponent::CollectionType,
      std::vector<CollectionsMemberEntityId>>
      collections_;
  // Empty vector return value for GetCollectionMembers.
  const std::vector<CollectionsMemberEntityId> empty_collection_;
};

CollectionsComponentImpl::CollectionsComponentImpl(
    const absl::node_hash_map<
        intrinsic_proto::world::CollectionsComponent::CollectionType,
        std::vector<CollectionsMemberEntityId>>& collections)
    : collections_(collections) {}

absl::StatusOr<intrinsic_proto::world::CollectionsComponent>
CollectionsComponentImpl::ToProto() const {
  intrinsic_proto::world::CollectionsComponent proto;
  for (const auto& [type, members] : collections_) {
    auto& members_proto = (*proto.mutable_collections())[type];
    for (CollectionsMemberEntityId member : members) {
      members_proto.add_uid(member.value());
    }
  }
  return proto;
}

std::unique_ptr<CollectionsComponent> CollectionsComponentImpl::Clone() const {
  auto ret = std::make_unique<CollectionsComponentImpl>();
  ret->collections_ = collections_;
  return ret;
}

const std::vector<CollectionsMemberEntityId>&
CollectionsComponentImpl::GetCollectionMembers(
    intrinsic_proto::world::CollectionsComponent::CollectionType type) const {
  auto iter = collections_.find(type);
  if (iter == collections_.end()) {
    return empty_collection_;
  }
  return iter->second;
}

std::set<CollectionsMemberEntityId>
CollectionsComponentImpl::GetAllCollectionMembers() const {
  std::set<CollectionsMemberEntityId> entities;

  for (const auto& [type, members] : collections_) {
    entities.insert(members.begin(), members.end());
  }

  return entities;
}

absl::Status CollectionsComponentImpl::SetCollectionMembers(
    intrinsic_proto::world::CollectionsComponent::CollectionType type,
    const std::vector<CollectionsMemberEntityId>& members) {
  if (!intrinsic_proto::world::CollectionsComponent::CollectionType_IsValid(
          type) ||
      type == intrinsic_proto::world::CollectionsComponent::
                  COLLECTION_TYPE_UNDEFINED) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "unknown or invalid collection type: "
           << intrinsic_proto::world::CollectionsComponent::CollectionType_Name(
                  type);
  }
  if (members.empty()) {
    collections_.erase(type);
  } else {
    collections_[type] = members;
  }
  return absl::OkStatus();
}

absl::StatusOr<absl::node_hash_map<
    intrinsic_proto::world::CollectionsComponent::CollectionType,
    std::vector<CollectionsMemberEntityId>>>
ParseCollectionsMap(const intrinsic_proto::world::CollectionsComponent& proto) {
  absl::node_hash_map<
      intrinsic_proto::world::CollectionsComponent::CollectionType,
      std::vector<CollectionsMemberEntityId>>
      collections;
  for (const auto& [type, members] : proto.collections()) {
    if (!intrinsic_proto::world::CollectionsComponent::CollectionType_IsValid(
            type) ||
        type == intrinsic_proto::world::CollectionsComponent::
                    COLLECTION_TYPE_UNDEFINED) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "unknown or invalid CollectionsComponent::CollectionType "
             << type;
    }
    auto& collection = collections
        [intrinsic_proto::world::CollectionsComponent::CollectionType(type)];
    for (const auto& id : members.uid()) {
      collection.emplace_back(id);
    }
  }

  return std::move(collections);
}

absl::Status CollectionsComponentImpl::UpdateFromProto(
    const intrinsic_proto::world::CollectionsComponent& proto) {
  INTR_ASSIGN_OR_RETURN(collections_, ParseCollectionsMap(proto));
  return absl::OkStatus();
}

absl::Status CollectionsComponentImpl::RekeyIds(
    const WorldHashMap<EntityId, EntityId>& id_mapping) {
  absl::node_hash_map<
      intrinsic_proto::world::CollectionsComponent::CollectionType,
      std::vector<CollectionsMemberEntityId>>
      new_collections;
  for (const auto& [type, members] : collections_) {
    std::vector<CollectionsMemberEntityId> new_members;
    new_members.reserve(members.size());

    for (const auto& member : members) {
      if (id_mapping.contains(member)) {
        new_members.emplace_back(id_mapping.at(member));
      } else {
        return absl::InvalidArgumentError(absl::StrCat(
            "Entity collections data references an entity with id ",
            member.value(), " that was not provided."));
      }
    }

    new_collections[type] = std::move(new_members);
  }

  collections_ = std::move(new_collections);
  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<CollectionsComponent> CollectionsComponent::Create() {
  return std::make_unique<CollectionsComponentImpl>();
}

absl::StatusOr<std::unique_ptr<CollectionsComponent>>
CollectionsComponent::FromProto(
    const intrinsic_proto::world::CollectionsComponent& proto) {
  INTR_ASSIGN_OR_RETURN(auto collections, ParseCollectionsMap(proto));
  return std::make_unique<CollectionsComponentImpl>(collections);
}

}  // namespace intrinsic
