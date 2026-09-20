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

#include "intrinsic/world/component/collision_component.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collision_component.pb.h"

namespace intrinsic {
namespace {

class CollisionComponentImpl : public CollisionComponent {
 public:
  CollisionComponentImpl() = default;
  explicit CollisionComponentImpl(WorldHashSet<PhysicalEntityId>&& exclusions)
      : exclusions_(std::move(exclusions)) {}

  absl::StatusOr<intrinsic_proto::world::CollisionComponent> ToProto()
      const override;
  std::unique_ptr<CollisionComponent> Clone() const override;

  void AddExclusionId(PhysicalEntityId other_handle) override;
  void RemoveExclusionId(PhysicalEntityId other_handle) override;
  bool HasCollisionResponse() const override;
  void SetCollisionResponse(bool flag) override;
  void ClearExclusions() override;
  const WorldHashSet<PhysicalEntityId>& GetExclusions() const override;

  absl::Status UpdateFromProto(
      const intrinsic_proto::world::CollisionComponent& proto) override;

  absl::Status RekeyIds(const WorldHashMap<EntityId, EntityId>& id_mapping,
                        bool drop_unknown_ids) override;

 private:
  WorldHashSet<PhysicalEntityId> exclusions_;
  bool has_collision_response_ = true;
};

absl::StatusOr<intrinsic_proto::world::CollisionComponent>
CollisionComponentImpl::ToProto() const {
  intrinsic_proto::world::CollisionComponent result;
  std::vector<PhysicalEntityId> sorted_exclusions(exclusions_.begin(),
                                                  exclusions_.end());
  std::sort(sorted_exclusions.begin(), sorted_exclusions.end());
  for (const auto& exclusion_handle : sorted_exclusions) {
    result.add_exclusions()->set_uid(exclusion_handle.id.value());
  }

  result.set_has_no_collision_response(!has_collision_response_);
  return result;
}

void CollisionComponentImpl::AddExclusionId(PhysicalEntityId other_handle) {
  exclusions_.emplace(other_handle);
}

void CollisionComponentImpl::RemoveExclusionId(PhysicalEntityId other_handle) {
  exclusions_.erase(other_handle);
}

bool CollisionComponentImpl::HasCollisionResponse() const {
  return has_collision_response_;
}

void CollisionComponentImpl::SetCollisionResponse(bool flag) {
  has_collision_response_ = flag;
}

void CollisionComponentImpl::ClearExclusions() { exclusions_.clear(); }

const WorldHashSet<PhysicalEntityId>& CollisionComponentImpl::GetExclusions()
    const {
  return exclusions_;
}

std::unique_ptr<CollisionComponent> CollisionComponentImpl::Clone() const {
  auto exclusions_copy = exclusions_;
  auto component_copy =
      std::make_unique<CollisionComponentImpl>(std::move(exclusions_copy));
  component_copy->has_collision_response_ = has_collision_response_;
  return component_copy;
}

absl::StatusOr<WorldHashSet<PhysicalEntityId>> ParseCollisionExclusions(
    const intrinsic_proto::world::CollisionComponent& proto) {
  WorldHashSet<PhysicalEntityId> exclusions;
  for (const auto& exclusion : proto.exclusions()) {
    if (exclusion.type_oneof_case() !=
        intrinsic_proto::world::CollisionComponent_Exclusion::kUid) {
      return DataLossErrorBuilder()
             << "Unknown exclusion type: " << exclusion.type_oneof_case();
    }

    // TODO(stoyang): We should validate these entities
    exclusions.emplace(exclusion.uid());
  }

  return std::move(exclusions);
}

absl::Status CollisionComponentImpl::UpdateFromProto(
    const intrinsic_proto::world::CollisionComponent& proto) {
  INTR_ASSIGN_OR_RETURN(exclusions_, ParseCollisionExclusions(proto));
  has_collision_response_ = !proto.has_no_collision_response();
  return absl::OkStatus();
}

absl::Status CollisionComponentImpl::RekeyIds(
    const WorldHashMap<EntityId, EntityId>& id_mapping, bool drop_unknown_ids) {
  WorldHashSet<PhysicalEntityId> new_exclusions;
  for (const auto& exclusion : exclusions_) {
    if (id_mapping.contains(exclusion)) {
      new_exclusions.insert(PhysicalEntityId(id_mapping.at(exclusion)));
    } else if (!drop_unknown_ids) {
      return absl::InvalidArgumentError(
          absl::StrCat("Entity collections data references an entity with id ",
                       exclusion.value(), " that was not provided."));
    }
  }
  exclusions_ = std::move(new_exclusions);
  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<CollisionComponent> CollisionComponent::Create() {
  return std::make_unique<CollisionComponentImpl>();
}

absl::StatusOr<std::unique_ptr<CollisionComponent>>
CollisionComponent::FromProto(
    const intrinsic_proto::world::CollisionComponent& proto) {
  INTR_ASSIGN_OR_RETURN(auto exclusions, ParseCollisionExclusions(proto));

  auto component =
      std::make_unique<CollisionComponentImpl>(std::move(exclusions));
  component->SetCollisionResponse(!proto.has_no_collision_response());
  return component;
}

}  // namespace intrinsic
