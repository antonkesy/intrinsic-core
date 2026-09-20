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

#include "intrinsic/world/world_acl_spec.h"

#include <ostream>
#include <ranges>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/acl_spec.pb.h"
#include "intrinsic/world/world_entity_acl_spec.h"

namespace intrinsic {

WorldACLSpec::WorldACLSpec(WorldEntityACLSpec default_acl)
    : default_acl_(default_acl) {}

WorldACLSpec& WorldACLSpec::AllowFullAccessFor(
    const WorldHashSet<EntityId>& entity_ids) {
  return AppendEntitySpecsWithOr(entity_ids, WorldEntityACLSpec::FullAccess());
}

WorldACLSpec& WorldACLSpec::DenyFullAccessFor(
    const WorldHashSet<EntityId>& entity_ids) {
  return AppendEntitySpecsWithAnd(entity_ids, WorldEntityACLSpec::NoAccess());
}

WorldACLSpec& WorldACLSpec::AllowReadFor(
    const WorldHashSet<EntityId>& entity_ids) {
  return AppendEntitySpecsWithOr(entity_ids, WorldEntityACLSpec(true, false));
}

WorldACLSpec& WorldACLSpec::AllowWriteFor(
    const WorldHashSet<EntityId>& entity_ids) {
  return AppendEntitySpecsWithOr(entity_ids, WorldEntityACLSpec(false, true));
}

WorldACLSpec& WorldACLSpec::DenyReadFor(
    const WorldHashSet<EntityId>& entity_ids) {
  return AppendEntitySpecsWithAnd(entity_ids, WorldEntityACLSpec(false, true));
}

WorldACLSpec& WorldACLSpec::DenyWriteFor(
    const WorldHashSet<EntityId>& entity_ids) {
  return AppendEntitySpecsWithAnd(entity_ids, WorldEntityACLSpec(true, false));
}

WorldACLSpec& WorldACLSpec::AppendEntitySpecsWithAnd(
    const WorldHashSet<EntityId>& entity_ids, WorldEntityACLSpec acl) {
  for (const auto& entity_id : entity_ids) {
    auto it = entity_acls_.find(entity_id);
    auto effective_acl = it != entity_acls_.end() ? it->second : default_acl_;
    entity_acls_.insert_or_assign(entity_id, acl.AndWith(effective_acl));
  }
  return *this;
}

WorldACLSpec& WorldACLSpec::AppendEntitySpecsWithOr(
    const WorldHashSet<EntityId>& entity_ids, WorldEntityACLSpec acl) {
  for (const auto& entity_id : entity_ids) {
    auto it = entity_acls_.find(entity_id);
    auto effective_acl = it != entity_acls_.end() ? it->second : default_acl_;
    entity_acls_.insert_or_assign(entity_id, acl.OrWith(effective_acl));
  }
  return *this;
}

WorldACLSpec& WorldACLSpec::SetAccessFor(EntityId entity_id,
                                         WorldEntityACLSpec acl) {
  entity_acls_.insert_or_assign(entity_id, acl);
  return *this;
}

WorldACLSpec& WorldACLSpec::ClearAccessFor(
    const WorldHashSet<EntityId>& entity_ids) {
  for (const auto& entity_id : entity_ids) {
    entity_acls_.erase(entity_id);
  }
  return *this;
}

const WorldEntityACLSpec& WorldACLSpec::GetSpecForEntityId(
    EntityId entity_id) const {
  if (!entity_acls_.contains(entity_id)) {
    return default_acl_;
  }

  return entity_acls_.at(entity_id);
}

bool WorldACLSpec::HasSpecForEntityId(EntityId entity_id) const {
  return entity_acls_.contains(entity_id);
}

WorldHashSet<EntityId> WorldACLSpec::GetAllSpecifiedEntities() const {
  const auto keys = std::views::keys(entity_acls_);
  return {keys.begin(), keys.end()};
}

absl::Status WorldACLSpec::IsCompatibleWith(const WorldACLSpec& other) const {
  for (const auto& [entity_id, spec] : entity_acls_) {
    if (!other.entity_acls_.contains(entity_id)) {
      // If the other list has no specs for this entity then we are compatible.
      continue;
    }

    const auto& other_spec = other.entity_acls_.at(entity_id);
    if (!spec.IsCompatible(other_spec)) {
      return PermissionDeniedErrorBuilder()
             << "Incompatible spec for entity '" << entity_id
             << "' spec: " << spec
             << " is not compatible with other spec: " << other_spec;
    }
  }

  return absl::OkStatus();
}

bool WorldACLSpec::operator==(const WorldACLSpec& other) const {
  return default_acl_ == other.default_acl_ &&
         entity_acls_ == other.entity_acls_;
}

bool WorldACLSpec::operator!=(const WorldACLSpec& other) const {
  return !(*this == other);
}

intrinsic_proto::world::WorldACLSpec WorldACLSpec::ToProto() const {
  intrinsic_proto::world::WorldACLSpec result;
  *result.mutable_default_acl() = default_acl_.ToProto();
  for (const auto& [entity_id, spec] : entity_acls_) {
    (*result.mutable_entities())[entity_id.value()] = spec.ToProto();
  }
  return result;
}

absl::StatusOr<WorldACLSpec> WorldACLSpec::FromProto(
    const intrinsic_proto::world::WorldACLSpec& proto_data) {
  INTR_ASSIGN_OR_RETURN(
      WorldEntityACLSpec default_acl,
      WorldEntityACLSpec::FromProto(proto_data.default_acl()));

  WorldACLSpec spec(default_acl);
  for (const auto& [entity_id, spec_proto] : proto_data.entities()) {
    INTR_ASSIGN_OR_RETURN(WorldEntityACLSpec acl,
                          WorldEntityACLSpec::FromProto(spec_proto));
    spec.SetAccessFor(EntityId(entity_id), acl);
  }

  return std::move(spec);
}

std::ostream& operator<<(std::ostream& os, const WorldACLSpec& acls) {
  return os << "default: " << acls.default_acl_ << "entity[ "
            << absl::StrJoin(acls.entity_acls_, ", ",
                             [](std::string* str,
                                std::pair<EntityId, WorldEntityACLSpec> entry) {
                               absl::StrAppend(
                                   str, "{id: ", entry.first.value(),
                                   ", acl: ", entry.second.ToString(), "}");
                             })
            << " ]";
}

}  // namespace intrinsic
