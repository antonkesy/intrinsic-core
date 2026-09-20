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

#include "intrinsic/world/internal/world_acl.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/world_acl_spec.h"
#include "intrinsic/world/world_entity_acl_spec.h"

namespace intrinsic {

WorldACL::WorldACL(WorldACLSpec acls) {
  lock_ = std::make_shared<ACLLock>(ACLLock{std::move(acls)});
}

WorldACL::WorldACL(std::shared_ptr<ACLLock> lock) : lock_(std::move(lock)) {}

void WorldACL::SetAccessFor(EntityId entity_id, WorldEntityACLSpec acl) {
  lock_->acls.SetAccessFor(entity_id, acl);
}

void WorldACL::ClearAccessFor(const WorldHashSet<EntityId>& entity_ids) {
  lock_->acls.ClearAccessFor(entity_ids);
}

const WorldEntityACLSpec& WorldACL::GetSpecForEntityId(
    EntityId entity_id) const {
  return lock_->acls.GetSpecForEntityId(entity_id);
}

bool WorldACL::HasSpecForEntityId(EntityId entity_id) const {
  return lock_->acls.HasSpecForEntityId(entity_id);
}

WorldHashSet<EntityId> WorldACL::GetAllSpecifiedEntities() const {
  return lock_->acls.GetAllSpecifiedEntities();
}

absl::StatusOr<WorldACL> WorldACL::TryLock(const WorldACLSpec& narrow_spec) {
  // Check if we can delegate the lock based on the outstanding locks.
  for (const auto& entity_id : narrow_spec.GetAllSpecifiedEntities()) {
    const auto& entity_spec = narrow_spec.GetSpecForEntityId(entity_id);
    if (!HasSpecForEntityId(entity_id)) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "Unknown entity id: " << entity_id;
    }

    INTR_RETURN_IF_ERROR(GetSpecForEntityId(entity_id).CanNarrowTo(entity_spec))
        << "Entity id: " << entity_id;

    for (const auto& lock_ptr : delegated_locks_) {
      auto maybe_acl = lock_ptr.lock();
      if (maybe_acl == nullptr) {
        continue;
      }

      if (!maybe_acl->acls.HasSpecForEntityId(entity_id)) {
        continue;
      }

      const auto& existing_spec = maybe_acl->acls.GetSpecForEntityId(entity_id);
      if (!entity_spec.IsCompatible(existing_spec)) {
        return PermissionDeniedErrorBuilder()
               << "Incompatible spec when trying to lock ACLs for entity '"
               << entity_id << "' existing locked spec: " << existing_spec
               << " is not compatible with requested spec: " << entity_spec;
      }
    }
  }

  auto narrow_lock = std::make_shared<ACLLock>(ACLLock{narrow_spec});
  delegated_locks_.emplace_back(narrow_lock);
  return WorldACL(narrow_lock);
}

WorldACLSpec WorldACL::GetACLSpec() const { return lock_->acls; }

}  // namespace intrinsic
