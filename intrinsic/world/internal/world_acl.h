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

#ifndef INTRINSIC_WORLD_INTERNAL_WORLD_ACL_H_
#define INTRINSIC_WORLD_INTERNAL_WORLD_ACL_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/world_acl_spec.h"
#include "intrinsic/world/world_entity_acl_spec.h"

namespace intrinsic {

// WorldACL wraps a WorldACLSpec and allows delegation of those permissions to
// other WorldACL instances, ensuring those delegations do not conflict with
// each other. It also manages the lifetime of those delegations to allow for
// re-delegation once the delegated instance has released its permissions back
// to this instance.
class WorldACL {
 public:
  explicit WorldACL(WorldACLSpec acls);

  // Disallow copy and assignment operators as they are not compatible with our
  // lifetime and delegation management. However we allow move operations.
  WorldACL(const WorldACL& other) = delete;
  WorldACL& operator=(const WorldACL& other) = delete;
  WorldACL(WorldACL&& other) = default;
  WorldACL& operator=(WorldACL&& other) = default;

  // Sets the WorldEntityACLSpec for a given entity id. It will replace any
  // existing access permissions previously set for the entity id. Returns
  // itself to allow chaining calls.
  void SetAccessFor(EntityId entity_id, WorldEntityACLSpec acl);

  // Removes any previously set specs for the given entity.
  void ClearAccessFor(const WorldHashSet<EntityId>& entity_ids);

  // Returns the WorldEntityACLSpec for the given entity, or the default acl if
  // there wasn't one previous set specifically for the given entity id. Returns
  // itself to allow chaining calls.
  const WorldEntityACLSpec& GetSpecForEntityId(EntityId entity_id) const;

  // Returns true if the given entity has a non default spec.
  bool HasSpecForEntityId(EntityId entity_id) const;

  // Returns the set of entities that have specified ACLs.
  WorldHashSet<EntityId> GetAllSpecifiedEntities() const;

  // Attempts to lock a subset of the permissions available in this instance. If
  // the given spec can be delegated the resulting WorldACL will lock the given
  // spec for its lifetime. Once the returned object is deallocated, the
  // permissions will be unlocked and a new delegation can happen through this
  // instance.
  absl::StatusOr<WorldACL> TryLock(const WorldACLSpec& narrow_spec);

  // Returns the current set of ACLs observed by this acl instance.
  WorldACLSpec GetACLSpec() const;

 private:
  // Internal structure that can have its lifetime managed for ensuring that our
  // locks are released when the object goes away.
  struct ACLLock {
    WorldACLSpec acls;
  };

  // Creates a new instance with the given acl lock.
  explicit WorldACL(std::shared_ptr<ACLLock> lock);

  std::shared_ptr<ACLLock> lock_;
  std::vector<std::weak_ptr<ACLLock>> delegated_locks_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_INTERNAL_WORLD_ACL_H_
