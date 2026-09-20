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

#ifndef INTRINSIC_WORLD_WORLD_ACL_SPEC_H_
#define INTRINSIC_WORLD_WORLD_ACL_SPEC_H_

#include <functional>
#include <memory>
#include <ostream>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/acl_spec.pb.h"
#include "intrinsic/world/world_entity_acl_spec.h"

namespace intrinsic {

// WorldACLSpec describes what access permissions are allowed for entities in a
// world. The spec is not tied to a specific world, instead it relies on a map
// of EntityIds and the access permissions that entity should allow.
class WorldACLSpec {
 public:
  // Creates a new starting spec with a default ACL to use for entities not
  // specifically assigned a special ACL spec. The default will be used as a
  // starting point if further modifications are made to any entities.
  explicit WorldACLSpec(
      WorldEntityACLSpec default_acl = WorldEntityACLSpec::NoAccess());

  // Adds all access permissions for the given entities, and overrides any
  // previously set permissions for them. Returns itself to allow chaining calls
  WorldACLSpec& AllowFullAccessFor(const WorldHashSet<EntityId>& entity_ids);

  // Removes all access permissions for the given entities, and overrides any
  // previously set permissions for them. Returns itself to allow chaining calls
  WorldACLSpec& DenyFullAccessFor(const WorldHashSet<EntityId>& entity_ids);

  // Adds the read permission to the given entities, if any of the entities
  // already had read permissions, nothing will change for them. If any of the
  // entities do not have a previously set acl the default will be used as a
  // starting point and read access will be added. Returns itself to allow
  // chaining calls.
  WorldACLSpec& AllowReadFor(const WorldHashSet<EntityId>& entity_ids);

  // Adds the write permission to the given entities, if any of the entities
  // already had write permissions, nothing will change for them. If any of the
  // entities do not have a previously set acl the default will be used as a
  // starting point and write access will be added. Returns itself to allow
  // chaining calls.
  WorldACLSpec& AllowWriteFor(const WorldHashSet<EntityId>& entity_ids);

  // Removes the read permission from the given entities, if any of the
  // entities did not have read permissions, nothing will change for them. If
  // any of the entities do not have a previously set acl the default will be
  // used as a starting point and read access will be removed. Returns itself to
  // allow chaining calls.
  WorldACLSpec& DenyReadFor(const WorldHashSet<EntityId>& entity_ids);

  // Removes the write permission from the given entities, if any of the
  // entities did not have write permissions, nothing will change for them. If
  // any of the entities do not have a previously set acl the default will be
  // used as a starting point and write access will be removed. Returns itself
  // to allow chaining calls.
  WorldACLSpec& DenyWriteFor(const WorldHashSet<EntityId>& entity_ids);

  // Sets the WorldEntityACLSpec for a given entity id. It will replace any
  // existing access permissions previously set for the entity id. Returns
  // itself to allow chaining calls.
  WorldACLSpec& SetAccessFor(EntityId entity_id, WorldEntityACLSpec acl);

  // Removes any previously set specs for the given entity.
  WorldACLSpec& ClearAccessFor(const WorldHashSet<EntityId>& entity_ids);

  // Returns the WorldEntityACLSpec for the given entity, or the default acl if
  // there wasn't one previous set specifically for the given entity id. Returns
  // itself to allow chaining calls.
  const WorldEntityACLSpec& GetSpecForEntityId(EntityId entity_id) const;

  // Returns true if the given entity has a non default spec.
  bool HasSpecForEntityId(EntityId entity_id) const;

  // Returns the set of entities that have specified ACLs.
  WorldHashSet<EntityId> GetAllSpecifiedEntities() const;

  // Returns ok status if the given spec and this spec do not conflicting acls
  // for the same entity. Conflicting in this case means the same as for a
  // mutex, if they both have read permissions then they can co-exist, if either
  // has a write permission then we will return an error because they are not
  // compatible.
  absl::Status IsCompatibleWith(const WorldACLSpec& other) const;

  // Returns true if the current and given spec are equivalent.
  bool operator==(const WorldACLSpec& other) const;

  // Returns true if the current and given spec are not equivalent.
  bool operator!=(const WorldACLSpec& other) const;

  // Returns a proto version of this world acl spec instance.
  intrinsic_proto::world::WorldACLSpec ToProto() const;

  // Returns an instance of a WorldACLSpec parsed rom the given proto.
  static absl::StatusOr<WorldACLSpec> FromProto(
      const intrinsic_proto::world::WorldACLSpec& proto_data);

 private:
  friend std::ostream& operator<<(std::ostream& os, const WorldACLSpec& acls);

  // Appends the given ACLs to the existing ACLs for the entity or default
  // acls if the previous ones were not set. It uses a logical AND operation
  // on each permission to calculate the resulting set of permission. The
  // computed ACLs are stored for the entity.
  WorldACLSpec& AppendEntitySpecsWithAnd(
      const WorldHashSet<EntityId>& entity_ids, WorldEntityACLSpec acl);

  // Appends the given ACLs to the existing ACLs for the entity or default acls
  // if the previous ones were not set. It uses a logical OR operation on each
  // permission to calculate the resulting set of permission. The computed ACLs
  // are stored for the entity.
  WorldACLSpec& AppendEntitySpecsWithOr(
      const WorldHashSet<EntityId>& entity_ids, WorldEntityACLSpec acl);

  WorldHashMap<EntityId, WorldEntityACLSpec> entity_acls_;
  WorldEntityACLSpec default_acl_;
};

// Prints WorldACLSpec to an ostream in a human readable version.
std::ostream& operator<<(std::ostream& os, const WorldACLSpec& acls);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_WORLD_ACL_SPEC_H_
