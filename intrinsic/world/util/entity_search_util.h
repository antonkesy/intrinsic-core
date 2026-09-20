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

#ifndef INTRINSIC_WORLD_UTIL_ENTITY_SEARCH_UTIL_H_
#define INTRINSIC_WORLD_UTIL_ENTITY_SEARCH_UTIL_H_

#include <string>
#include <type_traits>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/entity_search.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// Returns a set of EntityIds that match the search criteria within the given
// world instance. If allow_multiple is not set and we match multiple entities,
// an error will be returned. If we do not find any entities an error will be
// returned.
absl::StatusOr<WorldHashSet<EntityId>> GetEntities(
    const World& world,
    const intrinsic_proto::world::EntitySearchCriteria& criteria,
    bool allow_multiple = true);

// Returns the single, unique EntityId that matches the search criteria within
// the given world instance. If we do no match exactly one entity an error will
// be returned.
// Equivalent to calling GetEntities() with allow_multiple = false.
absl::StatusOr<EntityId> GetSingleEntity(
    const World& world,
    const intrinsic_proto::world::EntitySearchCriteria& criteria);

// Convenience method for GetSingleEntity() followed by World::ValidateEntity().
// Examples:
//
// INTR_ASSIGN_OR_RETURN(
//     TypedEntityId<RobotComponentType> id,
//     GetSingleTypedEntity<RobotComponentType>(world, criteria));
//
// INTR_ASSIGN_OR_RETURN(
//     PhysicalEntityId id,
//     GetSingleTypedEntity<PhysicalEntityId>(world, criteria));
template <typename... ComponentTypes>
absl::StatusOr<world_entity_details::TypedResult<ComponentTypes...>>
GetSingleTypedEntity(
    const World& world,
    const intrinsic_proto::world::EntitySearchCriteria& criteria) {
  INTR_ASSIGN_OR_RETURN(EntityId id, GetSingleEntity(world, criteria));
  return world.ValidateEntity<ComponentTypes...>(id);
}

// Create a by path EntitySearchCriteria
intrinsic_proto::world::EntitySearchCriteria CreatePathSearchCriteria(
    const std::vector<std::string>& local_names, bool can_skip);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_ENTITY_SEARCH_UTIL_H_
