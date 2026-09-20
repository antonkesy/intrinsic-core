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

#ifndef INTRINSIC_WORLD_UTIL_COLLISION_ACTION_UTIL_H_
#define INTRINSIC_WORLD_UTIL_COLLISION_ACTION_UTIL_H_

#include <optional>

#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/proto/collision_action.pb.h"

namespace intrinsic {

// Finds and returns the collision action associated with the given entity
// within the rule set. It must be an action that relates the entity to all
// others. If a pair-wise action is available, it will be ignored and not
// returned. If more than one rule exists the last rule will be used.
std::optional<intrinsic_proto::world::CollisionAction>
GetCollisionActionForEntity(const intrinsic_proto::RuleSet& rule_set,
                            EntityId entity_id);

// Sets the collision action associated with the given entity
// within the rule set. Any other rules that relate the entity to all others
// will be removed.
intrinsic_proto::RuleSet SetCollisionActionForEntity(
    const intrinsic_proto::RuleSet& rule_set, EntityId entity_id,
    std::optional<intrinsic_proto::world::CollisionAction> collision_action);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_COLLISION_ACTION_UTIL_H_
