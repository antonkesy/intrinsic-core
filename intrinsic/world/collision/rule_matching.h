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

#ifndef INTRINSIC_WORLD_RULE_MATCHING_H_
#define INTRINSIC_WORLD_RULE_MATCHING_H_

#include <ostream>
#include <utility>

#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collision_action.pb.h"

// This file provides sorting utilities for rules and actions. It also provides
// a rule matching utility that finds the first matching rule given a rule set
// and two groups of labels. The ruleset should be sorted based on the priority
// of the rules. operator< on Rule and Action defines the priority.

namespace intrinsic {

using PhysicalEntityIdPair = std::pair<PhysicalEntityId, PhysicalEntityId>;

// A stream operator for PhysicalEntityId pair needed by gtl::FindOrDie
inline std::ostream& operator<<(std::ostream& os,
                                const PhysicalEntityIdPair& object_pair) {
  os << "{" << object_pair.first << ", " << object_pair.second << "}";
  return os;
}

// Returns pair of objects from objects_1, and objects_2 that match with the
// rule. Labels associated with objects are obtained from label_objects.
// Flags objects_1_dynamic and objects_2_dynamic indicate whether the
// corresponding objects are dynamic.
WorldHashSet<PhysicalEntityIdPair> FindMatchingObjects(
    const intrinsic_proto::Rule& rule,
    const WorldHashSet<PhysicalEntityId>& objects_1,
    const WorldHashSet<PhysicalEntityId>& objects_2);

}  // namespace intrinsic

namespace intrinsic_proto {
namespace world {

bool operator==(const CollisionMarginPair& pair_1,
                const CollisionMarginPair& pair_2);
bool operator<(const CollisionMarginPair& pair_1,
               const CollisionMarginPair& pair_2);

bool operator==(const CollisionAction& action_1,
                const CollisionAction& action_2);
bool operator<(const CollisionAction& action_1,
               const CollisionAction& action_2);

}  // namespace world

bool operator==(const Rule& rule_1, const Rule& rule_2);
bool operator<(const Rule& rule_1, const Rule& rule_2);

}  // namespace intrinsic_proto

#endif  // INTRINSIC_WORLD_RULE_MATCHING_H_
