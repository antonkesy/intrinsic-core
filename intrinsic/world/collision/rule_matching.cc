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

#include "intrinsic/world/collision/rule_matching.h"

#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collision_action.pb.h"

namespace intrinsic {

namespace {

WorldHashSet<PhysicalEntityIdPair> FindMatchingObjectsImpl(
    const intrinsic_proto::Rule& rule,
    const WorldHashSet<PhysicalEntityId>& objects_1,
    const WorldHashSet<PhysicalEntityId>& objects_2) {
  WorldHashSet<PhysicalEntityIdPair> result;

  WorldHashSet<PhysicalEntityId> left_matching;
  if (rule.id_1_size() > 0) {
    for (const auto& id : rule.id_1()) {
      if (objects_1.contains((PhysicalEntityId(id)))) {
        left_matching.emplace(id);
      }
    }
  } else {
    left_matching = objects_1;
  }

  WorldHashSet<PhysicalEntityId> right_matching;
  if (rule.id_2_size() > 0) {
    for (const auto& id : rule.id_2()) {
      if (objects_2.contains((PhysicalEntityId(id)))) {
        right_matching.emplace(id);
      }
    }
  } else {
    right_matching = objects_2;
  }

  for (const auto& left_match : left_matching) {
    for (const auto& right_match : right_matching) {
      if (left_match != right_match) {
        result.emplace(left_match, right_match);
      }
    }
  }

  return result;
}

}  // namespace

WorldHashSet<PhysicalEntityIdPair> FindMatchingObjects(
    const intrinsic_proto::Rule& rule,
    const WorldHashSet<PhysicalEntityId>& objects_1,
    const WorldHashSet<PhysicalEntityId>& objects_2) {
  auto result1 = FindMatchingObjectsImpl(rule, objects_1, objects_2);
  auto result2 = FindMatchingObjectsImpl(rule, objects_2, objects_1);

  result1.insert(result2.begin(), result2.end());
  return result1;
}

}  // namespace intrinsic

namespace intrinsic_proto {
namespace world {

// Two margin pairs are equal if their hard margins are equal.
bool operator==(const CollisionMarginPair& pair_1,
                const CollisionMarginPair& pair_2) {
  return pair_1.hard_margin() == pair_2.hard_margin();
}

// Returns the result of comparing the hard margins
bool operator<(const CollisionMarginPair& pair_1,
               const CollisionMarginPair& pair_2) {
  return pair_1.hard_margin() < pair_2.hard_margin();
}

// Two actions are equal if they both indicate collision exclusion or
// They are both margin pairs with equal margins.
bool operator==(const CollisionAction& action_1,
                const CollisionAction& action_2) {
  if (action_1.action_case() != action_2.action_case()) {
    return false;
  }

  if (action_1.action_case() == CollisionAction::kMargin) {
    return action_1.margin() == action_2.margin();
  }

  return action_1.is_excluded() == action_2.is_excluded();
}

// We define actions relationships as:
// Exclusion relationsips have the highest priority.
// If both actions are margin pairs, their relationship is defined based on that
// of a margin pair.
bool operator<(const CollisionAction& action_1,
               const CollisionAction& action_2) {
  switch (action_1.action_case()) {
    case CollisionAction::ACTION_NOT_SET:
      // Unset case is always "greater" than a set case.
      return false;
    case CollisionAction::kIsExcluded:
      // If action_2 is not an exclusion case, then action_1 comes before it.
      return action_2.action_case() != CollisionAction::kIsExcluded;
    case CollisionAction::kMargin:
      // action_1 comes first if action_2 is unset, or its margin is greater.
      return action_2.action_case() == CollisionAction::ACTION_NOT_SET ||
             (action_2.action_case() == CollisionAction::kMargin &&
              action_1.margin() < action_2.margin());
  }
}

}  // namespace world

// Rule order is only defined based on the order of their actions. The labels of
// a rule do not participate in their order.
bool operator<(const Rule& rule_1, const Rule& rule_2) {
  return rule_1.action() < rule_2.action();
}

// Two rules are equal if all of their fields are euql.
bool operator==(const Rule& rule_1, const Rule& rule_2) {
  return rule_1.action() == rule_2.action();
}

}  // namespace intrinsic_proto
