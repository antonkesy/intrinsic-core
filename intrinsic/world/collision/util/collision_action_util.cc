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

#include "intrinsic/world/collision/util/collision_action_util.h"

#include <optional>
#include <utility>

#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/proto/collision_action.pb.h"

namespace intrinsic {
namespace {

bool MatchesEntityToAllRule(const intrinsic_proto::Rule& rule,
                            EntityId entity_id) {
  if (rule.id_2_size() == 0) {
    for (const auto& id : rule.id_1()) {
      if (id == entity_id.value()) {
        return true;
      }
    }
  }

  if (rule.id_1_size() == 0) {
    for (const auto& id : rule.id_2()) {
      if (id == entity_id.value()) {
        return true;
      }
    }
  }

  return false;
}

// Removes any rules that are enttiy_id vs *, and leaves all others alone.
// This includes partial modifications when a rule has multiple entity ids vs *.
std::optional<intrinsic_proto::Rule> ScrubEntityToAllRule(
    const intrinsic_proto::Rule& rule, EntityId entity_id) {
  if (rule.id_2_size() == 0) {
    if (rule.id_1_size() == 1 && rule.id_1(0) == entity_id.value()) {
      return std::nullopt;
    }

    intrinsic_proto::Rule result = rule;
    for (auto itr = result.mutable_id_1()->begin();
         itr != result.mutable_id_1()->end();) {
      if (*itr == entity_id.value()) {
        itr = result.mutable_id_1()->erase(itr);
      } else {
        itr++;
      }
    }
    return result;
  }

  if (rule.id_1_size() == 0) {
    if (rule.id_2_size() == 1 && rule.id_2(0) == entity_id.value()) {
      return std::nullopt;
    }

    intrinsic_proto::Rule result = rule;
    for (auto itr = result.mutable_id_2()->begin();
         itr != result.mutable_id_2()->end();) {
      if (*itr == entity_id.value()) {
        itr = result.mutable_id_2()->erase(itr);
      } else {
        itr++;
      }
    }

    return result;
  }

  return rule;
}

}  // namespace

using ::intrinsic_proto::world::CollisionAction;

std::optional<CollisionAction> GetCollisionActionForEntity(
    const intrinsic_proto::RuleSet& rule_set, EntityId entity_id) {
  std::optional<CollisionAction> rule_action = std::nullopt;
  for (const auto& rule : rule_set.rules()) {
    if (MatchesEntityToAllRule(rule, entity_id)) {
      rule_action = rule.action();
    }
  }

  return rule_action;
}

intrinsic_proto::RuleSet SetCollisionActionForEntity(
    const intrinsic_proto::RuleSet& rule_set, EntityId entity_id,
    std::optional<intrinsic_proto::world::CollisionAction> collision_action) {
  intrinsic_proto::RuleSet updated_rules;
  updated_rules.mutable_rules()->Reserve(rule_set.rules_size());
  for (const auto& rule : rule_set.rules()) {
    auto new_rule = ScrubEntityToAllRule(rule, entity_id);
    if (new_rule.has_value()) {
      *updated_rules.add_rules() = std::move(new_rule).value();
    }
  }

  // Add a new rule for the entity being updated if a collision action is
  // provided.
  if (collision_action.has_value()) {
    auto* new_rule = updated_rules.add_rules();
    new_rule->add_id_1(entity_id.value());
    *new_rule->mutable_action() = *collision_action;
  }

  return updated_rules;
}

}  // namespace intrinsic
