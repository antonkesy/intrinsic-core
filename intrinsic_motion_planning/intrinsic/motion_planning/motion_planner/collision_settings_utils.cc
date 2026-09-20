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

#include "intrinsic/motion_planning/motion_planner/collision_settings_utils.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "gloop/util/gtl/flat_set.h"
#include "google/protobuf/repeated_ptr_field.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/util/make_rule_set.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/proto/collision_action.pb.h"
#include "intrinsic/world/proto/collision_settings.pb.h"

namespace intrinsic {
namespace {

// Check if the repeated set of ObjectOrEntityReferences are the same. To be
// considered the same, it is expected that they have the same number of
// entities and that every entity in one set has a match in the other set.
absl::StatusOr<bool> ObjectReferenceSetsMatch(
    const object_world::ObjectWorld& object_world,
    const google::protobuf::RepeatedPtrField<
        intrinsic_proto::world::ObjectOrEntityReference>&
        object_references_left,
    const google::protobuf::RepeatedPtrField<
        intrinsic_proto::world::ObjectOrEntityReference>&
        object_references_right) {
  INTR_ASSIGN_OR_RETURN(gtl::flat_set<CollisionEntityId> rule_1_entities,
                        GetCollisionEntityIDsFromObjectReference(
                            object_world, object_references_left));

  INTR_ASSIGN_OR_RETURN(gtl::flat_set<CollisionEntityId> rule_2_entities,
                        GetCollisionEntityIDsFromObjectReference(
                            object_world, object_references_right));

  if (rule_1_entities.size() != rule_2_entities.size()) {
    return false;
  }

  for (const CollisionEntityId rule1_entity : rule_1_entities) {
    const auto it = rule_2_entities.find(rule1_entity);
    if (it == rule_2_entities.end()) {
      return false;
    }
  }

  return true;
}

absl::StatusOr<bool> RuleObjectMatch(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_1,
    const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_2) {
  INTR_ASSIGN_OR_RETURN(
      bool right_right_object_match,
      ObjectReferenceSetsMatch(object_world, rule_1.right(), rule_2.right()));
  INTR_ASSIGN_OR_RETURN(
      bool right_left_object_match,
      ObjectReferenceSetsMatch(object_world, rule_1.right(), rule_2.left()));
  if (!(right_right_object_match || right_left_object_match)) {
    return false;
  }
  if (right_right_object_match) {
    INTR_ASSIGN_OR_RETURN(
        bool left_object_match,
        ObjectReferenceSetsMatch(object_world, rule_1.left(), rule_2.left()));
    if (!left_object_match) {
      return false;
    }
  } else {
    INTR_ASSIGN_OR_RETURN(
        bool left_object_match,
        ObjectReferenceSetsMatch(object_world, rule_1.left(), rule_2.right()));
    if (!left_object_match) {
      return false;
    }
  }
  return true;
}
}  // namespace

absl::StatusOr<bool> CollisionRulesMatch(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_1,
    const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_2) {
  if (rule_1.collision_action().has_is_excluded() !=
      rule_2.collision_action().has_is_excluded()) {
    return false;
  }
  if (rule_1.collision_action().has_margin() !=
      rule_2.collision_action().has_margin()) {
    return false;
  }
  if (rule_1.collision_action().has_is_excluded() &&
      rule_2.collision_action().has_is_excluded() &&
      (rule_1.collision_action().is_excluded() ==
       rule_2.collision_action().is_excluded())) {
    // Check if they exclude the same pairs
    INTR_ASSIGN_OR_RETURN(bool object_match,
                          RuleObjectMatch(object_world, rule_1, rule_2));
    if (!object_match) {
      return false;
    }
    return true;
  }
  if (rule_1.collision_action().has_margin() &&
      rule_2.collision_action().has_margin()) {
    // Check if they define the same margin rule
    if (rule_1.collision_action().margin().hard_margin() !=
        rule_2.collision_action().margin().hard_margin()) {
      return false;
    }
    INTR_ASSIGN_OR_RETURN(bool object_match,
                          RuleObjectMatch(object_world, rule_1, rule_2));
    if (!object_match) {
      return false;
    }
    return true;
  }
  return false;
}

absl::StatusOr<bool> CollisionSettingsMatch(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings& setting_1,
    const intrinsic_proto::world::CollisionSettings& setting_2) {
  // Same disable settings.
  if (setting_1.disable_collision_checking() !=
      setting_2.disable_collision_checking()) {
    return false;
  }
  if (setting_1.disable_collision_checking() &&
      setting_2.disable_collision_checking()) {
    return true;
  }

  // Specify the same minimum margin.
  if (setting_1.has_minimum_margin() != setting_2.has_minimum_margin()) {
    return false;
  }
  if (setting_1.has_minimum_margin() && setting_2.has_minimum_margin() &&
      (setting_1.minimum_margin() != setting_2.minimum_margin())) {
    return false;
  }
  if (setting_1.collision_rules_size() != setting_2.collision_rules_size()) {
    return false;
  }

  // Specify the same collision rules
  if (setting_1.collision_rules_size() != setting_2.collision_rules_size()) {
    return false;
  }
  for (const auto& global_rule : setting_1.collision_rules()) {
    bool found_match = false;
    for (const auto& local_rule : setting_2.collision_rules()) {
      INTR_ASSIGN_OR_RETURN(
          const bool collision_settings_match,
          CollisionRulesMatch(object_world, global_rule, local_rule));
      if (collision_settings_match) {
        found_match = true;
        break;
      }
    }
    if (!found_match) {
      return false;
    }
  }
  return true;
}

absl::StatusOr<intrinsic_proto::world::CollisionSettings>
GetMostRelaxedCollisionSettingCombination(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings& setting_1,
    const intrinsic_proto::world::CollisionSettings& setting_2) {
  intrinsic_proto::world::CollisionSettings most_relaxed_combined_setting;
  // If either setting disables collision checking, the combined setting will
  // simply disable collision checking.
  if (setting_1.disable_collision_checking() ||
      setting_2.disable_collision_checking()) {
    most_relaxed_combined_setting.set_disable_collision_checking(true);
    return most_relaxed_combined_setting;
  }

  if (!setting_1.has_minimum_margin() || !setting_2.has_minimum_margin()) {
    // If either setting does not specify a minimum margin, the combined setting
    // will not specify a minimum margin.
    most_relaxed_combined_setting.clear_minimum_margin();
  } else {
    // If both settings specify a minimum margin, the combined setting will
    // specify the lower of the two margins.
    most_relaxed_combined_setting.set_minimum_margin(
        std::min(setting_1.minimum_margin(), setting_2.minimum_margin()));
  }

  // Below we identify the collision rules which have a match in the other
  // setting, and determine how we combine them to get the most relaxed setting.
  std::vector<bool> rule1_has_match(setting_1.collision_rules_size(), false);
  std::vector<bool> rule2_has_match(setting_2.collision_rules_size(), false);
  for (size_t i = 0; i < setting_1.collision_rules_size(); ++i) {
    const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_1 =
        setting_1.collision_rules(i);
    for (size_t j = 0; j < setting_2.collision_rules_size(); ++j) {
      const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_2 =
          setting_2.collision_rules(j);
      INTR_ASSIGN_OR_RETURN(const bool collision_rules_match,
                            CollisionRulesMatch(object_world, rule_1, rule_2));

      // Matching collision rules imply matching rule objects, so we only need
      // to check for rule object match if the collision rules do not match.
      bool rule_object_match = collision_rules_match;
      if (!collision_rules_match) {
        INTR_ASSIGN_OR_RETURN(rule_object_match,
                              RuleObjectMatch(object_world, rule_1, rule_2));
      }

      if (collision_rules_match || rule_object_match) {
        rule1_has_match[i] = true;
        rule2_has_match[j] = true;
        if (collision_rules_match) {
          // If the collision rules match, add the rule to the combined setting
          // as-is.
          *most_relaxed_combined_setting.add_collision_rules() = rule_1;
        } else {  // rule_object_match
          // If there is a collision rule that has the same object pairs, add
          // the rule to the combined setting and select the most relaxed
          // collision action between the two rules.

          intrinsic_proto::world::CollisionSettings_CollisionRule* new_rule =
              most_relaxed_combined_setting.add_collision_rules();
          new_rule->mutable_left()->CopyFrom(rule_1.left());
          new_rule->mutable_right()->CopyFrom(rule_1.right());

          // If any of the rules has `is_excluded == true`, then this is the
          // most relaxed collision action.
          if ((rule_1.collision_action().has_is_excluded() &&
               rule_1.collision_action().is_excluded()) ||
              (rule_2.collision_action().has_is_excluded() &&
               rule_2.collision_action().is_excluded())) {
            new_rule->mutable_collision_action()->set_is_excluded(true);
          }
          // The `margin` is the closest we allow two objects to get before we
          // treat it as a collision. Thus, the smallest `margin` among the two
          // is the most relaxed collision action's `margin`.
          else if (rule_1.collision_action().has_margin() &&
                   rule_2.collision_action().has_margin()) {
            new_rule->mutable_collision_action()
                ->mutable_margin()
                ->set_hard_margin(
                    std::min(rule_1.collision_action().margin().hard_margin(),
                             rule_2.collision_action().margin().hard_margin()));
          }
          // If one rule has margin and the other rule has
          // `is_excluded == false`, then the most relaxed collision action is
          // the rule with `is_excluded == false`.
          else if (rule_1.collision_action().has_margin() &&
                   (rule_2.collision_action().has_is_excluded() &&
                    !rule_2.collision_action().is_excluded())) {
            *new_rule->mutable_collision_action() = rule_2.collision_action();
          } else if (rule_2.collision_action().has_margin() &&
                     (rule_1.collision_action().has_is_excluded() &&
                      !rule_1.collision_action().is_excluded())) {
            *new_rule->mutable_collision_action() = rule_1.collision_action();
          } else {
            return absl::InvalidArgumentError(absl::StrCat(
                "The matching collision actions in the collision rule cannot "
                "be combined into the most relaxed one, rule 1's collision "
                "action: (",
                rule_1.collision_action(), ") and rule 2's collision action: (",
                rule_2.collision_action(), ")."));
          }
        }
        break;
      }
    }
  }

  // Add the remaining collision rules from both settings that satisfy all of
  // the following conditions:
  // (1) do not have a match in the other setting, and
  // (2) have `is_excluded == true`.
  for (size_t i = 0; i < setting_1.collision_rules_size(); ++i) {
    if (!rule1_has_match[i]) {
      const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_1 =
          setting_1.collision_rules(i);
      if (rule_1.collision_action().has_is_excluded() &&
          rule_1.collision_action().is_excluded()) {
        *most_relaxed_combined_setting.add_collision_rules() = rule_1;
      }
    }
  }
  for (size_t j = 0; j < setting_2.collision_rules_size(); ++j) {
    if (!rule2_has_match[j]) {
      const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_2 =
          setting_2.collision_rules(j);
      if (rule_2.collision_action().has_is_excluded() &&
          rule_2.collision_action().is_excluded()) {
        *most_relaxed_combined_setting.add_collision_rules() = rule_2;
      }
    }
  }

  return most_relaxed_combined_setting;
}

absl::StatusOr<intrinsic_proto::world::CollisionSettings>
GetMostRestrictedCollisionSettingCombination(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings& setting_1,
    const intrinsic_proto::world::CollisionSettings& setting_2) {
  intrinsic_proto::world::CollisionSettings most_restricted_combined_setting;

  // If one of the settings disables collision checking, the combined setting
  // will still perform collision checking --exactly following the non-disabled
  // collision checking setting--.
  if (setting_1.disable_collision_checking() &&
      !setting_2.disable_collision_checking()) {
    return setting_2;
  } else if (!setting_1.disable_collision_checking() &&
             setting_2.disable_collision_checking()) {
    return setting_1;
  } else if (setting_1.disable_collision_checking() &&
             setting_2.disable_collision_checking()) {
    most_restricted_combined_setting.set_disable_collision_checking(true);
    return most_restricted_combined_setting;
  }

  // TODO(b/489802845 and b/277629781): The implementation here is somewhat
  // de-coupled/independent from the underlying parsing of the
  // `CollisionSettings` (which has the member field `minimum_margin`) and
  // `CollisionRule` (which has the member field
  // `CollisionAction`::`CollisionMarginPair`::`hard_margin`).
  if (setting_1.has_minimum_margin() && setting_2.has_minimum_margin()) {
    // The margin is the closest we allow two objects to get before we treat it
    // as a collision. Thus, if both settings specify a minimum margin, the
    // most restricted combined setting will specify the higher of the two
    // margins.
    most_restricted_combined_setting.set_minimum_margin(
        std::max(setting_1.minimum_margin(), setting_2.minimum_margin()));
  }
  // If one of the settings specifies a minimum margin while the other one
  // does not, the combined setting will specify a minimum margin --exactly
  // following the setting with a minimum margin--.
  else if (!setting_1.has_minimum_margin() && setting_2.has_minimum_margin()) {
    most_restricted_combined_setting.set_minimum_margin(
        setting_2.minimum_margin());
  } else if (setting_1.has_minimum_margin() &&
             !setting_2.has_minimum_margin()) {
    most_restricted_combined_setting.set_minimum_margin(
        setting_1.minimum_margin());
  }

  // Below we identify the collision rules which have a match in the other
  // setting, and determine how we combine them to get the most restricted
  // setting.
  std::vector<bool> rule1_has_match(setting_1.collision_rules_size(), false);
  std::vector<bool> rule2_has_match(setting_2.collision_rules_size(), false);
  for (size_t i = 0; i < setting_1.collision_rules_size(); ++i) {
    const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_1 =
        setting_1.collision_rules(i);
    for (size_t j = 0; j < setting_2.collision_rules_size(); ++j) {
      const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_2 =
          setting_2.collision_rules(j);
      INTR_ASSIGN_OR_RETURN(const bool collision_rules_match,
                            CollisionRulesMatch(object_world, rule_1, rule_2));

      // Matching collision rules imply matching rule objects, so we only need
      // to check for rule object match if the collision rules do not match.
      bool rule_object_match = collision_rules_match;
      if (!collision_rules_match) {
        INTR_ASSIGN_OR_RETURN(rule_object_match,
                              RuleObjectMatch(object_world, rule_1, rule_2));
      }

      if (collision_rules_match || rule_object_match) {
        rule1_has_match[i] = true;
        rule2_has_match[j] = true;
        if (collision_rules_match) {
          // If the collision rules match, add the rule to the combined setting
          // as-is.
          *most_restricted_combined_setting.add_collision_rules() = rule_1;
        } else {  // rule_object_match
          // If there is a collision rule that has the same object pairs, add
          // the rule to the combined setting and select the most restricted
          // collision action between the two rules.

          intrinsic_proto::world::CollisionSettings_CollisionRule* new_rule =
              most_restricted_combined_setting.add_collision_rules();
          new_rule->mutable_left()->CopyFrom(rule_1.left());
          new_rule->mutable_right()->CopyFrom(rule_1.right());

          // If one of the rules has `is_excluded == true` while the other does
          // not have, then the most restricted collision action is the other
          // rule.
          if (rule_1.collision_action().has_is_excluded() &&
              rule_1.collision_action().is_excluded() &&
              (!rule_2.collision_action().has_is_excluded() ||
               !rule_2.collision_action().is_excluded())) {
            *new_rule->mutable_collision_action() = rule_2.collision_action();
          } else if (rule_2.collision_action().has_is_excluded() &&
                     rule_2.collision_action().is_excluded() &&
                     (!rule_1.collision_action().has_is_excluded() ||
                      !rule_1.collision_action().is_excluded())) {
            *new_rule->mutable_collision_action() = rule_1.collision_action();
          }
          // The `margin` is the closest we allow two objects to get before we
          // treat it as a collision. Thus, the higher `margin` among the two
          // is the most restricted collision action's `margin`.
          else if (rule_1.collision_action().has_margin() &&
                   rule_2.collision_action().has_margin()) {
            new_rule->mutable_collision_action()
                ->mutable_margin()
                ->set_hard_margin(
                    std::max(rule_1.collision_action().margin().hard_margin(),
                             rule_2.collision_action().margin().hard_margin()));
          }
          // If one rule has margin and the other rule has
          // `is_excluded == false`, then the most restricted collision action
          // is the rule with margin.
          else if (rule_1.collision_action().has_margin() &&
                   (rule_2.collision_action().has_is_excluded() &&
                    !rule_2.collision_action().is_excluded())) {
            *new_rule->mutable_collision_action() = rule_1.collision_action();
          } else if (rule_2.collision_action().has_margin() &&
                     (rule_1.collision_action().has_is_excluded() &&
                      !rule_1.collision_action().is_excluded())) {
            *new_rule->mutable_collision_action() = rule_2.collision_action();
          } else {
            return absl::InvalidArgumentError(absl::StrCat(
                "The matching collision actions in the collision rule cannot "
                "be combined into the most restricted one, rule 1's collision "
                "action: (",
                rule_1.collision_action(), ") and rule 2's collision action: (",
                rule_2.collision_action(), ")."));
          }
        }
        break;
      }
    }
  }

  // There are three possibilities of `CollisionAction`:
  // (a) have `is_excluded == true`
  //     If there is no match in the other setting, the most restricted
  //     combination is to not have the `CollisionRule` entirely.
  // (b) have `is_excluded == false`
  //     If there is no match in the other setting, the most restricted
  //     combination is to have the `CollisionRule` with `CollisionAction`
  //     having `is_excluded == false`.
  // (c) have `margin`.
  //     If there is no match in the other setting, the most restricted
  //     combination is to have the `CollisionRule` with `CollisionAction`
  //     having `margin`.
  // With the above reasoning, we add the remaining collision rules from both
  // settings that do not have a match in the other setting and either: (1) have
  // `is_excluded == false`, or (2) have `margin`.
  for (size_t i = 0; i < setting_1.collision_rules_size(); ++i) {
    if (!rule1_has_match[i]) {
      const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_1 =
          setting_1.collision_rules(i);
      if ((rule_1.collision_action().has_is_excluded() &&
           !rule_1.collision_action().is_excluded()) ||
          rule_1.collision_action().has_margin()) {
        *most_restricted_combined_setting.add_collision_rules() = rule_1;
      }
    }
  }
  for (size_t j = 0; j < setting_2.collision_rules_size(); ++j) {
    if (!rule2_has_match[j]) {
      const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_2 =
          setting_2.collision_rules(j);
      if ((rule_2.collision_action().has_is_excluded() &&
           !rule_2.collision_action().is_excluded()) ||
          rule_2.collision_action().has_margin()) {
        *most_restricted_combined_setting.add_collision_rules() = rule_2;
      }
    }
  }

  return most_restricted_combined_setting;
}

}  // namespace intrinsic
