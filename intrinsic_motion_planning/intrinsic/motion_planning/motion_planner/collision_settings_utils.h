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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_COLLISION_SETTINGS_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_COLLISION_SETTINGS_UTILS_H_

#include "absl/status/statusor.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/proto/collision_settings.pb.h"

namespace intrinsic {
// Checks if the two collision rules are the same. Two collision rules are
// considered the same if the specify the same actions for the same set of left
// and right objects.
absl::StatusOr<bool> CollisionRulesMatch(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_1,
    const intrinsic_proto::world::CollisionSettings_CollisionRule& rule_2);

// Checks if the collision settings are the same. Two collision settings are
// considered the same if they contain an equivalent set of collision rules,
// maximum margin, or both disabled the collision checking.
absl::StatusOr<bool> CollisionSettingsMatch(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings& setting_1,
    const intrinsic_proto::world::CollisionSettings& setting_2);

// Returns the most relaxed combination of the two provided collision settings,
// in the sense that:
// (1) If either setting disables collision checking, the combined setting
//     will simply disable collision checking.
// (2) If either setting does not specify a minimum margin, the combined
//     setting will not specify a minimum margin.
// (3) If both settings specify a minimum margin, the combined setting will
//     specify the lower of the two margins.
// (4) If both settings specify a collision rule:
//   (a) If the collision rules match, add the rule to the combined setting
//       as-is.
//   (b) If there is a collision rule that has the same object pairs, add the
//       rule to the combined setting and select the most relaxed collision
//       action between the two rules, as follows.
//     (b.1) If any of the rules has `is_excluded == true`, then this is the
//           most relaxed collision action.
//     (b.2) The `margin` is the closest we allow two objects to get before we
//           treat it as a collision. Thus, the smallest `margin` among the two
//           is the most relaxed collision action's `margin`.
//     (b.3) If one rule has margin and the other rule has
//           `is_excluded == false`, then the most relaxed collision action is
//           the rule with `is_excluded == false`.
//   (c) Add the remaining collision rules from both settings that satisfy all
//       of the following conditions:
//     (c.1) do not have a match in the other setting, and
//     (c.2) have `is_excluded == true`.
// (5) If both settings specify a collision check spacing, the combined setting
//     will specify the higher of the two collision check spacings.
absl::StatusOr<intrinsic_proto::world::CollisionSettings>
GetMostRelaxedCollisionSettingCombination(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings& setting_1,
    const intrinsic_proto::world::CollisionSettings& setting_2);

// Returns the most restricted combination of the two provided collision
// settings, in the sense that:
// (1) If one of the settings disables collision checking, the combined setting
//     will still perform collision checking --exactly following the
//     non-disabled collision checking setting--.
// (2) If one of the settings specifies a minimum margin while the other one
//     does not, the combined setting will specify a minimum margin --exactly
//     following the setting with a minimum margin--.
// (3) If both settings specify a minimum margin, the combined setting will
//     specify the higher of the two margins.
// (4) If both settings specify a collision rule:
//   (a) If the collision rules match, add the rule to the combined setting
//       as-is.
//   (b) If there is a collision rule that has the same object pairs, add the
//       rule to the combined setting and select the most restricted collision
//       action between the two rules, as follows.
//     (b.1) If one of the rules has `is_excluded == true` while the other does
//           not have, then the most restricted collision action is the other
//           rule.
//     (b.2) The `margin` is the closest we allow two objects to get before we
//           treat it as a collision. Thus, the higher `margin` among the two
//           is the most restricted collision action's `margin`.
//     (b.3) If one rule has margin and the other rule has
//           `is_excluded == false`, then the most restricted collision action
//           is the rule with margin.
//   (c) Add the remaining collision rules from both settings that do not have a
//       match in the other setting and either:
//     (c.1) have `is_excluded == false`, or
//     (c.2) have `margin`.
//       Please refer to the documentation in the implementation for a detailed
//       explanation on the reasoning behind (c).
// (5) If both settings specify a collision check spacing, the combined setting
//     will specify the lower of the two collision check spacings.
absl::StatusOr<intrinsic_proto::world::CollisionSettings>
GetMostRestrictedCollisionSettingCombination(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::world::CollisionSettings& setting_1,
    const intrinsic_proto::world::CollisionSettings& setting_2);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_COLLISION_SETTINGS_UTILS_H_
