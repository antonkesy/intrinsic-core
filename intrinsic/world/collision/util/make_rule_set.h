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

#ifndef INTRINSIC_WORLD_UTIL_MAKE_RULE_SET_H_
#define INTRINSIC_WORLD_UTIL_MAKE_RULE_SET_H_

#include "absl/functional/any_invocable.h"
#include "absl/status/statusor.h"
#include "gloop/util/gtl/flat_set.h"
#include "google/protobuf/repeated_ptr_field.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/proto/collision_settings.pb.h"

namespace intrinsic {

// Returns a rule set that has a global hard and soft margin equal to the given
// value, applied to all entities.
intrinsic_proto::RuleSet MakeRuleSetFromMargin(double margin);

// Returns a rule that has the given action and is applied to all entities.
intrinsic_proto::Rule MakeRuleFromCollisionAction(
    const intrinsic_proto::world::CollisionAction& action);

// Makes and returns a rule set from CollisionSettings. Returns an error
// status if the provided CollisionSettings is invalid.
absl::StatusOr<intrinsic_proto::RuleSet> MakeRuleSet(
    const intrinsic_proto::world::CollisionSettings& collision_settings,
    const object_world::ObjectWorld& object_world);

// Variant of MakeRuleSet(...) that requires an `object_world_provider`
// returning an ObjectWorld only when `collision_settings` contains pairwise
// collision rules. Otherwise, `object_world_provider` is not called.
// TODO(b/219463482): Remove this variant once all app worlds are object world
// compatible and it is safe to assume that an ObjectWorld can always be
// acquired.
absl::StatusOr<intrinsic_proto::RuleSet> MakeRuleSet(
    const intrinsic_proto::world::CollisionSettings& collision_settings,
    absl::AnyInvocable<absl::StatusOr<const object_world::ObjectWorld*>()>&
        object_world_provider);

absl::StatusOr<gtl::flat_set<CollisionEntityId>>
GetCollisionEntityIDsFromObjectReference(
    const object_world::ObjectWorld& world,
    const google::protobuf::RepeatedPtrField<
        intrinsic_proto::world::ObjectOrEntityReference>& object_references);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_MAKE_RULE_SET_H_
