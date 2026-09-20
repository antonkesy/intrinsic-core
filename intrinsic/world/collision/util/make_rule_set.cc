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

#include "intrinsic/world/collision/util/make_rule_set.h"

#include <cmath>
#include <limits>

#include "absl/functional/any_invocable.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "gloop/util/gtl/flat_set.h"
#include "google/protobuf/repeated_ptr_field.h"
#include "intrinsic/util/aggregate_type.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_creation_utils.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/collision_action.pb.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace {

using ::intrinsic::object_world::object_world_object_entity_filter_details::
    GetObjectCollisionEntitiesMatchingEntityFilter;
using ::intrinsic_proto::world::CollisionSettings;
using ::intrinsic_proto::world::ObjectOrEntityReference;

// Adds all intrinsic_proto::Rule corresponding to the objects represented by
// `collision_rule` to `rule_set`.
absl::Status AddRulesForCollisionRule(
    const object_world::ObjectWorld& world,
    const CollisionSettings::CollisionRule& collision_rule,
    intrinsic_proto::RuleSet& rule_set) {
  if (!collision_rule.has_collision_action()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Rule needs to specify collision action, but it doesn't. Rule: [%v]",
        collision_rule));
  }

  if (collision_rule.collision_action().has_margin()) {
    if (collision_rule.collision_action().margin().hard_margin() < 0) {
      return absl::InvalidArgumentError(
          "Rule specifies a negative hard margin.");
    }
  }

  INTR_ASSIGN_OR_RETURN(
      const gtl::flat_set<CollisionEntityId> left_entities,
      GetCollisionEntityIDsFromObjectReference(world, collision_rule.left()));
  if (left_entities.empty()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "'objects' in object rule is empty or it doesn't reference any valid "
        "objects, which is not allowed. Rule: [%v]",
        collision_rule));
  }

  const bool right_is_anything = collision_rule.right().empty();

  INTR_ASSIGN_OR_RETURN(
      const gtl::flat_set<CollisionEntityId> right_entities,
      GetCollisionEntityIDsFromObjectReference(world, collision_rule.right()));

  if (!right_is_anything && right_entities.empty()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "'others' in object rule attempts to specify objects, but no valid "
        "associated entities could be found. Rule: [%v]",
        collision_rule));
  }

  if (left_entities.size() == 1 && right_entities.size() == 1 &&
      *left_entities.begin() == *right_entities.begin()) {
    return absl::OkStatus();
  }

  auto* rule = rule_set.add_rules();
  *rule->mutable_action() = collision_rule.collision_action();
  for (const CollisionEntityId left : left_entities) {
    rule->add_id_1(left.value());
  }
  for (const CollisionEntityId right : right_entities) {
    rule->add_id_2(right.value());
  }

  return absl::OkStatus();
}

}  // namespace

// Returns the set of CollisionEntityIds contained by each Object referenced by
// each ObjectReference in `object_references`.
absl::StatusOr<gtl::flat_set<CollisionEntityId>>
GetCollisionEntityIDsFromObjectReference(
    const object_world::ObjectWorld& world,
    const google::protobuf::RepeatedPtrField<ObjectOrEntityReference>&
        object_references) {
  gtl::flat_set<CollisionEntityId> collision_ids;
  const World& entity_world = world.GetEntityWorld();
  for (const ObjectOrEntityReference& object_ref : object_references) {
    switch (object_ref.type_case()) {
      case intrinsic_proto::world::ObjectOrEntityReference::kObject: {
        INTR_ASSIGN_OR_RETURN(
            const object_world::WorldObject* object,
            object_world::GetObjectByReference(world, object_ref.object()));
        for (const AttachmentEntityId attachment_id : object->GetEntityIds()) {
          if (absl::StatusOr<CollisionEntityId> collision_id =
                  entity_world.ValidateEntity<CollisionEntityId>(attachment_id);
              collision_id.ok()) {
            collision_ids.insert(*collision_id);
          }
        }
      } break;
      case intrinsic_proto::world::ObjectOrEntityReference::kObjectWithFilter: {
        INTR_ASSIGN_OR_RETURN(
            const object_world::WorldObject* object,
            object_world::GetObjectByReference(
                world, object_ref.object_with_filter().reference()));
        const world::ObjectEntityFilter filter =
            world::ObjectEntityFilter::FromProto(
                object_ref.object_with_filter().entity_filter());
        INTR_ASSIGN_OR_RETURN(
            WorldHashSet<CollisionEntityId> entity_ids,
            GetObjectCollisionEntitiesMatchingEntityFilter(*object, filter));
        collision_ids.insert(entity_ids.begin(), entity_ids.end());
      } break;
      case intrinsic_proto::world::ObjectOrEntityReference::kEntity: {
        INTR_ASSIGN_OR_RETURN(
            EntityId entity_id,
            object_world::ObjectWorldResourceIdToEntityId(
                ObjectWorldResourceId(object_ref.entity().id())));
        if (absl::StatusOr<CollisionEntityId> collision_id =
                entity_world.ValidateEntity<CollisionEntityId>(entity_id);
            collision_id.ok()) {
          collision_ids.insert(*collision_id);
        }
      } break;
      case intrinsic_proto::world::ObjectOrEntityReference::TYPE_NOT_SET: {
        return absl::InvalidArgumentError(
            "Unset type for ObjectOrEntityReference");
      }
      default: {
        return absl::UnimplementedError(
            absl::StrCat("Unknown type (", object_ref.type_case(),
                         ") for ObjectOrEntityReference"));
      }
    }
  }
  // TODO(efernan): Consider optionally adding all CollisionEntityId of the
  // children objects of this object as well, as it may make it easier to
  // specify margins in a concise manner.
  return collision_ids;
}

intrinsic_proto::RuleSet MakeRuleSetFromMargin(double margin) {
  intrinsic_proto::RuleSet rule_set;
  auto* margins = rule_set.add_rules()->mutable_action()->mutable_margin();
  margins->set_hard_margin(margin);
  return rule_set;
}

intrinsic_proto::Rule MakeRuleFromCollisionAction(
    const intrinsic_proto::world::CollisionAction& action) {
  intrinsic_proto::Rule rule;
  *rule.mutable_action() = action;
  return rule;
}

absl::StatusOr<intrinsic_proto::RuleSet> MakeRuleSet(
    const intrinsic_proto::world::CollisionSettings& collision_settings,
    absl::AnyInvocable<absl::StatusOr<const object_world::ObjectWorld*>()>&
        object_world_provider) {
  if (collision_settings.disable_collision_checking()) {
    // While we could define a RuleSet that effectively disables collision, we
    // often want to shortcut collision checking altogether when the setting is
    // disabled. Not handling this condition elsewhere is therefore reported as
    // an error here.
    return absl::InvalidArgumentError(
        absl::StrFormat("Attempted to create a RuleSet but collision_checking "
                        "was set as disabled."));
  }

  intrinsic_proto::RuleSet rule_set;

  if (collision_settings.has_minimum_margin()) {
    const double minimum_margin = collision_settings.minimum_margin();
    if (minimum_margin < 0) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "The minimum_margin has to be >=0, but it is: %.5f", minimum_margin));
    }
    if (!std::isfinite(minimum_margin)) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "The minimum_margin must be finite, but it is %.5f", minimum_margin));
    }
    if (minimum_margin > std::numeric_limits<float>::max()) {
      return absl::InvalidArgumentError(
          absl::StrFormat("The minimum_margin must fit within 32-bit floating "
                          "point precision (%.5f), but it is %.5f",
                          std::numeric_limits<float>::max(), minimum_margin));
    }
    // Note that this rule catches everything. This may result in some or all of
    // the collision_rules below being ignored (if they have lower priority
    // thant this one -- i.e. when their margin is greater than this
    // minimum_margin).
    intrinsic_proto::Rule* rule = rule_set.add_rules();
    intrinsic_proto::world::CollisionMarginPair* margins =
        rule->mutable_action()->mutable_margin();
    margins->set_hard_margin(minimum_margin);
  }

  if (!collision_settings.collision_rules().empty()) {
    auto world = object_world_provider();
    if (!world.ok() || *world == nullptr) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "CollisionSettings specifies pairwise rules, which require an "
          "ObjectWorld. However, an ObjectWorld couldn't be acquired. Is the "
          "current world ObjectWorld-compatible? Error: %s",
          world.status().message()));
    }
    const object_world::ObjectWorld& object_world = *world.value();
    for (const auto& collision_rule : collision_settings.collision_rules()) {
      INTR_RETURN_IF_ERROR(
          AddRulesForCollisionRule(object_world, collision_rule, rule_set));
    }
  }

  return rule_set;
}

absl::StatusOr<intrinsic_proto::RuleSet> MakeRuleSet(
    const intrinsic_proto::world::CollisionSettings& collision_settings,
    const object_world::ObjectWorld& object_world) {
  absl::AnyInvocable<absl::StatusOr<const object_world::ObjectWorld*>()>
      object_world_provider = [&]() { return &object_world; };
  return MakeRuleSet(collision_settings, object_world_provider);
}

}  // namespace intrinsic
