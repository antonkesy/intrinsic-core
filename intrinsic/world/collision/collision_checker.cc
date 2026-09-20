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

#include "intrinsic/world/collision/collision_checker.h"

#include <algorithm>
#include <string>
#include <tuple>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "gloop/util/gtl/flat_set.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_checker_world.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collections_member_component.h"
#include "intrinsic/world/component/ppr_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {

namespace {

std::string GetUserDisplayString(const CollisionCheckerWorld& world,
                                 const EntityId& entity_id) {
  auto ent_or = world.GetEntityById(entity_id);
  if (!ent_or.ok()) {
    return "<INVALID ENTITY ID>";
  }

  const auto maybe_path =
      world.GetLocalNamePathString(AttachmentEntityId(entity_id.value()), "/");
  const std::string path_str =
      maybe_path.ok() ? absl::StrCat(", path=\"", *maybe_path, "\"") : "";

  const auto* ent = *ent_or;
  std::string ppr_component_str = "";
  const auto ppr_component = ent->GetComponent<PPRComponent>();

  if (ppr_component.ok()) {
    auto maybe_resource_name = (*ppr_component)->ResourceName();
    if (maybe_resource_name.has_value()) {
      absl::StrAppend(&ppr_component_str, ", resource_name=\"",
                      *maybe_resource_name, "\"");
    }
  }

  std::string collections_component_str = "";
  const auto collections_member_component =
      ent->GetComponent<CollectionsMemberComponent>();

  if (collections_member_component.ok()) {
    for (const auto& [parent_id, _] :
         (*collections_member_component)->GetParentCollectionsIdToTypesMap()) {
      absl::StrAppend(&collections_component_str,
                      ", collection=", GetUserDisplayString(world, parent_id));
    }
  }

  std::string name_str = "";
  if (!ent->GetAlias().empty()) {
    name_str = absl::StrCat(", alias=\"", ent->GetAlias(), "\"");
  } else if (!ent->GetLocalName().empty()) {
    name_str = absl::StrCat(", name=\"", ent->GetLocalName(), "\"");
  }

  const auto& labels = ent->GetLabels();
  std::string label_str;
  if (!labels.empty()) {
    label_str =
        absl::StrCat(", labels=[",
                     absl::StrJoin(labels, ", ", absl::StreamFormatter()), "]");
  }

  return absl::StrCat("Entity(", entity_id.value(), name_str, label_str,
                      ppr_component_str, path_str, ")");
}

}  // namespace

absl::StatusOr<std::tuple<std::string, std::vector<std::string>>>
CollisionChecker::GetCollisionEntitiesMessage(
    const CollisionCheckingDebug::Collision& collision) const {
  // We clip the number of entities shown in the report to 3 to avoid flooding
  // with long messages.
  constexpr int kMessageClipLimit = 3;
  const CollisionCheckerWorld& world = GetCollisionCheckerWorld();

  INTR_ASSIGN_OR_RETURN(std::string left_name,
                        world.TryGetObjectNameForEntity(collision.left_object));

  std::vector<std::string> right_entities;
  const int num_entities_limit =
      std::min<int>(collision.right_objects.size(), kMessageClipLimit);

  for (int i = 0; i < num_entities_limit; ++i) {
    const PhysicalEntityId right_object_id = collision.right_objects.data()[i];
    INTR_ASSIGN_OR_RETURN(std::string right_name,
                          world.TryGetObjectNameForEntity(right_object_id));
    right_entities.push_back(right_name);
  }
  if (collision.right_objects.size() > kMessageClipLimit) {
    right_entities.push_back(absl::StrFormat(
        "%d more entities are in collision, this message is clipped.",
        collision.right_objects.size() - kMessageClipLimit));
  }
  return std::make_tuple(left_name, right_entities);
}

std::tuple<EntityId, std::vector<EntityId>>
CollisionChecker::GetCollisionEntities(
    const CollisionCheckingDebug::Collision& collision) const {
  std::vector<EntityId> right_entities;
  right_entities.reserve(collision.right_objects.size());
  for (const PhysicalEntityId& right_object : collision.right_objects) {
    right_entities.push_back(right_object);
  }
  return std::make_tuple(collision.left_object, right_entities);
}

absl::StatusOr<std::string>
CollisionChecker::PrintCollisionCheckingDebugDetailed(
    const CollisionCheckingDebug& collision_debug) const {
  if (collision_debug.collisions.empty()) {
    return "No collisions detected";
  }
  // TODO(ferstl): This only includes entity-level information. Also include
  // object-level information such as object names of colliding objects so that
  // error messages become easier to interpret.
  const CollisionCheckerWorld& world = GetCollisionCheckerWorld();
  std::vector<std::string> collision_strings;
  for (const CollisionCheckingDebug::Collision& collision :
       collision_debug.collisions) {
    std::string left_object_string =
        GetUserDisplayString(world, collision.left_object);
    std::vector<std::string> right_object_strings;
    right_object_strings.reserve(collision.right_objects.size());
    for (const PhysicalEntityId right_object_id : collision.right_objects) {
      std::string right_object_string =
          GetUserDisplayString(world, right_object_id);
      right_object_strings.push_back(right_object_string);
    }
    collision_strings.push_back(absl::StrFormat(
        "    Left entity: %s\n    Right entities: %s \n", left_object_string,
        absl::StrJoin(right_object_strings, ", ")));
  }

  return absl::StrFormat("\t%d collisions detected:\n%s",
                         collision_debug.collisions.size(),
                         absl::StrJoin(collision_strings, "\n"));
}

absl::StatusOr<std::string> CollisionChecker::PrintCollisionCheckingDebug(
    const CollisionCheckingDebug& collision_debug) const {
  if (collision_debug.collisions.empty()) {
    return "No collisions detected";
  }
  auto collision = collision_debug.collisions[0];
  INTR_ASSIGN_OR_RETURN(auto entities, GetCollisionEntitiesMessage(collision));
  std::string& left_entity = std::get<0>(entities);
  std::vector<std::string>& right_entities = std::get<1>(entities);
  if (right_entities.empty()) {
    right_entities.push_back("No right entities reported correctly.");
  }
  if (left_entity.empty()) {
    left_entity = "No left entity reported correctly.";
  }
  return absl::StrFormat(
      "\n%d collision(s) detected:\n"
      "    Left object: %s \n"
      "    Right objects: %s"
      "\n",
      collision_debug.collisions.size(), left_entity,
      absl::StrJoin(right_entities, ", "));
}
}  // namespace intrinsic
