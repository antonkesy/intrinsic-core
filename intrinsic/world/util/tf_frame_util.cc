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

#include "intrinsic/world/util/tf_frame_util.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/time/time.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto/transform_stamped.pb.h"
#include "intrinsic/math/tf2_convert_intrinsic.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {

namespace {

// Decomposed parts of a TF frame ID.
struct DecomposedTfFrameId {
  std::string asset_name;
  std::string object_name;
  std::string entity_name;
};

// Parses a TF frame ID into its components.
// Supported formats:
// - /asset_name/object_name/entity_name
// - /object_name/entity_name
// - entity_name
absl::StatusOr<DecomposedTfFrameId> DecomposeTfFrameId(
    absl::string_view frame_id) {
  if (frame_id == RootObjectName().value()) {
    return DecomposedTfFrameId{
        .object_name = RootObjectName().value(),
    };
  }

  std::vector<std::string> split =
      absl::StrSplit(frame_id, "/", absl::SkipWhitespace());
  if (split.size() == 3) {
    return DecomposedTfFrameId{
        .asset_name = split[0],
        .object_name = split[1],
        .entity_name = split[2],
    };
  }
  if (split.size() == 2) {
    return DecomposedTfFrameId{
        .object_name = split[0],
        .entity_name = split[1],
    };
  }
  if (split.size() == 1) {
    return DecomposedTfFrameId{
        .entity_name = split[0],
    };
  }
  return absl::InvalidArgumentError(absl::Substitute(
      "Cannot parse frame id $0, expect frame id to be in the format of "
      "asset_name/object_name/entity_name, object_name/entity_name, or "
      "entity_name",
      frame_id));
}
}  // namespace

absl::StatusOr<
    std::pair<const object_world::WorldObject*, world::ObjectEntityFilter>>
GetWorldObjectAndEntityFilterFromFrameId(const object_world::ObjectWorld& world,
                                         absl::string_view frame_id) {
  const object_world::WorldObject* object = nullptr;
  INTR_ASSIGN_OR_RETURN(auto frame_refs, DecomposeTfFrameId(frame_id));
  if (!frame_refs.asset_name.empty()) {
    INTR_ASSIGN_OR_RETURN(
        object, world.GetObjectForSceneObjectInstance(frame_refs.asset_name));
    if (!frame_refs.object_name.empty() &&
        object->GetName().value() != frame_refs.object_name) {
      return absl::InvalidArgumentError(absl::Substitute(
          "Object name $0 and asset instance name $1 doesn't match for frame "
          "id $2",
          frame_refs.object_name, frame_refs.asset_name, frame_id));
    }
  } else if (!frame_refs.object_name.empty()) {
    INTR_ASSIGN_OR_RETURN(
        object, world.GetObject(WorldObjectName(frame_refs.object_name)));
  } else {
    // If only entity name is provided, it could be an object name (referring to
    // the object's base) or an entity name under the root object.
    auto object_or = world.GetObject(WorldObjectName(frame_refs.entity_name));
    if (object_or.ok()) {
      return std::make_pair(*object_or,
                            world::ObjectEntityFilter::BaseEntity());
    }
    INTR_ASSIGN_OR_RETURN(object, world.GetObject(RootObjectId()));
  }

  INTR_RET_CHECK_NE(object, nullptr);

  auto entity_filter = frame_refs.entity_name.empty()
                           ? world::ObjectEntityFilter::BaseEntity()
                           : world::ObjectEntityFilter::FromEntityNames(
                                 {frame_refs.entity_name});
  return std::make_pair(object, entity_filter);
}

absl::StatusOr<std::pair<object_world::WorldObject*, world::ObjectEntityFilter>>
GetWorldObjectAndEntityFilterFromFrameId(object_world::ObjectWorld& world,
                                         absl::string_view frame_id) {
  INTR_ASSIGN_OR_RETURN(
      (auto [parent_object, parent_entity_filter]),
      GetWorldObjectAndEntityFilterFromFrameId(
          static_cast<const object_world::ObjectWorld&>(world), frame_id));
  return std::make_pair(const_cast<object_world::WorldObject*>(parent_object),
                        std::move(parent_entity_filter));
}

absl::StatusOr<absl::flat_hash_map<std::string, std::string>>
GetFrameIdByEntityIdFromWorldObjectAndEntityFilter(
    const world::WorldObject& world_object,
    const world::ObjectEntityFilter& entity_filter) {
  absl::flat_hash_map<std::string, std::string> entity_id_to_frame_id;

  if (world_object.Name() == RootObjectName()) {
    entity_id_to_frame_id.emplace(RootEntityId().value(),
                                  RootObjectName().value());
    return entity_id_to_frame_id;
  }

  std::string object_name = world_object.Name().value();
  const auto& entities = world_object.Proto().entities();
  if (entities.empty()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "WorldObject '", object_name,
        "' has no entities. GetFrameIdByEntityIdFromWorldObjectAndEntityFilter "
        "requires an object retrieved with ObjectView::FULL. Call "
        "GetFrameIdFromWorldObject instead."));
  }

  absl::string_view root_entity_id = world_object.Proto().root_entity_id();
  if (entity_filter.IncludesBaseEntity() &&
      !entity_filter.IncludesAllEntities()) {
    if (root_entity_id.empty()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "WorldObject '", object_name,
          "' does not specify a root_entity_id, so BaseEntity filter cannot be "
          "applied."));
    }
    if (entity_filter.EntityIds().empty() &&
        entity_filter.EntityNames().empty()) {
      entity_id_to_frame_id.emplace(root_entity_id, object_name);
      return entity_id_to_frame_id;
    }
  }

  for (const auto& [key, entity_proto] : entities) {
    absl::string_view id_str =
        !entity_proto.id().empty() ? entity_proto.id() : key;

    bool is_base_entity = !root_entity_id.empty() && id_str == root_entity_id;

    bool matches =
        entity_filter.IncludesAllEntities() ||
        (entity_filter.IncludesBaseEntity() && is_base_entity) ||
        entity_filter.EntityIds().contains(ObjectWorldResourceId(id_str)) ||
        entity_filter.EntityNames().contains(entity_proto.name());

    if (!matches) {
      continue;
    }

    if (entity_proto.name().empty()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "WorldObject '", object_name, "' has an entity (id: ", id_str,
          ") with an empty entity_name, which cannot be "
          "formatted as a TF frame ID."));
    }

    entity_id_to_frame_id.emplace(
        id_str, absl::StrCat(object_name, "/", entity_proto.name()));
  }

  return entity_id_to_frame_id;
}

absl::StatusOr<std::string> GetFrameIdFromWorldObject(
    const world::WorldObject& world_object) {
  return world_object.Name().value();
}

absl::Status UpdateWorldTransform(
    object_world::ObjectWorld& world,
    const intrinsic_proto::TransformStamped& transform_stamped,
    bool bypass_movable_check) {
  INTR_ASSIGN_OR_RETURN((auto [parent_object, parent_entity_filter]),
                        GetWorldObjectAndEntityFilterFromFrameId(
                            world, transform_stamped.header().frame_id()));
  INTR_ASSIGN_OR_RETURN((auto [updated_object, updated_entity_filter]),
                        GetWorldObjectAndEntityFilterFromFrameId(
                            world, transform_stamped.child_frame_id()));
  INTR_ASSIGN_OR_RETURN(absl::Time timestamp,
                        ToAbslTime(transform_stamped.header().stamp()));
  geometry_msgs::msg::Transform transform =
      tf2::toMsg(transform_stamped.transform());
  INTR_ASSIGN_OR_RETURN(Pose3d pose, tf2::fromMsg(transform));
  return updated_object->SetTransform(updated_entity_filter, parent_object,
                                      parent_entity_filter, updated_object,
                                      updated_entity_filter, pose, timestamp,
                                      bypass_movable_check);
}

absl::StatusOr<Pose3d> GetFrameTransform(const object_world::ObjectWorld& world,
                                         absl::string_view parent_frame_id,
                                         absl::string_view child_frame_id) {
  INTR_ASSIGN_OR_RETURN(
      (auto [parent_object, parent_entity_filter]),
      GetWorldObjectAndEntityFilterFromFrameId(world, parent_frame_id));
  INTR_ASSIGN_OR_RETURN(
      (auto [child_object, child_entity_filter]),
      GetWorldObjectAndEntityFilterFromFrameId(world, child_frame_id));

  return parent_object->GetTransform(parent_entity_filter, child_object,
                                     child_entity_filter);
}

}  // namespace intrinsic
