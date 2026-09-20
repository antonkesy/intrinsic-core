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

#include "intrinsic/world/aspects/entity_grouping.h"

#include <string>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/grouping.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {

namespace {

WorldHashSet<std::string> GetAllLocalNames(
    const WorldHashSet<PhysicalEntityId>& ids,
    const entity_aspect_world_details::EntityWorld& world) {
  WorldHashSet<std::string> display_names;
  for (const auto& id : ids) {
    auto entity_or = world.GetEntityById(id);
    if (!entity_or.ok()) {
      display_names.insert(
          absl::StrCat("Could not find entity ", id.value(), " in the world."));
      continue;
    }
    display_names.emplace((*entity_or)->GetLocalName());
  }
  return display_names;
}

}  // namespace

namespace entity_grouping_details {

EntityGrouping::EntityGrouping(entity_aspect_world_details::EntityWorld* world)
    : world_(world) {}

void EntityGrouping::UpdateEntityWorld(
    entity_aspect_world_details::EntityWorld* world) {
  world_ = world;
}

absl::Status EntityGrouping::AddGroup(
    GroupId group_id, const WorldHashSet<PhysicalEntityId>& object_ids) {
  INTR_RETURN_IF_ERROR(world_->AddGroupId(group_id));
  LabelId label_id(group_id.value());
  for (auto object_id : object_ids) {
    INTR_ASSIGN_OR_RETURN(auto* entity, world_->GetEntityById(object_id));
    INTR_RETURN_IF_ERROR(entity->AddLabels({label_id}));
  }
  return absl::OkStatus();
}

void EntityGrouping::RemoveGroup(GroupId group_id) {
  absl::Status status = world_->RemoveGroupId(group_id);
  if (!status.ok()) {
    LOG(ERROR) << "RemoveGroupId(\"" << group_id.value()
               << "\") failed: " << status;
  }
}

bool EntityGrouping::CheckGroupExists(GroupId group_id) const {
  return world_->GetGroupIds().contains(group_id);
}

WorldHashSet<GroupId> EntityGrouping::GetGroupIds() const {
  const auto& group_ids = world_->GetGroupIds();
  return WorldHashSet<GroupId>(group_ids.begin(), group_ids.end());
}

WorldHashSet<PhysicalEntityId> EntityGrouping::GetObjectIds(
    GroupId group_id) const {
  // Replicates original implementation's behavior of crashing if the group
  // does not exist.
  CHECK(world_->GetGroupIds().contains(group_id))
      << "Failed to find group id: " << group_id;
  WorldHashSet<PhysicalEntityId> ret;
  for (auto id : world_->FindByLabel(LabelId(group_id.value()))) {
    ret.emplace(id);
  }
  return ret;
}

WorldHashSet<PhysicalEntityId> EntityGrouping::GetPhysicalObjectsForGroup(
    GroupId group_id, const LabelId& label_id) const {
  // Replicates original implementation's behavior of crashing if the group
  // does not exist.
  CHECK(world_->GetGroupIds().contains(group_id))
      << "Failed to find group id: " << group_id;
  WorldHashSet<PhysicalEntityId> ret;
  auto label_to_id_set = world_->GetLabelsMap();
  auto group_iter = label_to_id_set.find(LabelId(group_id.value()));
  auto label_iter = label_to_id_set.find(label_id);
  if (group_iter == label_to_id_set.end() ||
      label_iter == label_to_id_set.end()) {
    return ret;
  }

  for (auto id : group_iter->second) {
    if (label_iter->second.contains(id)) {
      ret.emplace(id);
    }
  }
  return ret;
}

PhysicalEntityId EntityGrouping::GetSingleObject(
    GroupId group_id, const LabelId& label_id) const {
  auto ids = GetPhysicalObjectsForGroup(group_id, label_id);
  CHECK_EQ(1, ids.size()) << "expected 1 entity labeled with \""
                          << group_id.value() << "\" and \"" << label_id.value()
                          << "\", got " << ids.size() << ", which are: "
                          << absl::StrJoin(GetAllLocalNames(ids, *world_),
                                           ", ");
  return *ids.begin();
}

}  // namespace entity_grouping_details
}  // namespace intrinsic
