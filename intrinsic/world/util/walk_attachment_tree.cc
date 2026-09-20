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

#include "intrinsic/world/util/walk_attachment_tree.h"

#include <deque>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/util/aggregate_type.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace {

// Returns true if the entity has a kinematic component and motion type is NOT
// fixed.
bool IsMovableDof(const World& world, const AttachmentEntityId& entity_id) {
  const auto* kinematic =
      world.GetComponentByEntityId<KinematicsComponent>(entity_id).value_or(
          nullptr);
  return kinematic != nullptr &&
         kinematic->GetMotionType() !=
             intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_FIXED;
}

}  // namespace

// The directed attachment graph where the key is id of the parent and the
// value vector contains the ids of children. Every object with an attachment
// component will appear in the map, even if it has no children, except for the
// root entity.
absl::StatusOr<AttachmentGraph> MakeAttachmentGraph(const World& world) {
  AttachmentGraph attachment_graph;
  for (const AttachmentEntityId& attachment_id :
       world.GetTypedEntityIds<AttachmentComponentType>()) {
    // Make sure every attachment id is represented in the map (even if it has
    // no children).
    attachment_graph.insert({attachment_id, {}});

    // Skip root entity as it has no parent.
    if (attachment_id == kRootEntityId) {
      continue;
    }
    INTR_ASSIGN_OR_RETURN(const auto* entity,
                          world.GetEntityById(attachment_id));
    INTR_ASSIGN_OR_RETURN(const auto* attachment,
                          entity->GetComponent<AttachmentComponent>());
    attachment_graph[attachment->GetParentId()].push_back(attachment_id);
  }
  return attachment_graph;
}

absl::Status WalkAttachmentTree(const World& world,
                                AttachmentTreeHandler& handler,
                                const AttachmentEntityId& start) {
  INTR_ASSIGN_OR_RETURN(auto attachment_graph, MakeAttachmentGraph(world));

  WorldHashSet<AttachmentEntityId> visited;
  std::vector<AttachmentEntityId> stack = {start};
  while (!stack.empty()) {
    AttachmentEntityId attachment_id = stack.back();
    INTR_ASSIGN_OR_RETURN(const auto* entity,
                          world.GetEntityById(attachment_id));
    stack.pop_back();
    // The attachment graph is always a single tree. Therefore, we only
    // encounter visited attachment id because we deliberately put each id into
    // the stack twice to support the end call.
    if (visited.contains(attachment_id)) {
      INTR_RETURN_IF_ERROR(handler.OnEndEntity(attachment_id, *entity));
    } else {
      INTR_RETURN_IF_ERROR(handler.OnStartEntity(attachment_id, *entity));
      visited.insert(attachment_id);
      stack.push_back(attachment_id);
      stack.insert(stack.end(), attachment_graph[attachment_id].begin(),
                   attachment_graph[attachment_id].end());
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<std::vector<AttachmentEntityId>> GetAllParentsOrdered(
    const World& world, AttachmentEntityId id) {
  std::vector<AttachmentEntityId> output;
  while (id != kRootEntityId) {
    INTR_ASSIGN_OR_RETURN(
        const auto* attachment,
        world.GetComponentByEntityId<AttachmentComponent>(id));
    output.push_back(attachment->GetParentId());
    id = attachment->GetParentId();
  }

  return output;
}

absl::StatusOr<WorldHashSet<AttachmentEntityId>> GetAllParents(
    const World& world, const WorldHashSet<AttachmentEntityId>& ids) {
  std::deque<AttachmentEntityId> queue(ids.begin(), ids.end());
  WorldHashSet<AttachmentEntityId> output;
  while (!queue.empty()) {
    AttachmentEntityId current = queue.front();
    queue.pop_front();

    output.insert(current);

    if (current == kRootEntityId) {
      continue;
    }

    INTR_ASSIGN_OR_RETURN(const auto* entity, world.GetEntityById(current));
    INTR_ASSIGN_OR_RETURN(const auto* attachment,
                          entity->GetComponent<AttachmentComponent>());
    queue.push_back(attachment->GetParentId());
  }

  return output;
}

absl::StatusOr<WorldHashSet<AttachmentEntityId>> GetAllAttachments(
    const World& world, const WorldHashSet<AttachmentEntityId>& ids) {
  INTR_ASSIGN_OR_RETURN(auto attachment_graph, MakeAttachmentGraph(world));

  std::deque<AttachmentEntityId> queue(ids.begin(), ids.end());
  WorldHashSet<AttachmentEntityId> output;
  // Now walk attachments.
  while (!queue.empty()) {
    AttachmentEntityId current = queue.front();
    queue.pop_front();

    if (!output.insert(current).second) {
      // This node has already been expanded once.
      continue;
    }

    // The map should have all attachment entities (and the root entity is
    // excluded above) so the 'at' is ok.
    for (const auto& child : attachment_graph.at(current)) {
      queue.push_back(child);
    }
  }

  return output;
}

absl::StatusOr<std::vector<AttachmentEntityId>> WalkParentsUntilFound(
    const World& world, const AttachmentEntityId& start,
    const WorldHashSet<AttachmentEntityId>& stop_at_entities) {
  std::vector<AttachmentEntityId> out{start};
  AttachmentEntityId current = start;
  while (current != kRootEntityId && !stop_at_entities.contains(current)) {
    INTR_ASSIGN_OR_RETURN(const auto* entity, world.GetEntityById(current));
    INTR_ASSIGN_OR_RETURN(const auto* attachment,
                          entity->GetComponent<AttachmentComponent>());
    AttachmentEntityId parent = attachment->GetParentId();
    out.push_back(parent);
    current = parent;
  }
  return out;
}

absl::StatusOr<WorldHashSet<AttachmentEntityId>> GetRigidlyAttachedEntities(
    const World& world, const AttachmentEntityId& entity_id) {
  // Upward walk until we find the first node that is not a fixed joint.
  AttachmentEntityId current = entity_id;
  while (current != kRootEntityId && !IsMovableDof(world, current)) {
    INTR_ASSIGN_OR_RETURN(const auto* entity, world.GetEntityById(current));
    INTR_ASSIGN_OR_RETURN(const auto* attachment,
                          entity->GetComponent<AttachmentComponent>());
    AttachmentEntityId parent = attachment->GetParentId();
    current = parent;
  }

  return GetRigidlyAttachedChildrenEntities(world, current);
}

namespace {

// Separates the creation of the attachment graph so the function can be used in
// a nested fashion without inefficiently creating the map each time.
absl::StatusOr<WorldHashSet<AttachmentEntityId>>
GetRigidlyAttachedChildrenEntitiesImpl(
    const World& world, const AttachmentEntityId& entity_id,
    const AttachmentGraph& attachment_graph) {
  WorldHashSet<AttachmentEntityId> attached_entities;
  attached_entities.insert(entity_id);

  // Breadth first search.
  // Invariant: items in the queue are confirmed to be rigidly attached to the
  // specified entity, and have already added to the attached_entities list.
  std::deque<AttachmentEntityId> queue{entity_id};
  while (!queue.empty()) {
    AttachmentEntityId current = queue.front();
    queue.pop_front();

    for (const auto& child : attachment_graph.at(current)) {
      if (IsMovableDof(world, child)) {
        continue;
      }

      attached_entities.insert(child);
      queue.push_back(child);
    }
  }
  return attached_entities;
}

// Separates the creation of the attachment graph so the function can be used in
// a nested fashion without inefficiently creating the map each time.
absl::StatusOr<WorldHashSet<AttachmentEntityId>>
GetRigidlyAttachedEntitiesMapImpl(
    const World& world, const AttachmentEntityId& entity_id,
    const WorldHashSet<AttachmentEntityId>& break_ids,
    const AttachmentGraph& attachment_graph) {
  WorldHashSet<AttachmentEntityId> attached_entities;
  attached_entities.insert(entity_id);

  // Breadth first search.
  // Invariant: items in the queue are confirmed to be rigidly attached to the
  // specified entity, and have already added to the attached_entities list.
  std::deque<AttachmentEntityId> queue{entity_id};
  while (!queue.empty()) {
    AttachmentEntityId current = queue.front();
    queue.pop_front();

    for (const auto& child : attachment_graph.at(current)) {
      if (break_ids.contains(child)) {
        continue;
      }

      attached_entities.insert(child);
      queue.push_back(child);
    }
  }

  return attached_entities;
}

}  // namespace

absl::StatusOr<WorldHashSet<AttachmentEntityId>>
GetRigidlyAttachedChildrenEntities(const World& world,
                                   const AttachmentEntityId& entity_id) {
  INTR_ASSIGN_OR_RETURN(auto attachment_graph, MakeAttachmentGraph(world));
  return GetRigidlyAttachedChildrenEntitiesImpl(world, entity_id,
                                                attachment_graph);
}

absl::StatusOr<WorldHashSet<AttachmentEntityId>> GetRigidAttachmentsOfChildren(
    const World& world, const WorldHashSet<AttachmentEntityId>& ids) {
  INTR_ASSIGN_OR_RETURN(auto attachment_graph, MakeAttachmentGraph(world));
  WorldHashSet<AttachmentEntityId> out;
  for (const auto& id : ids) {
    auto children = attachment_graph.find(id);
    CHECK(children != attachment_graph.end());
    for (const auto& child : children->second) {
      INTR_ASSIGN_OR_RETURN(auto rigid_children,
                            GetRigidlyAttachedChildrenEntitiesImpl(
                                world, child, attachment_graph));
      out.insert(rigid_children.begin(), rigid_children.end());
    }
  }
  return out;
}

absl::StatusOr<
    WorldHashMap<AttachmentEntityId, WorldHashSet<AttachmentEntityId>>>
GetRigidlyAttachedEntitiesMap(const World& world,
                              const WorldHashSet<AttachmentEntityId>& ids) {
  INTR_ASSIGN_OR_RETURN(auto attachment_graph, MakeAttachmentGraph(world));
  WorldHashMap<AttachmentEntityId, WorldHashSet<AttachmentEntityId>> out;
  for (const auto& id : ids) {
    auto children = attachment_graph.find(id);
    CHECK(children != attachment_graph.end());
    for (const auto& child : children->second) {
      INTR_ASSIGN_OR_RETURN(auto rigid_children,
                            GetRigidlyAttachedEntitiesMapImpl(
                                world, child, ids, attachment_graph));
      if (!ids.contains(child)) {
        out[id].insert(child);
      }

      out[id].insert(rigid_children.begin(), rigid_children.end());
    }
  }
  return out;
}

}  // namespace intrinsic
