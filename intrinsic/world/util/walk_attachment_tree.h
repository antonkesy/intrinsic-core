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

#ifndef INTRINSIC_WORLD_UTIL_WALK_ATTACHMENT_TREE_H_
#define INTRINSIC_WORLD_UTIL_WALK_ATTACHMENT_TREE_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

using AttachmentGraph =
    WorldHashMap<AttachmentEntityId, std::vector<AttachmentEntityId>>;

// Abstract base class for attachment tree handler.
//
// Any walker implementation will call the client's OnStartEntity() and
// OnEndEntity() once for each entity. The OnStartEntity() call will occur
// before the OnEndEntity() call, and if the client ever returns a non-OK
// status, the walk ends
//
// The exact traversal order is defined by walker implementation.
class AttachmentTreeHandler {
 public:
  virtual ~AttachmentTreeHandler() = default;
  virtual absl::Status OnStartEntity(AttachmentEntityId handle,
                                     const WorldEntity& entity) = 0;
  virtual absl::Status OnEndEntity(AttachmentEntityId handle,
                                   const WorldEntity& entity) = 0;
};

// The directed attachment graph where the key is id of the parent and the
// value vector contains the ids of children. Every object with an attachment
// component will appear in the map, even if it has no children, except for the
// root entity.
absl::StatusOr<AttachmentGraph> MakeAttachmentGraph(const World& world);

// Walks the attachment tree of `world` in a depth-first traversal order.
//
// OnStartEntity() is called when an entity is first encountered.
// OnEndEntity() is called when the attachment subtree under the current entity
// has been visited.
//
// `start` specifies the node that is the start of the traversal. Only the nodes
// below this one will have the callbacks called on them.
absl::Status WalkAttachmentTree(
    const World& world, AttachmentTreeHandler& handler,
    const AttachmentEntityId& start = kRootEntityId);

// Walks the tree upward from each entity and returns a vector with the parents.
// The vector is ordered such that the first entry is the parent of `id`, and
// the last entity should be the root entity.
absl::StatusOr<std::vector<AttachmentEntityId>> GetAllParentsOrdered(
    const World& world, AttachmentEntityId id);

// Walks the tree upward from each entity and returns a set with the parents.
// The set includes the specified entities themselves.
absl::StatusOr<WorldHashSet<AttachmentEntityId>> GetAllParents(
    const World& world, const WorldHashSet<AttachmentEntityId>& ids);

// Returns all entities attached to the given entities (including the entities
// themselves).
//
// If the entity does not have an Attachment component, it will be skipped.
absl::StatusOr<WorldHashSet<AttachmentEntityId>> GetAllAttachments(
    const World& world, const WorldHashSet<AttachmentEntityId>& ids);

// Walk the tree upward until one of the specified entities is found or the
// search hits the root entity. Entities are returned ordered so that the first
// is `start`, and the last entity will be either one of `stop_at_entities` or
// the root entity.
absl::StatusOr<std::vector<AttachmentEntityId>> WalkParentsUntilFound(
    const World& world, const AttachmentEntityId& start,
    const WorldHashSet<AttachmentEntityId>& stop_at_entities);

// Collects the entities rigidly attached to the specified one. Rigid attachment
// means that the path between them in the attachment graph does not contain any
// degrees of freedom. This function handles fixed joints and will include the
// entities on both sides of them.
//
// In our convention, the pose of a joint includes the transformation created by
// the dof value of the joint. Therefore, you should typically expect this
// function to return everything from (and including) the nearest ancestor joint
// downward to the next movable joints.
absl::StatusOr<WorldHashSet<AttachmentEntityId>> GetRigidlyAttachedEntities(
    const World& world, const AttachmentEntityId& entity_id);

// Gets only the children entities, those below `entity_id` and/or itself.
absl::StatusOr<WorldHashSet<AttachmentEntityId>>
GetRigidlyAttachedChildrenEntities(const World& world,
                                   const AttachmentEntityId& entity_id);

// Same as above but unions the results of multiple objects.
absl::StatusOr<WorldHashSet<AttachmentEntityId>> GetRigidAttachmentsOfChildren(
    const World& world, const WorldHashSet<AttachmentEntityId>& ids);

// Similar to GetRigidlyAttachedEntitiesMap but will group all the rigidly
// attached entities to each other under the 'root' dof that is moveable.
absl::StatusOr<
    WorldHashMap<AttachmentEntityId, WorldHashSet<AttachmentEntityId>>>
GetRigidlyAttachedEntitiesMap(const World& world,
                              const WorldHashSet<AttachmentEntityId>& ids);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_WALK_ATTACHMENT_TREE_H_
