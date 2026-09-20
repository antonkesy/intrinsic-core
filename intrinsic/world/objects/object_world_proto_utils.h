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

#ifndef INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_PROTO_UTILS_H_
#define INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_PROTO_UTILS_H_

// Contains helpers that interface ObjectWorldService protos with an
// ObjectWorld.

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/proto/motion_target.pb.h"
#include "intrinsic/skills/proto/motion_targets.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/entity_search.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"

namespace intrinsic {
namespace object_world {

absl::StatusOr<WorldObject*> GetObjectByReference(
    ObjectWorld& world,
    const ::intrinsic_proto::world::ObjectReference& reference);

absl::StatusOr<const WorldObject*> GetObjectByReference(
    const ObjectWorld& world,
    const ::intrinsic_proto::world::ObjectReference& reference);

absl::StatusOr<KinematicObject*> GetKinematicObjectByReference(
    ObjectWorld& world,
    const ::intrinsic_proto::world::ObjectReference& reference);

absl::StatusOr<const KinematicObject*> GetKinematicObjectByReference(
    const ObjectWorld& world,
    const ::intrinsic_proto::world::ObjectReference& reference);

absl::StatusOr<Frame*> GetFrameByReference(
    ObjectWorld& world,
    const ::intrinsic_proto::world::FrameReference& reference);

absl::StatusOr<const Frame*> GetFrameByReference(
    const ObjectWorld& world,
    const ::intrinsic_proto::world::FrameReference& reference);

absl::StatusOr<TransformNode*> GetTransformNodeByNameReference(
    ObjectWorld& world,
    const ::intrinsic_proto::world::TransformNodeReferenceByName& reference);

absl::StatusOr<const TransformNode*> GetTransformNodeByNameReference(
    const ObjectWorld& world,
    const ::intrinsic_proto::world::TransformNodeReferenceByName& reference);

absl::StatusOr<TransformNode*> GetTransformNodeByReference(
    ObjectWorld& world,
    const ::intrinsic_proto::world::TransformNodeReference& reference);

absl::StatusOr<const TransformNode*> GetTransformNodeByReference(
    const ObjectWorld& world,
    const ::intrinsic_proto::world::TransformNodeReference& reference);

absl::StatusOr<AttachmentEntityId> GetEntityIdByReference(
    const ObjectWorld& world,
    const ::intrinsic_proto::world::EntityReference& reference);

// Returns an error if the entity matching the given criteria cannot be mapped
// to an object, i.e., if the entity is not the root entity of an object or a
// collection entity representing an object.
absl::StatusOr<intrinsic_proto::world::ObjectReference> ToObjectReference(
    const intrinsic_proto::world::EntitySearchCriteria& criteria,
    const ObjectWorld& object_world);

// Returns an error if the entity matching the given criteria cannot be mapped
// to a frame, e.g., if the entity is not an attachment-only entity represented
// by a frame.
absl::StatusOr<intrinsic_proto::world::FrameReference> ToFrameReference(
    const intrinsic_proto::world::EntitySearchCriteria& criteria,
    const ObjectWorld& object_world);

// Returns an error if the entity matching the given criteria cannot be mapped
// to any frame or object, e.g., if the entity is a non-root entity of an
// object.
absl::StatusOr<intrinsic_proto::world::TransformNodeReference>
ToTransformNodeReference(
    const intrinsic_proto::world::EntitySearchCriteria& criteria,
    const ObjectWorld& object_world);

absl::StatusOr<intrinsic_proto::world::EntitySearchCriteria>
ToEntitySearchCriteria(
    const intrinsic_proto::world::TransformNodeReference& ref,
    const ObjectWorld& object_world);

absl::StatusOr<intrinsic_proto::world::EntitySearchCriteria>
ToEntitySearchCriteria(const intrinsic_proto::world::ObjectReference& ref,
                       const ObjectWorld& object_world);

absl::StatusOr<intrinsic_proto::world::EntitySearchCriteria>
ToEntitySearchCriteria(const intrinsic_proto::world::FrameReference& ref,
                       const ObjectWorld& object_world);

absl::StatusOr<intrinsic_proto::skills::CartesianMotionTarget>
ToEntityBasedCartesianMotionTarget(
    const intrinsic_proto::motion_planning::CartesianMotionTarget& target,
    const ObjectWorld& object_world);

// Returns true if the two object references point to the same object reference.
bool AreTheSame(const intrinsic_proto::world::ObjectReference& a,
                const intrinsic_proto::world::ObjectReference& b);

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_PROTO_UTILS_H_
