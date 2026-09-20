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

#include "intrinsic/world/objects/object_world_proto_utils.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/substitute.h"
#include "google/protobuf/message.h"
#include "intrinsic/math/proto/point.pb.h"
#include "intrinsic/math/proto/pose.pb.h"
#include "intrinsic/motion_planning/proto/motion_target.pb.h"
#include "intrinsic/skills/proto/motion_targets.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_creation_utils.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/simple_transform_node_visitor.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/entity_search.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/util/entity_search_util.h"

namespace intrinsic {
namespace object_world {

using ::intrinsic_proto::world::EntityReference;
using ::intrinsic_proto::world::FrameReference;
using ::intrinsic_proto::world::FrameReferenceByName;
using ::intrinsic_proto::world::ObjectReference;
using ::intrinsic_proto::world::TransformNodeReference;
using ::intrinsic_proto::world::TransformNodeReferenceByName;

absl::StatusOr<WorldObject*> GetObjectByReference(
    ObjectWorld& world, const ObjectReference& reference) {
  WorldObject* object = nullptr;
  switch (reference.object_reference_case()) {
    case ObjectReference::kId: {
      INTR_ASSIGN_OR_RETURN(
          object, world.GetObject(ObjectWorldResourceId(reference.id())),
          _ << " Unable to resolve ObjectReference with debug hint: \""
            << reference.debug_hint() << "\"");
      break;
    }
    case ObjectReference::kByName: {
      INTR_ASSIGN_OR_RETURN(
          object,
          world.GetObject(WorldObjectName(reference.by_name().object_name())),
          _ << " Unable to resolve ObjectReference with debug hint: \""
            << reference.debug_hint() << "\"");
      break;
    }
    case ObjectReference::OBJECT_REFERENCE_NOT_SET:
      return absl::InvalidArgumentError(
          "The ObjectReference in the request must be set.");
  }
  return object;
}

absl::StatusOr<const WorldObject*> GetObjectByReference(
    const ObjectWorld& world, const ObjectReference& reference) {
  // Delegate to non-const implementation.
  return GetObjectByReference(const_cast<ObjectWorld&>(world), reference);
}

absl::StatusOr<KinematicObject*> GetKinematicObjectByReference(
    ObjectWorld& world, const ObjectReference& reference) {
  KinematicObject* kinematic_object = nullptr;
  switch (reference.object_reference_case()) {
    case ObjectReference::kId: {
      INTR_ASSIGN_OR_RETURN(
          kinematic_object,
          world.GetKinematicObject(ObjectWorldResourceId(reference.id())),
          _ << " Unable to resolve ObjectReference with debug hint: \""
            << reference.debug_hint() << "\"");
      break;
    }
    case ObjectReference::kByName: {
      INTR_ASSIGN_OR_RETURN(
          kinematic_object,
          world.GetKinematicObject(
              WorldObjectName(reference.by_name().object_name())),
          _ << " Unable to resolve ObjectReference with debug hint: \""
            << reference.debug_hint() << "\"");
      break;
    }
    case ObjectReference::OBJECT_REFERENCE_NOT_SET:
      return absl::InvalidArgumentError(
          "The ObjectReference in the request must be set.");
  }
  return kinematic_object;
}

absl::StatusOr<const KinematicObject*> GetKinematicObjectByReference(
    const ObjectWorld& world, const ObjectReference& reference) {
  // Delegate to non-const implementation.
  return GetKinematicObjectByReference(const_cast<ObjectWorld&>(world),
                                       reference);
}

absl::StatusOr<Frame*> GetFrameByReference(ObjectWorld& world,
                                           const FrameReference& reference) {
  Frame* frame = nullptr;
  switch (reference.frame_reference_case()) {
    case FrameReference::kId: {
      INTR_ASSIGN_OR_RETURN(
          frame, world.GetFrame(ObjectWorldResourceId(reference.id())),
          _ << " Unable to resolve FrameReference with debug hint hint: \""
            << reference.debug_hint() << "\"");
      break;
    }
    case FrameReference::kByName: {
      INTR_ASSIGN_OR_RETURN(
          frame,
          world.GetFrame(WorldObjectName(reference.by_name().object_name()),
                         FrameName(reference.by_name().frame_name())),
          _ << " Unable to resolve FrameReference with debug hint hint: \""
            << reference.debug_hint() << "\"");
      break;
    }
    case FrameReference::FRAME_REFERENCE_NOT_SET:
      return absl::InvalidArgumentError(
          "The FrameReference in the request must be set.");
  }
  return frame;
}

absl::StatusOr<const Frame*> GetFrameByReference(
    const ObjectWorld& world, const FrameReference& reference) {
  // Delegate to non-const implementation.
  return GetFrameByReference(const_cast<ObjectWorld&>(world), reference);
}

absl::StatusOr<TransformNode*> GetTransformNodeByNameReference(
    ObjectWorld& world, const TransformNodeReferenceByName& reference) {
  switch (reference.transform_node_reference_by_name_case()) {
    case TransformNodeReferenceByName::kObject:
      return world.GetObject(WorldObjectName(reference.object().object_name()));
    case TransformNodeReferenceByName::kFrame:
      return world.GetFrame(WorldObjectName(reference.frame().object_name()),
                            FrameName(reference.frame().frame_name()));
    case TransformNodeReferenceByName::TRANSFORM_NODE_REFERENCE_BY_NAME_NOT_SET:
      return absl::InvalidArgumentError(
          "TransformNodeReference in the request must be set.");
  }
}

absl::StatusOr<const TransformNode*> GetTransformNodeByNameReference(
    const ObjectWorld& world, const TransformNodeReferenceByName& reference) {
  // Delegate to non-const implementation.
  return GetTransformNodeByNameReference(const_cast<ObjectWorld&>(world),
                                         reference);
}

absl::StatusOr<TransformNode*> GetTransformNodeByReference(
    ObjectWorld& world, const TransformNodeReference& reference) {
  TransformNode* transform_node = nullptr;
  switch (reference.transform_node_reference_case()) {
    case TransformNodeReference::kId: {
      INTR_ASSIGN_OR_RETURN(
          transform_node,
          world.GetTransformNode(ObjectWorldResourceId(reference.id())),
          _ << "Unable to resolve TransformNodeReference [id: "
            << absl::StrCat(reference.id()) << "]");
      break;
    }
    case TransformNodeReference::kByName: {
      INTR_ASSIGN_OR_RETURN(
          transform_node,
          GetTransformNodeByNameReference(world, reference.by_name()),
          _ << "Unable to resolve TransformNodeReference [id: "
            << absl::StrCat(reference.id()) << "]");
      break;
    }
    case TransformNodeReference::TRANSFORM_NODE_REFERENCE_NOT_SET:
      return absl::InvalidArgumentError(
          "TransformNodeReference in the request must be set.");
  }
  return transform_node;
}

absl::StatusOr<const TransformNode*> GetTransformNodeByReference(
    const ObjectWorld& world, const TransformNodeReference& reference) {
  // Delegate to non-const implementation.
  return GetTransformNodeByReference(const_cast<ObjectWorld&>(world),
                                     reference);
}

absl::StatusOr<AttachmentEntityId> GetEntityIdByReference(
    const ObjectWorld& world, const EntityReference& reference) {
  switch (reference.entity_reference_case()) {
    case EntityReference::kId: {
      INTR_ASSIGN_OR_RETURN(EntityId id,
                            ObjectWorldResourceIdToEntityId(
                                ObjectWorldResourceId(reference.id())));
      INTR_ASSIGN_OR_RETURN(const WorldEntity* entity,
                            world.GetEntityWorld().GetEntityById(id));
      if (!entity->HasComponent<AttachmentComponent>()) {
        return absl::InvalidArgumentError(
            absl::Substitute("The given entity reference \"$0\" is referencing "
                             "an internal entity of an unexpected type.",
                             reference.id()));
      }
      return AttachmentEntityId(id);
    }
    case EntityReference::ENTITY_REFERENCE_NOT_SET:
      return absl::InvalidArgumentError(
          "The EntityReference in the request must be set.");
  }
}

namespace {

absl::Status VisitTransformNodeMatchingCriteria(
    const intrinsic_proto::world::EntitySearchCriteria& criteria,
    const ObjectWorld& object_world, SimpleTransformNodeConstVisitor& visitor) {
  INTR_ASSIGN_OR_RETURN(
      EntityId id, GetSingleEntity(object_world.GetEntityWorld(), criteria));
  INTR_ASSIGN_OR_RETURN(const TransformNode* node,
                        object_world.GetTransformNodeByEntityId(id));
  return node->Accept(visitor);
}

}  // namespace

absl::StatusOr<ObjectReference> ToObjectReference(
    const intrinsic_proto::world::EntitySearchCriteria& criteria,
    const ObjectWorld& object_world) {
  ObjectReference result;
  SimpleTransformNodeConstVisitor visitor(
      [&](const Frame& frame) {
        return absl::InvalidArgumentError(absl::Substitute(
            "Given EntitySearchCriteria { $0 } cannot be mapped to an object, "
            "they map to the frame \"$1\" under object \"$2\"",
            google::protobuf::ShortFormat(criteria), frame.GetName().value(),
            frame.GetParent()->GetName().value()));
      },
      [&](const WorldObject& object) {
        result.mutable_by_name()->set_object_name(object.GetName().value());
        return absl::OkStatus();
      });
  INTR_RETURN_IF_ERROR(
      VisitTransformNodeMatchingCriteria(criteria, object_world, visitor));
  return result;
}

absl::StatusOr<FrameReference> ToFrameReference(
    const intrinsic_proto::world::EntitySearchCriteria& criteria,
    const ObjectWorld& object_world) {
  FrameReference result;
  SimpleTransformNodeConstVisitor visitor(
      [&](const Frame& frame) {
        result.mutable_by_name()->set_object_name(
            frame.GetParent()->GetName().value());
        result.mutable_by_name()->set_frame_name(frame.GetName().value());
        return absl::OkStatus();
      },
      [&](const WorldObject& object) {
        return absl::InvalidArgumentError(absl::Substitute(
            "Given EntitySearchCriteria { $0 } cannot be mapped to a frame, "
            "they map to the object \"$1\"",
            google::protobuf::ShortFormat(criteria), object.GetName().value()));
      });
  INTR_RETURN_IF_ERROR(
      VisitTransformNodeMatchingCriteria(criteria, object_world, visitor));
  return result;
}

absl::StatusOr<TransformNodeReference> ToTransformNodeReference(
    const intrinsic_proto::world::EntitySearchCriteria& criteria,
    const ObjectWorld& object_world) {
  TransformNodeReference result;
  SimpleTransformNodeConstVisitor visitor(
      [&](const Frame& frame) {
        FrameReferenceByName* ref = result.mutable_by_name()->mutable_frame();
        ref->set_object_name(frame.GetParent()->GetName().value());
        ref->set_frame_name(frame.GetName().value());
        return absl::OkStatus();
      },
      [&](const WorldObject& object) {
        result.mutable_by_name()->mutable_object()->set_object_name(
            object.GetName().value());
        return absl::OkStatus();
      });
  INTR_RETURN_IF_ERROR(
      VisitTransformNodeMatchingCriteria(criteria, object_world, visitor));
  return result;
}

namespace {

absl::StatusOr<intrinsic_proto::world::EntitySearchCriteria>
ToEntitySearchCriteriaImpl(const TransformNode& node) {
  INTR_ASSIGN_OR_RETURN(AttachmentEntityId entity_id,
                        node.GetTransformOriginEntityId());
  intrinsic_proto::world::EntitySearchCriteria result;
  result.mutable_by_id()->set_entity_id(entity_id.value());
  return result;
}

}  // namespace

absl::StatusOr<intrinsic_proto::world::EntitySearchCriteria>
ToEntitySearchCriteria(const TransformNodeReference& ref,
                       const ObjectWorld& object_world) {
  INTR_ASSIGN_OR_RETURN(const TransformNode* node,
                        GetTransformNodeByReference(object_world, ref));
  return ToEntitySearchCriteriaImpl(*node);
}

absl::StatusOr<intrinsic_proto::world::EntitySearchCriteria>
ToEntitySearchCriteria(const ObjectReference& ref,
                       const ObjectWorld& object_world) {
  INTR_ASSIGN_OR_RETURN(const WorldObject* object,
                        GetObjectByReference(object_world, ref));
  return ToEntitySearchCriteriaImpl(*object);
}

absl::StatusOr<intrinsic_proto::world::EntitySearchCriteria>
ToEntitySearchCriteria(const FrameReference& ref,
                       const ObjectWorld& object_world) {
  INTR_ASSIGN_OR_RETURN(const Frame* frame,
                        GetFrameByReference(object_world, ref));
  return ToEntitySearchCriteriaImpl(*frame);
}

absl::StatusOr<intrinsic_proto::skills::CartesianMotionTarget>
ToEntityBasedCartesianMotionTarget(
    const intrinsic_proto::motion_planning::CartesianMotionTarget& target,
    const ObjectWorld& object_world) {
  intrinsic_proto::skills::CartesianMotionTarget result;
  INTR_ASSIGN_OR_RETURN(*result.mutable_tool(),
                        ToEntitySearchCriteria(target.tool(), object_world));
  INTR_ASSIGN_OR_RETURN(*result.mutable_frame(),
                        ToEntitySearchCriteria(target.frame(), object_world));
  if (target.has_offset()) {
    *result.mutable_offset() = target.offset();
  }
  return result;
}

bool AreTheSame(const ObjectReference& a, const ObjectReference& b) {
  if (a.has_id() && b.has_id() && a.id() == b.id()) {
    return true;
  }

  if (a.has_by_name() && b.has_by_name() &&
      a.by_name().object_name() == b.by_name().object_name()) {
    return true;
  }
  return false;
}

}  // namespace object_world
}  // namespace intrinsic
