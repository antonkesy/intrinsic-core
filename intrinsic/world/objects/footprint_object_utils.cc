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

#include "intrinsic/world/objects/footprint_object_utils.h"

#include <optional>

#include "absl/functional/any_invocable.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/message.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_creation_utils.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/entity_search.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"

namespace intrinsic {
namespace object_world {
namespace {

absl::StatusOr<intrinsic_proto::skills::EntityReservation::SharingType>
ToEntitySharingType(
    intrinsic_proto::skills::ObjectWorldReservation::SharingType sharing_type) {
  switch (sharing_type) {
    case intrinsic_proto::skills::ObjectWorldReservation::READ:
      return intrinsic_proto::skills::EntityReservation::READ;
    case intrinsic_proto::skills::ObjectWorldReservation::WRITE:
      return intrinsic_proto::skills::EntityReservation::WRITE;
    default:
      return absl::InvalidArgumentError(absl::StrCat(
          "Unsupported enum value for "
          "intrinsic_proto::skills::ObjectWorldReservation::SharingType with "
          "value ",
          intrinsic_proto::skills::ObjectWorldReservation::SharingType_Name(
              sharing_type)));
  }
}

absl::StatusOr<WorldHashSet<EntityId>> GetEntityIds(
    const intrinsic_proto::skills::ObjectWorldReservation& object_reservation,
    const ObjectWorld& object_world) {
  switch (object_reservation.object_type_case()) {
    case intrinsic_proto::skills::ObjectWorldReservation::kObject: {
      INTR_ASSIGN_OR_RETURN(const WorldObject* object,
                            object_world.GetObject(WorldObjectName(
                                object_reservation.object().object_name())));
      WorldHashSet<EntityId> result;
      // TODO: b/326692169 -- We are ignoring frames here to preserve backwards
      // compatibility, but it's not clear that we should. Also, some frames are
      // special, like the sensor and flange frames. Should these be included?
      for (const AttachmentEntityId& member_entity_id :
           object->GetEntityIds()) {
        INTR_ASSIGN_OR_RETURN(
            const WorldEntity* member_entity,
            object_world.GetEntityWorld().GetEntityById(member_entity_id));
        if (!IsFrameEntity(member_entity).value_or(true)) {
          result.insert(member_entity_id);
        }
      }
      if (object->GetCollectionEntity().has_value()) {
        result.insert(*object->GetCollectionEntity());
      }
      return result;
    }
    case intrinsic_proto::skills::ObjectWorldReservation::kFrame: {
      INTR_ASSIGN_OR_RETURN(
          const Frame* frame,
          object_world.GetFrame(
              WorldObjectName(object_reservation.frame().object_name()),
              FrameName(object_reservation.frame().frame_name())));
      return WorldHashSet<EntityId>{frame->GetEntityId()};
    }
    case intrinsic_proto::skills::ObjectWorldReservation::OBJECT_TYPE_NOT_SET:
      return absl::InvalidArgumentError(absl::StrCat(
          "ObjectWorldReservation has no 'object_type' set. Received {",
          google::protobuf::ShortFormat(object_reservation), "}."));
  }
}

// Reserve the parents in read mode.
//
// This function assumes that `permissions` is in a consistent state when
// called. This means that previously processed Objects should have already had
// their permissions propagated. This assumption allows us to use a stronger
// stopping criteria, resulting in less visits to each node.
void ReserveParentsWithReadAccess(
    const object_world::WorldObject* root,
    WorldHashMap<WorldObjectName,
                 intrinsic_proto::skills::ObjectWorldReservation::SharingType>&
        permissions) {
  if (root == nullptr) {
    return;
  }
  const WorldObject* parent = root->GetParent();
  while (parent != nullptr) {
    // If the parent has already been seen, we can stop iterating early. The
    // reason this works is because of the consistent state assumption; a marked
    // parent (read OR write) will have already been propagated upward as far as
    // possible.
    if (permissions.contains(parent->GetName())) {
      break;
    }

    permissions[parent->GetName()] =
        intrinsic_proto::skills::ObjectWorldReservation::READ;
    parent = parent->GetParent();
  }
}

// Reserves this object and all its (recursive) children objects.
//
// This function handles two cases: when we propagate a READ, and when we
// propagate a WRITE. They are mostly the same except for different stopping
// conditions. A READ permission doesn't propagate downward beyond this object.
//
// Note that this function only propagates through WorldObjects and not Frames.
// In the overall flow of the code, this is ok, because Frames must have a
// parent WorldObject, and the propagation of permissions in the entity world
// will cover the entities representing Frames.
void ReserveObjectAndChildren(
    intrinsic_proto::skills::ObjectWorldReservation::SharingType
        propagating_type,
    const object_world::WorldObject* root,
    WorldHashMap<WorldObjectName,
                 intrinsic_proto::skills::ObjectWorldReservation::SharingType>&
        permissions) {
  if (root == nullptr) {
    return;
  }
  auto current_permission_iter = permissions.find(root->GetName());
  if (current_permission_iter != permissions.end()) {
    intrinsic_proto::skills::ObjectWorldReservation::SharingType existing_type =
        current_permission_iter->second;
    if (propagating_type ==
        intrinsic_proto::skills::ObjectWorldReservation::READ) {
      // If we are propagating READs, we can stop as soon as we see any existing
      // permission (READ or WRITE).
      return;
    } else if (propagating_type ==
                   intrinsic_proto::skills::ObjectWorldReservation::WRITE &&
               existing_type ==
                   intrinsic_proto::skills::ObjectWorldReservation::WRITE) {
      // If we are propagating WRITEs and encounter a WRITE we can stop
      // iterating. However, we must propagate over READs.
      return;
    }
  }

  // If we reach this point, we are going to set the permission of root,
  // overwriting an existing value.
  permissions[root->GetName()] = propagating_type;

  // READs do not need to propagate to children, since reading the pose of an
  // object doesn't imply reading the pose of the children. WRITEs do need to
  // propagate.
  if (propagating_type ==
      intrinsic_proto::skills::ObjectWorldReservation::READ) {
    return;
  }
  for (const WorldObject* child : root->GetChildren()) {
    ReserveObjectAndChildren(propagating_type, child, permissions);
  }
}

}  // namespace

absl::StatusOr<intrinsic_proto::skills::Footprint>
ConvertObjectFootprintToEntityFootprint(
    const intrinsic_proto::skills::Footprint& footprint,
    absl::AnyInvocable<absl::StatusOr<const ObjectWorld*>()>&
        object_world_provider) {
  if (footprint.entity_size() > 0 && footprint.object_reservation_size() > 0) {
    return absl::FailedPreconditionError(
        "Footprint may only contain one type of world resource but the given "
        "footprint contains both entity as well as object resources.");
  }

  if (footprint.object_reservation_size() == 0) {
    // Footprint already is entity-based or does not contain any world resources
    // -> nothing to do.
    return footprint;
  }

  INTR_ASSIGN_OR_RETURN(const ObjectWorld* object_world,
                        object_world_provider());

  intrinsic_proto::skills::Footprint result = footprint;
  result.clear_object_reservation();

  for (const intrinsic_proto::skills::ObjectWorldReservation&
           object_reservation : footprint.object_reservation()) {
    INTR_ASSIGN_OR_RETURN(
        intrinsic_proto::skills::EntityReservation::SharingType
            entity_sharing_type,
        ToEntitySharingType(object_reservation.type()));
    INTR_ASSIGN_OR_RETURN(WorldHashSet<EntityId> entity_ids,
                          GetEntityIds(object_reservation, *object_world));

    for (EntityId entity_id : entity_ids) {
      intrinsic_proto::skills::EntityReservation* entity_reservation =
          result.add_entity();
      entity_reservation->set_type(entity_sharing_type);
      entity_reservation->mutable_entity()->mutable_by_id()->set_entity_id(
          entity_id.value());
    }
  }

  return result;
}

// Adds missing permissions for objects based on how they are attached in the
// world.
absl::StatusOr<intrinsic_proto::skills::Footprint> AddMissingPermissions(
    const intrinsic_proto::skills::Footprint& footprint,
    absl::AnyInvocable<absl::StatusOr<const ObjectWorld*>()>&
        object_world_provider) {
  // Short-circuit to avoid creating an object world unless we are likely to use
  // it.
  if (footprint.skip_missing_permissions() ||
      footprint.object_reservation().empty()) {
    return footprint;
  }

  INTR_ASSIGN_OR_RETURN(const ObjectWorld* object_world,
                        object_world_provider());

  WorldHashMap<WorldObjectName,
               intrinsic_proto::skills::ObjectWorldReservation::SharingType>
      new_permissions;
  for (const auto& object_reservation : footprint.object_reservation()) {
    const WorldObject* object;
    if (object_reservation.has_object()) {
      INTR_ASSIGN_OR_RETURN(object,
                            object_world->GetObject(WorldObjectName(
                                object_reservation.object().object_name())));
      ReserveParentsWithReadAccess(object, new_permissions);
      ReserveObjectAndChildren(object_reservation.type(), object,
                               new_permissions);
    } else if (object_reservation.has_frame()) {
      INTR_ASSIGN_OR_RETURN(object,
                            object_world->GetObject(WorldObjectName(
                                object_reservation.frame().object_name())));
      ReserveParentsWithReadAccess(object, new_permissions);
      // Only read permission is required for the object of the frame.
      ReserveObjectAndChildren(
          intrinsic_proto::skills::ObjectWorldReservation::READ, object,
          new_permissions);
    } else {
      return absl::UnimplementedError(
          absl::StrCat("Unsupported object resource type: ",
                       object_reservation.object_type_case()));
    }
  }

  // Now we just need to rebuild the object_reservations_.
  intrinsic_proto::skills::Footprint new_footprint = footprint;
  new_footprint.clear_object_reservation();
  for (const auto& [object_name, sharing_type] : new_permissions) {
    intrinsic_proto::skills::ObjectWorldReservation* new_resource =
        new_footprint.add_object_reservation();
    new_resource->mutable_object()->set_object_name(object_name.value());
    new_resource->set_type(sharing_type);
  }
  // We need to separately copy over the non-object resources, since the above
  // traversal only covers WorldObjects.
  for (const auto& original_resource : footprint.object_reservation()) {
    if (!original_resource.has_object()) {
      *new_footprint.add_object_reservation() = original_resource;
    }
  }
  return new_footprint;
}

}  // namespace object_world
}  // namespace intrinsic
