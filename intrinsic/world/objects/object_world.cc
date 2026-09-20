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

#include "intrinsic/world/objects/object_world.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <queue>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "intrinsic/geometry/api/geometry_options.h"
#include "intrinsic/geometry/api/io.h"
#include "intrinsic/geometry/storage/geometry_deserializer.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/component/physics_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/defaulting_world_object_visitor.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world_data.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/geometry_component.pb.h"
#include "intrinsic/world/proto/physics_component.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {

ObjectWorld::ObjectWorld(std::unique_ptr<ObjectWorldData> data)
    : data_(std::move(data)) {}

absl::StatusOr<WorldObject*> ObjectWorld::GetObject(
    const ObjectWorldResourceId& object_id) {
  const WorldHashMap<ObjectWorldResourceId, std::unique_ptr<WorldObject>>&
      objects = data_->GetObjectsById();

  auto it = objects.find(object_id);
  if (it == objects.end()) {
    return absl::NotFoundError(absl::StrCat(
        "Object with id \"", object_id.value(), "\" does not exist."));
  }
  return it->second.get();
}

absl::StatusOr<const WorldObject*> ObjectWorld::GetObject(
    const ObjectWorldResourceId& object_id) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(WorldObject * result,
                        const_cast<ObjectWorld*>(this)->GetObject(object_id));
  return result;
}

absl::StatusOr<WorldObject*> ObjectWorld::GetObject(
    const WorldObjectName& object_name) {
  const WorldHashMap<WorldObjectName, WorldHashSet<WorldObject*>>& objects =
      data_->GetObjectsByName();
  auto objects_with_name_itr = objects.find(object_name);
  if (objects_with_name_itr == objects.end()) {
    return absl::NotFoundError(absl::StrCat("No object with name \"",
                                            object_name.value(), "\" exists."));
  }
  const WorldHashSet<WorldObject*>& objects_with_name =
      objects_with_name_itr->second;

  const auto it = absl::c_find_if(objects_with_name, [](auto* object_ptr) {
    return object_ptr->NameIsGlobalAlias().value();
  });

  if (it == objects_with_name.end()) {
    auto error = intrinsic::NotFoundErrorBuilder()
                 << absl::StrCat("Object with globally unique name \"",
                                 object_name.value(), "\" does not exist.");
    const int name_count = objects_with_name.size();
    if (name_count == 0) {
      error << " No object with name \"" << object_name.value() << "\" exists.";
    } else {
      error << " Found " << name_count << " objects with name "
            << object_name.value()
            << ", but none of them has the 'name_is_global_alias' option "
               "enabled.";
    }
    return error;
  }

  return *it;
}

absl::StatusOr<const WorldObject*> ObjectWorld::GetObject(
    const WorldObjectName& object_name) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(WorldObject * result,
                        const_cast<ObjectWorld*>(this)->GetObject(object_name));
  return result;
}

absl::StatusOr<WorldObject*> ObjectWorld::GetObjectForSceneObjectInstance(
    absl::string_view instance_name) {
  for (auto& [_, object] : data_->GetObjectsById()) {
    INTR_ASSIGN_OR_RETURN(std::optional<absl::string_view> resource_name,
                          object->GetResourceName());
    if (resource_name == instance_name) {
      return object.get();
    }

    // For compatibility with older worlds we also support the equipment
    // component being defined on any member entity of the object's
    // collection. This case triggers a warning in ObjectWorld::CreateView.
    for (AttachmentEntityId member_id : object->GetEntityIds()) {
      INTR_ASSIGN_OR_RETURN(const WorldEntity* entity,
                            GetEntityWorld().GetEntityById(member_id));
      if (entity->GetLocalName() == instance_name) {
        return object.get();
      }
    }
  }

  return absl::NotFoundError(absl::StrCat(
      "Could not find an object associated with the instance name \"",
      instance_name, "\""));
}

absl::StatusOr<const WorldObject*> ObjectWorld::GetObjectForSceneObjectInstance(
    absl::string_view instance_name) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      WorldObject * result,
      const_cast<ObjectWorld*>(this)->GetObjectForSceneObjectInstance(
          instance_name));
  return result;
}

absl::StatusOr<WorldObject*> ObjectWorld::GetObjectByMemberEntityId(
    AttachmentEntityId entity_id) {
  const auto& objects_by_entity_id = data_->GetObjectsByEntityId();
  auto it = objects_by_entity_id.find(entity_id);
  if (it != objects_by_entity_id.end()) {
    return it->second;
  }

  return absl::NotFoundError(
      absl::Substitute("Could not find an object that contains the entity with "
                       "id $0 as a member.",
                       entity_id.value()));
}

absl::StatusOr<const WorldObject*> ObjectWorld::GetObjectByMemberEntityId(
    AttachmentEntityId entity_id) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      WorldObject * result,
      const_cast<ObjectWorld*>(this)->GetObjectByMemberEntityId(entity_id));
  return result;
}

absl::StatusOr<WorldObject*> ObjectWorld::GetObjectByFullPath(
    const std::vector<WorldObjectName>& names) {
  INTR_ASSIGN_OR_RETURN(WorldObject * current_object,
                        GetObject(RootObjectId()));
  for (const WorldObjectName& name : names) {
    std::vector<WorldObject*> children = current_object->GetChildren();
    auto child_it =
        std::find_if(children.begin(), children.end(),
                     [&name](WorldObject* o) { return o->GetName() == name; });
    if (child_it != children.end()) {
      current_object = *child_it;
    } else {
      return absl::NotFoundError(
          absl::Substitute("Could not find an object with the full path \"$0\"",
                           absl::StrJoin(names, ".", absl::StreamFormatter())));
    }
  }
  return current_object;
}

absl::StatusOr<const WorldObject*> ObjectWorld::GetObjectByFullPath(
    const std::vector<WorldObjectName>& names) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      const WorldObject* result,
      const_cast<ObjectWorld*>(this)->GetObjectByFullPath(names));
  return result;
}

struct ToRobotVisitor : public DefaultingWorldObjectVisitor {
  absl::Status DefaultVisit(WorldObject& object) override {
    return absl::InvalidArgumentError(
        absl::StrCat("Found object with name \"", object.GetName().value(),
                     "\" but it is not a kinematic object."));
  }
  absl::Status Visit(KinematicObject& kinematic_object) override {
    result = &kinematic_object;
    return absl::OkStatus();
  }
  KinematicObject* result;
};

absl::StatusOr<KinematicObject*> ObjectWorld::GetKinematicObject(
    const ObjectWorldResourceId& object_id) {
  INTR_ASSIGN_OR_RETURN(WorldObject * object, GetObject(object_id));
  ToRobotVisitor to_robot_visitor;
  INTR_RETURN_IF_ERROR(object->Accept(to_robot_visitor));
  return to_robot_visitor.result;
}

absl::StatusOr<const KinematicObject*> ObjectWorld::GetKinematicObject(
    const ObjectWorldResourceId& object_id) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      KinematicObject * result,
      const_cast<ObjectWorld*>(this)->GetKinematicObject(object_id));
  return result;
}

absl::StatusOr<KinematicObject*> ObjectWorld::GetKinematicObject(
    const WorldObjectName& object_name) {
  INTR_ASSIGN_OR_RETURN(WorldObject * object, GetObject(object_name));
  ToRobotVisitor to_robot_visitor;
  INTR_RETURN_IF_ERROR(object->Accept(to_robot_visitor));
  return to_robot_visitor.result;
}

absl::StatusOr<const KinematicObject*> ObjectWorld::GetKinematicObject(
    const WorldObjectName& object_name) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      KinematicObject * result,
      const_cast<ObjectWorld*>(this)->GetKinematicObject(object_name));
  return result;
}

absl::StatusOr<KinematicObject*>
ObjectWorld::GetKinematicObjectForSceneObjectInstance(
    absl::string_view instance_name) {
  INTR_ASSIGN_OR_RETURN(WorldObject * object,
                        GetObjectForSceneObjectInstance(instance_name));
  ToRobotVisitor to_robot_visitor;
  INTR_RETURN_IF_ERROR(object->Accept(to_robot_visitor));
  return to_robot_visitor.result;
}

absl::StatusOr<const KinematicObject*>
ObjectWorld::GetKinematicObjectForSceneObjectInstance(
    absl::string_view instance_name) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      KinematicObject * result,
      const_cast<ObjectWorld*>(this)->GetKinematicObjectForSceneObjectInstance(
          instance_name));
  return result;
}

std::vector<const WorldObject*> ObjectWorld::GetObjects() const {
  const WorldHashMap<ObjectWorldResourceId, std::unique_ptr<WorldObject>>&
      objects_by_id = data_->GetObjectsById();

  std::vector<const WorldObject*> objects;
  objects.reserve(objects_by_id.size());
  for (const auto& [_, object] : objects_by_id) {
    objects.push_back(object.get());
  }
  return objects;
}

std::vector<const WorldObject*> ObjectWorld::GetObjectsSorted() const {
  std::vector<const WorldObject*> objects = GetObjects();
  absl::c_sort(objects, [](const WorldObject* a, const WorldObject* b) {
    return a->GetName() < b->GetName();
  });
  return objects;
}

std::vector<const WorldObject*> ObjectWorld::GetObjectsPartiallySorted() const {
  std::vector<const WorldObject*> objects_in_hierarchy;
  objects_in_hierarchy.reserve(GetObjects().size());
  const WorldObject* root_object = GetObject(RootObjectId()).value();
  std::queue<const WorldObject*> traversal_q;
  traversal_q.push(root_object);
  while (!traversal_q.empty()) {
    const WorldObject* object = traversal_q.front();
    const std::vector<const WorldObject*> children = object->GetChildren();
    for (const WorldObject* child : children) {
      traversal_q.push(child);
    }
    objects_in_hierarchy.push_back(object);
    traversal_q.pop();
  }
  return objects_in_hierarchy;
}

absl::StatusOr<Frame*> ObjectWorld::GetFrame(const ObjectWorldResourceId& id) {
  for (const auto& [_, object] : data_->GetObjectsById()) {
    absl::StatusOr<Frame*> frame = object->GetFrame(id);
    if (absl::IsNotFound(frame.status())) {
      continue;
    }
    return frame;
  }
  return absl::NotFoundError(absl::StrCat("Frame with id \"", id.value(),
                                          "\" does not exist on any object."));
}

absl::StatusOr<const Frame*> ObjectWorld::GetFrame(
    const ObjectWorldResourceId& id) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(Frame * result,
                        const_cast<ObjectWorld*>(this)->GetFrame(id));
  return result;
}
absl::StatusOr<Frame*> ObjectWorld::GetFrame(const WorldObjectName& object_name,
                                             const FrameName& frame_name) {
  INTR_ASSIGN_OR_RETURN(WorldObject * object, GetObject(object_name));
  INTR_ASSIGN_OR_RETURN(Frame * frame, object->GetFrame(frame_name));
  return frame;
}

absl::StatusOr<const Frame*> ObjectWorld::GetFrame(
    const WorldObjectName& object_name, const FrameName& frame_name) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      Frame * result,
      const_cast<ObjectWorld*>(this)->GetFrame(object_name, frame_name));
  return result;
}

absl::StatusOr<TransformNode*> ObjectWorld::GetTransformNode(
    const ObjectWorldResourceId& id) {
  absl::StatusOr<WorldObject*> object = GetObject(id);
  if (!absl::IsNotFound(object.status())) {
    return object;
  }
  absl::StatusOr<Frame*> frame = GetFrame(id);
  if (!absl::IsNotFound(frame.status())) {
    return frame;
  }
  return absl::NotFoundError(
      absl::StrCat("Neither an object nor a frame with id \"", id.value(),
                   "\" does exist."));
}

absl::StatusOr<const TransformNode*> ObjectWorld::GetTransformNode(
    const ObjectWorldResourceId& id) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(TransformNode * result,
                        const_cast<ObjectWorld*>(this)->GetTransformNode(id));
  return result;
}

absl::StatusOr<TransformNode*> ObjectWorld::GetTransformNodeByEntityId(
    EntityId entity_id) {
  for (auto& [_, object] : data_->GetObjectsById()) {
    if (object->GetCollectionEntity().has_value() &&
        *object->GetCollectionEntity() == entity_id) {
      return object.get();
    }

    INTR_ASSIGN_OR_RETURN(AttachmentEntityId object_root_id,
                          object->GetRootEntityId());
    if (object_root_id == entity_id) {
      return object.get();
    }

    for (Frame* frame : object->GetFrames()) {
      INTR_ASSIGN_OR_RETURN(AttachmentEntityId frame_id,
                            frame->GetTransformOriginEntityId());
      if (frame_id == entity_id) {
        return frame;
      }
    }
  }

  return absl::NotFoundError(
      absl::StrCat("Could not find an object or frame corresponding to entity ",
                   entity_id.value()));
}

absl::StatusOr<const TransformNode*> ObjectWorld::GetTransformNodeByEntityId(
    EntityId entity_id) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      TransformNode * result,
      const_cast<ObjectWorld*>(this)->GetTransformNodeByEntityId(entity_id));
  return result;
}

absl::StatusOr<TransformNode*> ObjectWorld::GetTransformNodeByAlias(
    absl::string_view entity_alias) {
  INTR_ASSIGN_OR_RETURN(EntityId id,
                        GetEntityWorld().FindByAlias(entity_alias));

  INTR_ASSIGN_OR_RETURN(
      TransformNode * node, GetTransformNodeByEntityId(id),
      _ << "For the entity with alias \"" << entity_alias << "\"");

  return node;
}

absl::StatusOr<const TransformNode*> ObjectWorld::GetTransformNodeByAlias(
    absl::string_view entity_alias) const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      TransformNode * result,
      const_cast<ObjectWorld*>(this)->GetTransformNodeByAlias(entity_alias));
  return result;
}

namespace {

template <class ComponentProto, class Component>
absl::Status UpdateEntityComponentFromProtoImpl(
    AttachmentEntityId entity_id, const ComponentProto& component_proto,
    const ObjectWorld& object_world, World* world) {
  absl::Status status =
      object_world.GetObjectByMemberEntityId(entity_id).status();
  if (absl::IsNotFound(status)) {
    return absl::InvalidArgumentError(absl::Substitute(
        "Could not update a component of entity \"$0\" because the "
        "entity is not a member of any object.",
        entity_id.value()));
  }
  INTR_RETURN_IF_ERROR(status);

  INTR_ASSIGN_OR_RETURN(WorldEntity * entity, world->GetEntityById(entity_id));
  INTR_ASSIGN_OR_RETURN(Component * component,
                        entity->GetComponent<Component>());

  INTR_RETURN_IF_ERROR(component->UpdateFromProto(component_proto));
  return absl::OkStatus();
}

}  // namespace

absl::Status ObjectWorld::UpdateEntityComponentFromProto(
    AttachmentEntityId entity_id,
    const intrinsic_proto::world::PhysicsComponent& physics_component) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  return UpdateEntityComponentFromProtoImpl<
      intrinsic_proto::world::PhysicsComponent, PhysicsComponent>(
      entity_id, physics_component, *this, world);
}

absl::Status ObjectWorld::UpdateEntityComponentFromProto(
    AttachmentEntityId entity_id,
    const intrinsic_proto::world::GeometryComponent& geometry_component) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  return UpdateEntityComponentFromProtoImpl<
      intrinsic_proto::world::GeometryComponent, GeometryComponent>(
      entity_id, geometry_component, *this, world);
}

absl::Status ObjectWorld::ApplyGeometryOptionOverrides(
    AttachmentEntityId entity_id,
    const intrinsic_proto::world::GeometryOptions& geometry_options,
    GeometryLibrary& geolib) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());

  if (world->ValidateEntity<GeometryComponentType>(entity_id).ok()) {
    INTR_ASSIGN_OR_RETURN(
        GeometryComponent * geometry_component,
        world->GetComponentByEntityId<GeometryComponent>(entity_id));
    // TODO: b/469159066 -- We need to deserialize the geometry here in order to
    // set the proto options due to our design choices when defining our protos.
    for (const auto& name : geometry_component->GetGeometryNames()) {
      INTR_RETURN_IF_ERROR(
          geometry_component->GetGeometry(name, geolib.Deserializer())
              .status());
    }

    INTR_RETURN_IF_ERROR(
        geometry_component->ApplyGeometryOptionOverrides(geometry_options));

    // TODO: b/469159066 -- Reserialize so we can write them out again.
    for (const auto& name : geometry_component->GetGeometryNames()) {
      INTR_ASSIGN_OR_RETURN(
          NamedGeometrySet geos,
          geometry_component->GetGeometry(name, geolib.Deserializer()));
      // Reserialize so that we can write them out again.
      NamedGeometryProtoSet geo_protos;
      for (const auto& [name, geo] : geos) {
        INTR_ASSIGN_OR_RETURN(geo_protos[name],
                              ToProto(geo, &geolib.Serializer()));
      }
      geometry_component->SetGeometry(name, geo_protos);
    }
  }

  return absl::OkStatus();
}

absl::StatusOr<intrinsic_proto::RuleSet>
ObjectWorld::GetDefaultCollisionSettings() const {
  return GetEntityWorld().GetDefaultRuleSet();
}

absl::Status ObjectWorld::SetDefaultCollisionSettings(
    const intrinsic_proto::RuleSet& settings) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  return world->SetDefaultRuleSet(settings);
}

absl::Status ObjectWorld::SyncFrameFromOtherWorld(const Frame& other_frame) {
  if (&this->GetEntityWorld() == &other_frame.GetEntityWorld()) {
    return absl::InvalidArgumentError(
        "The other_frame is located in this world, not in another world.");
  }

  if (data_->GetObjectsById().find(other_frame.GetParent()->GetId()) ==
      data_->GetObjectsById().end()) {
    return absl::InvalidArgumentError(
        "Frames modified for a new object require the object to be saved "
        "first.");
  }
  INTR_ASSIGN_OR_RETURN(WorldObject * object,
                        GetObject(other_frame.GetParent()->GetId()));
  std::vector<EntityId> new_entity_ids;
  new_entity_ids.push_back(other_frame.GetEntityId());

  for (const Frame* child_frame : other_frame.GetChildFramesRecursively()) {
    new_entity_ids.push_back(child_frame->GetEntityId());
  }

  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());

  absl::StatusOr<Frame*> frame = object->GetFrame(other_frame.GetId());
  if (!absl::IsNotFound(frame.status())) {
    std::vector<EntityId> old_entity_ids;
    old_entity_ids.push_back(frame.value()->GetEntityId());

    for (const Frame* child_frame :
         frame.value()->GetChildFramesRecursively()) {
      old_entity_ids.push_back(child_frame->GetEntityId());
    }

    for (const EntityId& id : old_entity_ids) {
      INTR_RETURN_IF_ERROR(world->SafelyRemoveEntity(id));
    }
  }

  WorldHashMap<EntityId, const WorldEntity*> from_entities;
  for (const EntityId& id : new_entity_ids) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* from_entity,
                          other_frame.GetEntityWorld().GetEntityById(id));
    from_entities.insert({id, from_entity});
  }
  INTR_RETURN_IF_ERROR(world->IntegrateEntitiesKeepingIds(from_entities));
  return absl::OkStatus();
}

absl::Status ObjectWorld::SyncObjectFromOtherWorld(
    const WorldObject& other_object) {
  if (&this->GetEntityWorld() == &other_object.GetEntityWorld()) {
    return absl::InvalidArgumentError(
        "The other_object is located in this world, not in another world.");
  }
  std::vector<EntityId> entity_ids;
  if (other_object.GetCollectionEntity().has_value()) {
    entity_ids.push_back(other_object.GetCollectionEntity().value());
  }
  for (EntityId entity_id : other_object.GetEntityIds()) {
    entity_ids.push_back(entity_id);
  }
  for (const Frame* frame : other_object.GetFrames()) {
    entity_ids.push_back(frame->GetEntityId());
  }

  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());

  if (data_->GetObjectsById().find(other_object.GetId()) !=
      data_->GetObjectsById().end()) {
    INTR_ASSIGN_OR_RETURN(WorldObject * object,
                          GetObject(other_object.GetId()));
    std::vector<EntityId> object_entity_ids;
    if (object->GetCollectionEntity().has_value()) {
      object_entity_ids.push_back(*object->GetCollectionEntity());
    }
    for (const EntityId id : object->GetEntityIds()) {
      object_entity_ids.push_back(id);
    }
    for (const Frame* frame : object->GetFrames()) {
      object_entity_ids.push_back(frame->GetEntityId());
    }

    for (const EntityId& id : object_entity_ids) {
      INTR_RETURN_IF_ERROR(world->RemoveEntity(id));
    }
  }

  WorldHashMap<EntityId, const WorldEntity*> from_entities;
  for (const EntityId& id : entity_ids) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* from_entity,
                          other_object.GetEntityWorld().GetEntityById(id));
    from_entities.insert({id, from_entity});
  }
  INTR_RETURN_IF_ERROR(world->IntegrateEntitiesKeepingIds(from_entities));

  // Cleanup collision exclusion pairs: We just replaced some entities
  // ('from_entities') with new, modified entities, potentially removing some
  // collision exclusions with entities that were not replaced. The entities
  // that were not replaced might still have collision exclusions with the
  // replaced entities.
  INTR_RETURN_IF_ERROR(world->RemoveNonmutualExclusions());

  // TODO(b/261969695): Clean up the object world after this operation.

  return absl::OkStatus();
}

}  // namespace object_world
}  // namespace intrinsic
