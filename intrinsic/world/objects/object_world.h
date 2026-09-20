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

#ifndef INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_H_
#define INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_H_

#include <memory>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/api/geometry_options.h"
#include "intrinsic/geometry/storage/geometry_library.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/object_world_data.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/geometry_component.pb.h"
#include "intrinsic/world/proto/physics_component.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {

// An object-based view onto a intrinsic::World.
//
// Wraps a intrinsic::World and provides a simplified interface for accessing
// it. Roughly speaking, entity-collections which represent physical objects are
// presented as WorldObject's and pose-only entities (entities which only have
// an AttachmentComponent) are presented as Frames.
class ObjectWorld {
 public:
  // Creates an ObjectWorld view for the given intrinsic::World. All changes to
  // the returned view will directly access and mutate the given world.
  //
  // The given intrinsic::World is owned externally and must outlive the
  // returned ObjectWorld instance.
  static absl::StatusOr<std::unique_ptr<ObjectWorld>> CreateView(World& world);
  static absl::StatusOr<std::unique_ptr<const ObjectWorld>> CreateView(
      const World& world);

  // Checks whether the given intrinsic::World supports the ObjectWorld view and
  // whether it can safely be passed to CreateView() (see above). In the case of
  // incompatibility, returns an error explaining how the given world could be
  // fixed.
  static absl::Status VerifyWorldIsCompatible(const World& world);

  // Checks whether the given intrinsic::World could be automatically converted
  // to support the ObjectWorld view and whether it can safely be passed to the
  // non const version of CreateView() (see above). In the case of
  // incompatibility, returns an error explaining how the given world could be
  // fixed.
  static absl::Status VerifyWorldCouldBeCompatible(const World& world);

  // Returns the object with the given id. Returns an error if no matching
  // object can be found.
  absl::StatusOr<WorldObject*> GetObject(
      const ObjectWorldResourceId& object_id);
  absl::StatusOr<const WorldObject*> GetObject(
      const ObjectWorldResourceId& object_id) const;

  // Returns the object with the given name. Returns an error if no matching
  // object can be found.
  absl::StatusOr<WorldObject*> GetObject(const WorldObjectName& object_name);
  absl::StatusOr<const WorldObject*> GetObject(
      const WorldObjectName& object_name) const;

  // Returns the object associated with the given scene object instance. Returns
  // an error if no matching object can be found.
  absl::StatusOr<WorldObject*> GetObjectForSceneObjectInstance(
      absl::string_view instance_name);
  absl::StatusOr<const WorldObject*> GetObjectForSceneObjectInstance(
      absl::string_view instance_name) const;

  // Returns the object that contains the entity with the given id as a member.
  // Returns an error if no matching object can be found.
  absl::StatusOr<WorldObject*> GetObjectByMemberEntityId(
      AttachmentEntityId entity_id);
  absl::StatusOr<const WorldObject*> GetObjectByMemberEntityId(
      AttachmentEntityId entity_id) const;

  // Returns the object by the full path names from root(not included).
  // If an empty list is provided, the root object is returned.
  // Returns an error if no matching object can be found.
  absl::StatusOr<WorldObject*> GetObjectByFullPath(
      const std::vector<WorldObjectName>& names);
  absl::StatusOr<const WorldObject*> GetObjectByFullPath(
      const std::vector<WorldObjectName>& names) const;

  // Returns the object with the given id as a KinematicObject. Returns an error
  // if no matching object can be found or if the object is not a kinematic
  // object.
  absl::StatusOr<KinematicObject*> GetKinematicObject(
      const ObjectWorldResourceId& object_id);
  absl::StatusOr<const KinematicObject*> GetKinematicObject(
      const ObjectWorldResourceId& object_id) const;

  // Returns the object with the given name as a KinematicObject. Returns an
  // error if no matching object can be found or if the object is not a
  // kinematic object.
  absl::StatusOr<KinematicObject*> GetKinematicObject(
      const WorldObjectName& object_name);
  absl::StatusOr<const KinematicObject*> GetKinematicObject(
      const WorldObjectName& object_name) const;

  // Returns the object associated with the given scene object instance as a
  // KinematicObject. Returns an error if no matching object can be found or if
  // the object is not a kinematic object.
  absl::StatusOr<KinematicObject*> GetKinematicObjectForSceneObjectInstance(
      absl::string_view instance_name);
  absl::StatusOr<const KinematicObject*>
  GetKinematicObjectForSceneObjectInstance(
      absl::string_view instance_name) const;

  // Returns all objects in the world. The ordering is arbitrary and may vary
  // with each call to this method. The result includes the root object.
  std::vector<const WorldObject*> GetObjects() const;

  // Returns all objects in the world sorted according to their names in
  // lexicograhical order. The result includes the root object.
  std::vector<const WorldObject*> GetObjectsSorted() const;

  // Returns all objects in the world sorted partially according to their
  // hierarchical relationship: parent objects before child objects. The result
  // includes the root object.
  std::vector<const WorldObject*> GetObjectsPartiallySorted() const;

  // Returns the frame with the given id. Returns an error if no such frame
  // exists under any object in this world.
  absl::StatusOr<Frame*> GetFrame(const ObjectWorldResourceId& id);
  absl::StatusOr<const Frame*> GetFrame(const ObjectWorldResourceId& id) const;

  // Returns the frame with the given frame-name from the object with the given
  // object-name. Returns an error if no such frame exists.
  // This is a convenience method for ObjectWorld::GetObject(WorldObjectName)
  // followed by WorldObject::GetFrame(FrameName).
  absl::StatusOr<Frame*> GetFrame(const WorldObjectName& object_name,
                                  const FrameName& frame_name);
  absl::StatusOr<const Frame*> GetFrame(const WorldObjectName& object_name,
                                        const FrameName& frame_name) const;

  // Returns the TransformNode (e.g., a frame or an object) with the given id.
  // Returns an error if no such TransformNode exists anywhere in the world.
  absl::StatusOr<TransformNode*> GetTransformNode(
      const ObjectWorldResourceId& id);
  absl::StatusOr<const TransformNode*> GetTransformNode(
      const ObjectWorldResourceId& id) const;

  // Returns the TransformNode (e.g., a frame or an object) that represents the
  // entity with the given alias. Returns an error if no matching TransformNode
  // exists in the world.
  // This is advanced functionality for compatibility with the entity-based
  // world API. It only works for certain attachment entities as they are common
  // in worlds of apps that still use the entity-based world view (e.g., an
  // attachment entity that is a leaf of an entity collection/object won't
  // be accepted - we'd expect the root entity).
  absl::StatusOr<TransformNode*> GetTransformNodeByAlias(
      absl::string_view entity_alias);
  absl::StatusOr<const TransformNode*> GetTransformNodeByAlias(
      absl::string_view entity_alias) const;

  absl::StatusOr<TransformNode*> GetTransformNodeByEntityId(EntityId entity_id);
  absl::StatusOr<const TransformNode*> GetTransformNodeByEntityId(
      EntityId entity_id) const;

  // Copies an object from another world into this world. Creates a new object
  // if it does not exist or overwrites an existing object with this ID.
  absl::Status SyncObjectFromOtherWorld(const WorldObject& other_object);

  // Copies a frame from another world into this world. Creates a new frame
  // if it does not exist or overwrites an existing frame with this ID.
  absl::Status SyncFrameFromOtherWorld(const Frame& other_frame);

  // Update certain allowed components of an entity with a given id.
  // A new component of type T will be created if the entity does not have a
  // component of type T yet. An error will be returned if the entity is not
  // part of any object in the world or if the given component is not
  // object-view compatible.
  absl::Status UpdateEntityComponentFromProto(
      AttachmentEntityId entity_id,
      const intrinsic_proto::world::PhysicsComponent& physics_component);
  absl::Status UpdateEntityComponentFromProto(
      AttachmentEntityId entity_id,
      const intrinsic_proto::world::GeometryComponent& geometry_component);

  // Applies the geometry option overrides for all the geometries of this
  // object. If a field is not set in the overrides, it will not modify the
  // equivalent field in the existing (or default) options for the geometry.
  absl::Status ApplyGeometryOptionOverrides(
      AttachmentEntityId entity_id,
      const intrinsic_proto::world::GeometryOptions& options,
      GeometryLibrary& geolib);

  // Returns the default collision settings for the given world. These are used
  // when doing collision checking. When constructing a collision checker these
  // can be overridden by rules passed into the collision checker creation
  // methods.
  //  TODO(b/253631144): Use CollisionSettings instead of RuleSet
  absl::StatusOr<intrinsic_proto::RuleSet> GetDefaultCollisionSettings() const;

  // Sets the default collision settings, for more information take a look at
  // the comments for GetDefaultCollisionSettings()
  //  TODO(b/253631144): Use CollisionSettings instead of RuleSet
  absl::Status SetDefaultCollisionSettings(
      const intrinsic_proto::RuleSet& settings);

  // Returns the underlying World for this view.
  const World& GetEntityWorld() const { return data_->GetEntityWorld(); }

 private:
  explicit ObjectWorld(std::unique_ptr<ObjectWorldData> data);

  absl::StatusOr<World*> GetMutableEntityWorld() {
    return data_->GetMutableEntityWorld();
  }

  std::unique_ptr<ObjectWorldData> data_;
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_H_
