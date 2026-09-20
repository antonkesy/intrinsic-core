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

#ifndef INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_DATA_H_
#define INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_DATA_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {

class WorldObject;

// Internal helper for holding the content of an ObjectWorld so that it can be
// shared with all objects and frames in the world.
//
// This class is split into two classes - ObjectWorldData and a subclass
// MutableObjectWorldData - to facilitate const-correctness of the ObjectWorld
// class. ObjectWorld holds an ObjectWorldData* which we set to an instance of
// ObjectWorldData when we create a const instance of ObjectWorld from a "const
// World&" and which we set to an instance of MutableObjectWorldData when we
// create a non-const instance of ObjectWorld from a "World&".
class ObjectWorldData {
 public:
  explicit ObjectWorldData(const World& world);
  virtual ~ObjectWorldData();

  // This class is neither copyable nor moveable.
  ObjectWorldData(const ObjectWorldData&) = delete;
  ObjectWorldData& operator=(const ObjectWorldData&) = delete;
  ObjectWorldData(ObjectWorldData&&) = delete;
  ObjectWorldData& operator=(ObjectWorldData&&) = delete;

  // Returns the underlying entity world.
  const World& GetEntityWorld() const;

  // Returns a mutable world. This will be either an error or non-null.
  virtual absl::StatusOr<World*> GetMutableEntityWorld();

  // Returns the map from ids to objects for reading.
  const WorldHashMap<ObjectWorldResourceId, std::unique_ptr<WorldObject>>&
  GetObjectsById() const {
    return objects_by_id_;
  }

  const WorldHashMap<WorldObjectName, WorldHashSet<WorldObject*>>&
  GetObjectsByName() const {
    return objects_by_name_;
  }

  const WorldHashMap<AttachmentEntityId, WorldObject*>& GetObjectsByEntityId()
      const {
    return objects_by_entity_id_;
  }

  // Returns an error if the given name of a (not yet existing) global frame
  // would collide with the name of an existing object.
  //
  // Together with CheckObjectNameAgainstGlobalFrameNames() below this check
  // ensures that we can use the notation "[<object name>.]<frame name>" without
  // ambiguities between objects and global frames (cases a) and b)):
  //   a) Object:        "my_object"
  //   b) Global frame:  "my_global_frame"
  //   c) Local frame:   "my_object.my_local_frame"
  absl::Status CheckGlobalFrameNameAgainstGlobalObjectNames(
      const FrameName& name_to_be_added) const;

  // Returns error if the given name of (not yet existing) global name would
  // collide with the name of an existing global object(child of root object or
  // object with global alias).
  absl::Status CheckObjectNameAgainstGlobalObjectNames(
      const WorldObjectName& name_to_be_added) const;

  // Returns an error if the given name of a (not yet existing) object would
  // collide with the name of an existing global frame.
  //
  // Also see CheckGlobalFrameNameAgainstGlobalObjectNames() above.
  absl::Status CheckObjectNameAgainstGlobalFrameNames(
      const WorldObjectName& name_to_be_added) const;

  // Adds a the given object or returns an error if an object with the same id
  // already exists.
  absl::Status InsertObject(std::unique_ptr<WorldObject> object);

  // Removes the object with the given id or returns an error if no such object
  // exists.
  absl::StatusOr<std::unique_ptr<WorldObject>> RemoveObject(
      ObjectWorldResourceId object_id);

  absl::Status RenameObject(const WorldObjectName& old_name,
                            const WorldObjectName& new_name,
                            WorldObject& object);

  // Registers an entity to belong to the given object.
  void RegisterEntity(AttachmentEntityId entity_id, WorldObject* object);

  // Unregisters an entity.
  void UnregisterEntity(AttachmentEntityId entity_id);

 private:
  WorldHashMap<ObjectWorldResourceId, std::unique_ptr<WorldObject>>
      objects_by_id_;
  WorldHashMap<WorldObjectName, WorldHashSet<WorldObject*>> objects_by_name_;
  WorldHashMap<AttachmentEntityId, WorldObject*> objects_by_entity_id_;
  const World& world_;
};

// Extension of ObjectWorldData for use in non-const instances of ObjectWorld.
// See the description of ObjectWorldData.
class MutableObjectWorldData : public ObjectWorldData {
 public:
  explicit MutableObjectWorldData(World& world);

  absl::StatusOr<World*> GetMutableEntityWorld() override;

 private:
  World& mutable_world_;
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_OBJECT_WORLD_DATA_H_
