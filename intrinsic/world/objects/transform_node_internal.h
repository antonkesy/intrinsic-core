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

#ifndef INTRINSIC_WORLD_OBJECTS_TRANSFORM_NODE_INTERNAL_H_
#define INTRINSIC_WORLD_OBJECTS_TRANSFORM_NODE_INTERNAL_H_

#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "google/protobuf/timestamp.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world_data.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {

class Frame;
class WorldObject;

// Interface for the visitor pattern on TransformNodes. Implementations can be
// used with TransformNode::Accept().
class TransformNodeVisitor {
 public:
  virtual ~TransformNodeVisitor() = default;
  virtual absl::Status Visit(Frame& frame) = 0;
  virtual absl::Status Visit(WorldObject& object) = 0;
};

// Const variant of TransformNodeVisitor.
class TransformNodeConstVisitor {
 public:
  virtual ~TransformNodeConstVisitor() = default;
  virtual absl::Status Visit(const Frame& frame) = 0;
  virtual absl::Status Visit(const WorldObject& object) = 0;
};

// Abstract base class for all resources in an ObjectWorld that have a pose and
// are part of a world's transform tree.
class TransformNode {
 public:
  TransformNode(ObjectWorldResourceId id, WorldObject* parent,
                ObjectWorldData& data);
  virtual ~TransformNode() = default;

  // Returns the resource id which is unique within a world.
  const ObjectWorldResourceId& GetId() const { return id_; }

  // Returns the parent object or nullptr if called on the root node.
  WorldObject* GetParent() { return parent_; }
  const WorldObject* GetParent() const { return parent_; }

  // Returns the pose of this nodes's origin/base in the space of the parent
  // nodes's origin/base. Returns an error if called on the root node of a
  // world.
  virtual absl::StatusOr<Pose3d> GetParentTThis() const = 0;

  // Returns the transform 'this_t_other', i.e., the pose of the given 'other'
  // node in the space of this transform node.
  absl::StatusOr<Pose3d> GetTransform(
      const TransformNode* other,
      absl::Time timestamp = absl::UnixEpoch()) const;

  // Returns the transform 'this_t_other', i.e., the pose of the entity within
  // the given 'other' node described by the 'other_filter' in the space of the
  // entity within this transform node described by the 'this_filter'.
  absl::StatusOr<Pose3d> GetTransform(
      std::optional<world::ObjectEntityFilter> this_filter,
      const TransformNode* other,
      std::optional<world::ObjectEntityFilter> other_filter,
      absl::Time timestamp = absl::UnixEpoch()) const;

  // Updates this node's pose in the space of its parent (i.e., updates
  // 'parent_t_this') such that the transform between 'node_a' and 'node_b'
  // becomes 'a_t_b'. Returns an error if this node is not located on the path
  // from 'node_a' to 'node_b'. It is valid to set one of 'node_a' or 'node_b'
  // to this node.
  absl::Status SetTransform(const TransformNode* node_a,
                            const TransformNode* node_b, const Pose3d& a_t_b,
                            std::optional<absl::Time> timestamp = std::nullopt);

  // Updates this node's pose in the space of its parent (i.e., updates
  // 'parent_t_this') such that the transform between 'node_a' and 'node_b'
  // becomes 'a_t_b'. Returns an error if this node is not located on the path
  // from 'node_a' to 'node_b'. It is valid to set one of 'node_a' or 'node_b'
  // to this node.
  // If 'bypass_movable_check' is set to true, the check whether this node is
  // allowed to be moved (CheckIsMovable) is bypassed. This should only be used
  // when applying external state updates (e.g. from simulator) where we want
  // to force the update.
  absl::Status SetTransform(
      std::optional<world::ObjectEntityFilter> this_filter,
      const TransformNode* node_a,
      std::optional<world::ObjectEntityFilter> node_a_filter,
      const TransformNode* node_b,
      std::optional<world::ObjectEntityFilter> node_b_filter,
      const Pose3d& a_t_b, std::optional<absl::Time> timestamp = std::nullopt,
      bool bypass_movable_check = false);

  // Accepts the given visitor and calls exactly one of its Visit() methods
  // appropriate to the type of this TransformNode.
  // Note that only this instance is visited and there is no tree-traversal that
  // implicitly causes visits to other TransformNodes.
  virtual absl::Status Accept(TransformNodeVisitor& visitor) = 0;
  virtual absl::Status Accept(TransformNodeConstVisitor& visitor) const = 0;

  // Returns the entity which corresponds to the origin/base of this transform
  // node. This function should only be used internally, or to interface with
  // legacy code that uses entity ids.
  virtual absl::StatusOr<AttachmentEntityId> GetTransformOriginEntityId()
      const = 0;

  virtual absl::StatusOr<AttachmentEntityId> GetTransformEntityId(
      const world::ObjectEntityFilter& filter) const = 0;

  // Returns the underlying entity world.
  const World& GetEntityWorld() const { return data_.GetEntityWorld(); }

 protected:
  ObjectWorldData& GetObjectWorldData() { return data_; }
  const ObjectWorldData& GetObjectWorldData() const { return data_; }

  absl::StatusOr<World*> GetMutableEntityWorld() {
    return data_.GetMutableEntityWorld();
  }

  // Sets the parent object. CAUTION: Does not update child references in the
  // old and new parent object and can lead to an inconsistent state.
  void SetParentAsymmetric(WorldObject* parent);

  // Fails if this transform node is not allowed to be moved relative to the
  // transform node it is attached to.
  virtual absl::Status CheckIsMovable(
      const std::optional<world::ObjectEntityFilter>& filter) const {
    return absl::OkStatus();
  }
  virtual absl::Status CheckIsMovable() const {
    return CheckIsMovable(std::nullopt);
  }

 private:
  ObjectWorldData& data_;
  ObjectWorldResourceId id_;
  WorldObject* parent_;
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_TRANSFORM_NODE_INTERNAL_H_
