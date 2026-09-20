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

#ifndef INTRINSIC_WORLD_OBJECTS_FRAME_INTERNAL_H_
#define INTRINSIC_WORLD_OBJECTS_FRAME_INTERNAL_H_

#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world_data.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/transform_node_internal.h"

namespace intrinsic {
namespace object_world {

class WorldObject;

// A frame in the object-based view onto a world (see ObjectWorld).
//
// Corresponds to an attachment entity which represents a coordinate frame in
// the world such as a tool frame or grasp pose.
class Frame : public TransformNode {
 public:
  // Creates an instance. You shouldn't need to call this directly, use
  // ObjectWorld::CreateView to create an entire ObjectWorld instance instead.
  Frame(ObjectWorldResourceId id, FrameName name, WorldObject* parent,
        Frame* parent_frame, AttachmentEntityId entity_id,
        ObjectWorldData& data);

  // Returns the name of the frame which is guaranteed to be unique amongst all
  // frames under one object.
  const FrameName& GetName() const { return name_; }

  // Sets the name of this frame. Returns an error if the new name is not unique
  // amongst all frames under the same parent object.
  absl::Status SetName(const FrameName& name);

  // Returns the parent frame or nullptr if the frame is attached directly to
  // the parent object.
  Frame* GetParentFrame() { return parent_frame_; }
  const Frame* GetParentFrame() const { return parent_frame_; }

  // Returns all frames under this frame, including ones that are attached
  // indirectly to this frame via another child frame. The ordering is
  // arbitrary and may vary with each call to this method.
  std::vector<Frame*> GetChildFramesRecursively();
  std::vector<const Frame*> GetChildFramesRecursively() const;

  // Returns all frames under this frame, including ones that are attached
  // indirectly to this frame via another child frame. The returned frames are
  // sorted according to their names in lexicograhical order.
  std::vector<const Frame*> GetChildFramesRecursivelySorted() const;

  // Returns the child frames which have this frame as their parent frame and
  // which are not attached directly to the parent object. Not sorted in any
  // particular order.
  std::vector<Frame*> GetChildFrames();
  std::vector<const Frame*> GetChildFrames() const;

  // Returns the child frames which have this frame as their parent frame and
  // which are not attached directly to the parent object. The returned frames
  // are sorted according to their names in lexicograhical order.
  std::vector<const Frame*> GetChildFramesSorted() const;

  // Creates a new child frame attached to this frame and grouped under the same
  // parent object.
  absl::StatusOr<Frame*> CreateChildFrame(const FrameName& new_frame_name,
                                          const Pose3d& frame_t_new_frame);

  // Returns the child objects which have this frame as their parent frame. Not
  // sorted in any particular order.
  absl::StatusOr<std::vector<const WorldObject*>> GetChildObjects() const;

  // Reparents this frame and all of its child frames to the root entity of the
  // given new parent object. This frame and its children will consequently be
  // grouped under the new parent object. The global pose (="root_t_this") of
  // this frame and its children remains unaffected.
  absl::Status ReparentTo(WorldObject& new_parent);

  // Reparents this frame and all of its child frames to the entity represented
  // by the given filter that is part of the given new parent object. This frame
  // and its children will consequently be grouped under the new parent object.
  // The global pose (="root_t_this") of this frame and its children remains
  // unaffected. If the filter resolves to more than one entity this will return
  // an error.
  absl::Status ReparentTo(WorldObject& new_parent,
                          const world::ObjectEntityFilter& filter);

  // Reparents this frame and all of its child frames to the given new parent
  // frame. This frame and its children will consequently be grouped under the
  // same object as the new parent frame. The global pose (="root_t_this") of
  // this frame and its children remains unaffected.
  absl::Status ReparentTo(Frame& new_parent);

  // Reparents this frame and all of its child frames to the final entity of the
  // given new parent object. This frame and its children will consequently be
  // grouped under the new parent object. The global pose (="root_t_this") of
  // this frame and its children remains unaffected.
  // If 'new_parent' is not a kinematic object and thus consists of a single
  // entity, this method is equivalent to "ReparentTo". If 'new_parent' is a
  // kinematic object and the final entity cannot be determined uniquely, an
  // error will be returned.
  absl::Status ReparentToFinalEntityOf(WorldObject& new_parent);

  // Deletes this frame or returns an error if this frame has attached child
  // frames which would be left dangling by this operation. All references to
  // this frame will become invalid after this operation.
  absl::Status DeleteIfNoChildFrames();

  // Recursively deletes this frame and all (directly or indirectly) attached
  // child frames. All references to the deletes frames will become invalid
  // after this operation.
  absl::Status DeleteIncludingChildFrames();

  // Returns the attachment entity corresponding to this object.
  AttachmentEntityId GetEntityId() const { return entity_id_; }

  absl::StatusOr<Pose3d> GetParentTThis() const override;

  absl::Status Accept(TransformNodeVisitor& visitor) override {
    return visitor.Visit(*this);
  }
  absl::Status Accept(TransformNodeConstVisitor& visitor) const override {
    return visitor.Visit(*this);
  }

  absl::StatusOr<AttachmentEntityId> GetTransformOriginEntityId()
      const override;

  absl::StatusOr<AttachmentEntityId> GetTransformEntityId(
      const world::ObjectEntityFilter& filter) const override;

  bool IsAttachmentFrame() const { return is_attachment_frame_; }
  absl::Status SetIsAttachmentFrame(bool is_attachment_frame);

  // Returns OK if movable, otherwise returns some error stating why.
  absl::Status CheckIsMovable(
      const std::optional<world::ObjectEntityFilter>& filter) const override;
  absl::Status CheckIsMovable() const override {
    return CheckIsMovable(std::nullopt);
  }

 private:
  // Friend declarations required for updating references to children.
  friend class ObjectWorldCreationProcess;
  friend class WorldObject;

  // Add the given frame to this frames' children. CAUTION: Does not update
  // parent references in the child frame and can lead to an inconsistent
  // state.
  void AddChildFrameAsymmetric(Frame* child);

  // Removes the given frame from this frames' children. CAUTION: Does not
  // update anything child frame and can lead to an inconsistent state.
  void RemoveChildFrameAsymmetric(const Frame* child);

  absl::Status Delete(bool delete_child_frames);

  absl::Status ReparentToImpl(WorldObject& new_parent_object,
                              Frame* new_parent_frame,
                              const world::ObjectEntityFilter& filter);

  FrameName name_;
  Frame* parent_frame_;
  std::vector<Frame*> child_frames_;
  AttachmentEntityId entity_id_;
  bool is_attachment_frame_ = false;
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_FRAME_INTERNAL_H_
