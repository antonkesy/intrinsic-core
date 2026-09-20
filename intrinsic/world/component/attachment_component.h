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

#ifndef INTRINSIC_WORLD_COMPONENT_ATTACHMENT_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_ATTACHMENT_COMPONENT_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/proto/attachment_component.pb.h"

namespace intrinsic {

// A component to hold the attachment structure of an entity and its dependent
// entities and parent entity.
class AttachmentComponent {
 public:
  virtual ~AttachmentComponent() = default;

  // Returns a new AttachmentComponent instance.
  static std::unique_ptr<AttachmentComponent> Create();

  // Returns a new AttachmentComponent instance derived from the given proto.
  static absl::StatusOr<std::unique_ptr<AttachmentComponent>> FromProto(
      const intrinsic_proto::world::AttachmentComponent& proto);

  // Returns a new AttachmentComponent instance that is a copy of this one.
  virtual std::unique_ptr<AttachmentComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::AttachmentComponent> ToProto()
      const = 0;

  // Returns the parent of this entity.
  virtual AttachmentEntityId GetParentId() const = 0;

  // Sets the parent of this entity.
  virtual void SetParentId(AttachmentEntityId handle) = 0;

  // Returns the transform between this entity and its parent.
  virtual Pose3d GetParentTThis() const = 0;

  // Returns the timestamp of ParentTThis
  // Returinig std::nullopt indicates the pose stored has no time record.
  virtual std::optional<absl::Time> GetTimestamp() const = 0;

  // Sets the transform between this entity and its parent.
  // Setting to timestamp to std::nullopt indicates the pose has no time record.
  virtual void SetParentTThis(
      const Pose3d& parent_t_this,
      std::optional<absl::Time> timestamp = std::nullopt) = 0;

  // Returns true if the pose between this entity and the parent is inaccurate.
  virtual bool IsInaccurate() const = 0;

  // Sets the inaccurate flag for the transform between this entity and its
  // parent.
  virtual void MarkInaccurate(bool inaccurate) = 0;

  // Returns true if the this attachment represents a rigid fixture to the root.
  virtual bool IsFixedInRoot() const = 0;

  // Set whether this attachment represents a rigid fixture to the root or not.
  virtual void MarkFixedInRoot(bool fixed_in_root) = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::AttachmentComponent& proto) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_ATTACHMENT_COMPONENT_H_
