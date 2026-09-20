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

#ifndef INTRINSIC_WORLD_COMPONENT_PROJECTOR_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_PROJECTOR_COMPONENT_H_

#include <cstdint>
#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/proto/projector_component.pb.h"

namespace intrinsic {

// Struct to hold texture data
struct Texture {
  std::string data;
  std::string format;
};

// A component to hold information about projectors in the
// World.
class ProjectorComponent {
 public:
  virtual ~ProjectorComponent() = default;

  // Returns a new ProjectorComponent instance.
  static std::unique_ptr<ProjectorComponent> Create();

  // Returns a new ProjectorComponent instance derived from the given proto.
  static absl::StatusOr<std::unique_ptr<ProjectorComponent>> FromProto(
      const intrinsic_proto::world::ProjectorComponent& proto);

  // Returns a new ProjectorComponent instance that is a copy of this one.
  virtual std::unique_ptr<ProjectorComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::ProjectorComponent> ToProto()
      const = 0;

  // Get the horizontal field of viewused by projector
  virtual double GetHorizontalFov() const = 0;

  // Set the horizontal field of view
  virtual void SetHorizontalFov(double hfov) = 0;

  // Get the near clip distance in meters
  virtual double GetNearClip() const = 0;

  // Set the near clip distance in meters
  virtual void SetNearClip(double near) = 0;

  // Get the far clip distance in meters
  virtual double GetFarClip() const = 0;

  // Set the far clip distance in meters
  virtual void SetFarClip(double far) = 0;

  // Get the visibility flags
  virtual uint32_t GetVisibilityFlags() const = 0;

  // Set the visibility flags
  virtual void SetVisibilityFlags(uint32_t visibility_flags) = 0;

  // Get the texture used by projector
  virtual const Texture& GetTexture() const = 0;

  // Sets the projector texture for the given name.
  virtual void SetTexture(const Texture& texture) = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::ProjectorComponent& proto) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_PROJECTOR_COMPONENT_H_
