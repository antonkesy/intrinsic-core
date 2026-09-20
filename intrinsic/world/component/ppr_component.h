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

#ifndef INTRINSIC_WORLD_COMPONENT_PPR_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_PPR_COMPONENT_H_

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/world/proto/ppr_component.pb.h"

namespace intrinsic {

// A component to bind resource instance or products to specific world entities.
class PPRComponent {
 public:
  virtual ~PPRComponent() = default;

  // Returns a new PPRComponent instance.
  static std::unique_ptr<PPRComponent> Create();

  // Returns a new PPRComponent instance derived from the given proto.
  static absl::StatusOr<std::unique_ptr<PPRComponent>> FromProto(
      const intrinsic_proto::world::PPRComponent& proto);

  // Returns a new PPRComponent instance that is a copy of this one.
  virtual std::unique_ptr<PPRComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::PPRComponent> ToProto()
      const = 0;

  // Returns the name of the associated resource instance or nullopt if no
  // resource is associated to this component.
  virtual std::optional<absl::string_view> ResourceName() const = 0;

  // Sets the name of the associated resource instance. Additionally, clears the
  // product name.
  virtual void SetResourceName(absl::string_view resource_name) = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::PPRComponent& proto) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_PPR_COMPONENT_H_
