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

#ifndef INTRINSIC_WORLD_COMPONENT_SIMULATION_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_SIMULATION_COMPONENT_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/proto/simulation_component.pb.h"

namespace intrinsic {

// Component indicating how the owning Entity (or the object/collection
// represented by the owning entity) should be treated in simulation.
class SimulationComponent {
 public:
  virtual ~SimulationComponent() = default;

  // Returns a new SimulationComponent instance.
  static std::unique_ptr<SimulationComponent> Create();

  // Returns a new SimulationComponent instance derived from the given proto.
  static absl::StatusOr<std::unique_ptr<SimulationComponent>> FromProto(
      const intrinsic_proto::world::SimulationComponent& proto);

  // Returns a new SimulationComponent instance that is a copy of this one.
  virtual std::unique_ptr<SimulationComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::SimulationComponent> ToProto()
      const = 0;

  // Returns whether the entity or object is disabled in simulation.
  virtual bool IsDisabled() const = 0;

  // Sets whether the entity or object is disabled in simulation.
  virtual void SetDisabled(bool disabled) = 0;

  // Returns whether the entity or object is immovable.
  virtual bool IsStatic() const = 0;

  // Sets whether the entity or object is immovable.
  virtual void SetIsStatic(bool is_static) = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::SimulationComponent& proto) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_SIMULATION_COMPONENT_H_
