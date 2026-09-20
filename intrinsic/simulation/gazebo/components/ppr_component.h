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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_PPR_COMPONENT_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_PPR_COMPONENT_H_

#include <string>

#include "gz/sim/components/Component.hh"

namespace intrinsic {
namespace simulation {

// Used with models to denote the resource (as in PPR) name for the object. This
// name is expected to be unique across all resources in a world.
using ResourceName =
    gz::sim::components::Component<std::string, class ResourceNameTag>;
}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_PPR_COMPONENT_H_
