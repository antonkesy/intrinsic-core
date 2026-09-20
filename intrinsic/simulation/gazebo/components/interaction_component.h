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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_INTERACTION_COMPONENT_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_INTERACTION_COMPONENT_H_

#include "gz/sim/components/Component.hh"

namespace intrinsic {
namespace simulation {

// Used to denote the type of interactions enabled for this link entity in sim.

enum InteractionType { kRobotTip, kFloatingObject };

using Interaction = gz::sim::components::Component<int, class InteractionTag>;

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_INTERACTION_COMPONENT_H_
