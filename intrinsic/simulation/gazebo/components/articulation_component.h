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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_ARTICULATION_COMPONENT_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_ARTICULATION_COMPONENT_H_

#include "gz/sim/components/Component.hh"

namespace intrinsic {
namespace simulation {

// Used to denote how a particular link is articulated.
// Note that this value is expected to be computed based on the kinematic
// structure of the model, and is independent of the `Static` property of the
// parent model.
// Also note that `DetachableJoint`s are not considered in computing the
// articulation type.
// - kFixed: the link is fixed with respect to the world.
// - kArticulated: the link has at least one degree of freedom.
// - kFloating: the link has full six degrees of freedom.
enum class ArticulationType { kFixed, kArticulated, kFloating };

using Articulation =
    gz::sim::components::Component<ArticulationType, class ArticulationTag>;

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_ARTICULATION_COMPONENT_H_
