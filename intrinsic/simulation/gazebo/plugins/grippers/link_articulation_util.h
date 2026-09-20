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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_LINK_ARTICULATION_UTIL_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_LINK_ARTICULATION_UTIL_H_

#include <ostream>

#include "absl/status/statusor.h"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Link.hh"
#include "intrinsic/simulation/gazebo/components/articulation_component.h"

namespace intrinsic {
namespace simulation {

// Returns the articulation type of the given link from the ECM if the
// Articulation component is present. If not, the function computes the
// articulation type by traversing the kinematic chain of the link.
// The ECM is expected to have a standard structure as created by
// `SdfEntityCreator` on an SDF string with a `world` element. Otherwise, an
// `InvalidArgumentError` is returned.
absl::StatusOr<ArticulationType> GetArticulationType(
    const gz::sim::Link& link, const gz::sim::EntityComponentManager& ecm);

// Print ArticulationType to ostream.
std::ostream& operator<<(std::ostream& _out,
                         ArticulationType articulation_type);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_LINK_ARTICULATION_UTIL_H_
