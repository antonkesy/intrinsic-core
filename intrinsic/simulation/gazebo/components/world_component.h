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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_WORLD_COMPONENT_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_WORLD_COMPONENT_H_

#include <string>

#include "gz/sim/components/Component.hh"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace simulation {

// Used with links and models to denote their entity ID in the corresponding
// Intrinsic world.
using WorldEntityId =
    gz::sim::components::Component<uint32_t, class WorldEntityIdTag>;

// Used with models to denote their corresponding world object in the Intrinsic
// world. It is guaranteed that this object resource ID is unique by
// construction. (See intrinsic/world/objects/object_world_ids.h).
using WorldObjectResourceId =
    gz::sim::components::Component<std::string, class WorldObjectResourceIdTag>;

// Used with models to denote the name of their corresponding world object in
// the Intrinsic world.
using WorldObjectName =
    gz::sim::components::Component<std::string, class WorldObjectNameTag>;

// Used with the world entity to signal that something has changed about the
// structure of the corresponding Intrinsic world, so pose updates for this
// entity may be inaccurate. Handled by the PoseUpdateSystem (See
// http://intrinsic/simulation/gazebo/pose_update_system.h)
using WorldDirtyFlag =
    gz::sim::components::Component<bool, class WorldDirtyTag>;

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_WORLD_COMPONENT_H_
