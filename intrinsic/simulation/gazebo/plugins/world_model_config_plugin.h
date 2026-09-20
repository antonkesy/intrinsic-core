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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_WORLD_MODEL_CONFIG_PLUGIN_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_WORLD_MODEL_CONFIG_PLUGIN_H_

#include <memory>
#include <vector>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "sdf/Element.hh"

namespace intrinsic {
namespace simulation {

// The world model config plugin is used with every generated Gazebo model when
// we initialize the simulator from the world service. The purpose of this
// plugin is to simply tag the model with information regarding its relationship
// to the world service. We manage the pose updates for the entities separately.
//
// We expect this plugin to tie the model's links to their associated entities
// in the Intrinsic world. Similarly, the model should know which world object
// it corresponds to. For example:
//
// <model name="snowman">
//   <link name="base" />      <!--    O    -->
//   <link name="middle" />    <!--    o    -->
//   <link name="top" />       <!--    .    -->
//   <plugin filename="static://intrinsic::simulation::WorldModelConfigPlugin"
//           name="intrinsic::simulation::WorldModelConfigPlugin">
//     <link_data name="base" world_entity_id="31" />
//     <link_data name="middle" world_entity_id="32" />
//     <link_data name="top" world_entity_id="33" />
//     <model_data world_entity_id="2" />
//   </plugin>
// </model>
//
// This example has what could be expected from a snowman kinematic object with
// three links.
//
// Additional (optional) attributes for the <model_data> tag are:
//   * world_object_resource_id - the resource id in the ObjectWorld view
//   * world_object_name        - the name of the object in the ObjectWorld view
//   * resource_name            - the name of the (PPR) resource for this model
//   * product_part_name        - the name of the (PPR) product for this model
//
// A model cannot have both a `resource_name` and `product_part_name` attribute
// set.

class WorldModelConfigPlugin : public ::gz::sim::System,
                               public ::gz::sim::ISystemConfigure {
 public:
  WorldModelConfigPlugin() = default;

  void Configure(const ::gz::sim::Entity& entity,
                 const std::shared_ptr<const ::sdf::Element>& sdf,
                 ::gz::sim::EntityComponentManager& ecm,
                 ::gz::sim::EventManager& event_mgr) override;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_WORLD_MODEL_CONFIG_PLUGIN_H_
