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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SYSTEM_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SYSTEM_H_

#include <memory>

#include "gz/sim/Entity.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "intrinsic/simulation/gazebo/plugins/sim_inputs/sim_inputs_service.h"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

// A Gazebo system plugin that hosts a SimInputsService gRPC service to allow
// callers to set simulated inputs.
// Individual inputs are registered through DigitalInputOutput plugins in
// individual Models in the Gazebo scene.
// This plugin should be specified (once!) at the world scope in the SDFormat
// definition.
class SimInputsSystem final : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemUpdate {
 public:
  SimInputsSystem()
      : sim_inputs_service_(std::make_shared<SimInputsService>()) {}
  ~SimInputsSystem() override;

  // This registers the system with SimInputsServiceSingleton as a handler for
  // the SetSimulatedInputs gRPC service.
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_mgr) override;

  void Update(const gz::sim::UpdateInfo& info,
              gz::sim::EntityComponentManager& ecm) override;

 private:
  // Needs to be a shared_ptr for shared ownership with
  // SimInputsServiceSingleton.
  std::shared_ptr<SimInputsService> sim_inputs_service_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SYSTEM_H_
