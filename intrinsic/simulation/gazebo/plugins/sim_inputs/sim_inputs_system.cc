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

#include "intrinsic/simulation/gazebo/plugins/sim_inputs/sim_inputs_system.h"

#include <memory>

#include "absl/log/log.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Types.hh"
#include "intrinsic/simulation/gazebo/plugins/sim_inputs/sim_inputs_service.h"
#include "intrinsic/simulation/gazebo/plugins/sim_inputs/sim_inputs_service_singleton.h"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

SimInputsSystem::~SimInputsSystem() {
  if (sim_inputs_service_) {
    sim_inputs_service_->Shutdown();
    sim_inputs_service_.reset();
  }
}

void SimInputsSystem::Configure(const gz::sim::Entity& entity,
                                const std::shared_ptr<const sdf::Element>& sdf,
                                gz::sim::EntityComponentManager& ecm,
                                gz::sim::EventManager& event_mgr) {
  absl::Status register_status =
      SimInputsServiceSingleton::Get().RegisterServiceImplementation(
          std::weak_ptr<SimInputsService>(sim_inputs_service_));
  if (!register_status.ok()) {
    LOG(ERROR) << "Failed to register SetSimulatedInputsService: "
               << register_status;
  } else {
    LOG(INFO) << "SetSimulatedInputsService is ready to accept calls";
  }
}

void SimInputsSystem::Update(const gz::sim::UpdateInfo& info,
                             gz::sim::EntityComponentManager& ecm) {
  sim_inputs_service_->ProcessRequests(ecm);
}

}  // namespace intrinsic::simulation
