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

#include "intrinsic/simulation/service/proto/conversions.h"

#include "intrinsic/simulation/service/proto/first_party/simulation_service.pb.h"
#include "intrinsic/simulation/service/proto/v1/simulation_service.pb.h"
#include "intrinsic/simulation/simulator/proto/v1/simulator_control_service.pb.h"

namespace intrinsic {
namespace simulation {

intrinsic_proto::simulation::first_party::ResetSimulationRequest FromV1(
    const intrinsic_proto::simulation::v1::ResetSimulationRequest& request) {
  return {};
}

intrinsic_proto::simulation::first_party::SimulatorState FromSimulatorV1(
    intrinsic_proto::simulation::v1::SimulatorState v1_state) {
  switch (v1_state) {
    case intrinsic_proto::simulation::v1::SimulatorState::
        SIMULATOR_STATE_UNSPECIFIED:
      return intrinsic_proto::simulation::first_party::
          SIMULATOR_STATE_UNSPECIFIED;
    case intrinsic_proto::simulation::v1::SimulatorState::RUNNING:
      return intrinsic_proto::simulation::first_party::RUNNING;
    case intrinsic_proto::simulation::v1::SimulatorState::PAUSED:
      return intrinsic_proto::simulation::first_party::PAUSED;
    case intrinsic_proto::simulation::v1::SimulatorState::STOPPED:
      return intrinsic_proto::simulation::first_party::STOPPED;
    default:
      return intrinsic_proto::simulation::first_party::
          SIMULATOR_STATE_UNSPECIFIED;
  }
}

intrinsic_proto::simulation::first_party::GetSimulationStatusResponse
FromSimulatorV1(
    const intrinsic_proto::simulation::v1::GetSimulatorStatusResponse&
        response) {
  intrinsic_proto::simulation::first_party::GetSimulationStatusResponse
      simulation_status_response;
  simulation_status_response.set_simulator_state(
      FromSimulatorV1(response.simulator_state()));
  if (response.has_status()) {
    *simulation_status_response.mutable_status() = response.status();
  }
  return simulation_status_response;
}

}  // namespace simulation
}  // namespace intrinsic
