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

#ifndef INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_STARTUP_H_
#define INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_STARTUP_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/simulation/service/simulation_service_collection.h"

namespace intrinsic {
namespace simulation {

struct SimulationServiceStartupOptions {
  std::string resource_registry_address;
  std::string world_service_address;
  std::string hss_application_service_address;
  std::string sim_control_address;
  bool simulator_from_resource_registry = false;
  absl::Duration grpc_connect_timeout =
      connect::kGrpcClientConnectDefaultTimeout;
  PubSub* pubsub = nullptr;
};

// Creates and initializes the simulation service collection configured from
// the given options.
absl::StatusOr<std::unique_ptr<SimulationServiceCollection>>
RunSimulationService(const SimulationServiceStartupOptions& options);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_STARTUP_H_
