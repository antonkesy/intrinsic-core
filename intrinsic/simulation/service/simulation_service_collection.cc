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

#include "intrinsic/simulation/service/simulation_service_collection.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/simulation/service/simulation_runtime.h"
#include "intrinsic/simulation/service/simulation_service_first_party.h"
#include "intrinsic/simulation/service/simulation_service_impl.h"
#include "intrinsic/simulation/service/simulation_service_v1.h"
#include "intrinsic/simulation/service/simulator_world_sync_v1.h"
#include "intrinsic/storage/hot_shared_state/proto/application_service.grpc.pb.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace simulation {

absl::StatusOr<std::unique_ptr<SimulationServiceCollection>>
SimulationServiceCollection::Create(
    SimulationRuntime::SimulatorClientFactory create_simulator_client,
    SimulationRuntime::IconClientFactory create_icon_client,
    SimulationServiceImpl::ObjectWorldServiceClientFactory
        connect_to_object_world_service,
    std::unique_ptr<intrinsic_proto::hot_shared_state::v1::
                        HotSharedStateApplicationService::Stub>
        hss_application_service_stub,
    SimulationServiceImpl::ResourceRegistryClientFactory
        create_resource_registry_client,
    SimulationServiceImpl::WorldUpdaterClientFactory
        connect_to_world_updater_service,
    absl::Span<const ConnectionParams> manual_application_layer_targets,
    PubSub* pubsub) {
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<SimulationServiceImpl> impl,
      SimulationServiceImpl::Create(std::move(create_simulator_client),
                                    std::move(create_icon_client),
                                    std::move(connect_to_object_world_service),
                                    std::move(hss_application_service_stub),
                                    std::move(create_resource_registry_client),
                                    std::move(connect_to_world_updater_service),
                                    manual_application_layer_targets, pubsub));
  auto first_party = std::make_unique<SimulationServiceFirstParty>(impl.get());
  auto v1 = std::make_unique<SimulationServiceV1>(impl.get());
  auto simulator_world_sync =
      std::make_unique<SimulatorWorldSyncV1>(impl.get());
  return absl::WrapUnique(new SimulationServiceCollection(
      std::move(impl), std::move(first_party), std::move(v1),
      std::move(simulator_world_sync)));
}

}  // namespace simulation
}  // namespace intrinsic
