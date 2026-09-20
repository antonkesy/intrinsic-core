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

#ifndef INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_COLLECTION_H_
#define INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_COLLECTION_H_

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/simulation/service/simulation_runtime.h"
#include "intrinsic/simulation/service/simulation_service_first_party.h"
#include "intrinsic/simulation/service/simulation_service_impl.h"
#include "intrinsic/simulation/service/simulation_service_v1.h"
#include "intrinsic/simulation/service/simulator_world_sync_v1.h"
#include "intrinsic/storage/hot_shared_state/proto/application_service.grpc.pb.h"
#include "intrinsic/util/grpc/connection_params.h"

namespace intrinsic {
namespace simulation {

// Collection of versioned and first-party simulation services with shared
// state.
class SimulationServiceCollection {
 public:
  // See `SimulationServiceImpl::Create` for parameter details.
  static absl::StatusOr<std::unique_ptr<SimulationServiceCollection>> Create(
      SimulationRuntime::SimulatorClientFactory
      absl_nonnull create_simulator_client,
      SimulationRuntime::IconClientFactory absl_nonnull create_icon_client,
      SimulationServiceImpl::ObjectWorldServiceClientFactory
      absl_nonnull connect_to_object_world_service,
      std::unique_ptr<intrinsic_proto::hot_shared_state::v1::
                          HotSharedStateApplicationService::Stub>
          hss_application_service_stub = nullptr,
      SimulationServiceImpl::ResourceRegistryClientFactory
          create_resource_registry_client = nullptr,
      SimulationServiceImpl::WorldUpdaterClientFactory
          connect_to_world_updater_service = nullptr,
      absl::Span<const ConnectionParams> manual_application_layer_targets = {},
      PubSub* pubsub = nullptr);

  SimulationServiceFirstParty* first_party() const {
    return first_party_.get();
  }

  SimulationServiceV1* v1() const { return v1_.get(); }

  SimulatorWorldSyncV1* simulator_world_sync() const {
    return simulator_world_sync_.get();
  }

 private:
  explicit SimulationServiceCollection(
      std::unique_ptr<SimulationServiceImpl> impl,
      std::unique_ptr<SimulationServiceFirstParty> first_party,
      std::unique_ptr<SimulationServiceV1> v1,
      std::unique_ptr<SimulatorWorldSyncV1> simulator_world_sync)
      : impl_(std::move(impl)),
        first_party_(std::move(first_party)),
        v1_(std::move(v1)),
        simulator_world_sync_(std::move(simulator_world_sync)) {}

  // Order is important: first_party_, v1_ and simulator_world_sync_ hold
  // references to impl_.
  const std::unique_ptr<SimulationServiceImpl> impl_;
  const std::unique_ptr<SimulationServiceFirstParty> first_party_;
  const std::unique_ptr<SimulationServiceV1> v1_;
  const std::unique_ptr<SimulatorWorldSyncV1> simulator_world_sync_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_COLLECTION_H_
