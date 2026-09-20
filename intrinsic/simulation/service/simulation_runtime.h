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

#ifndef INTRINSIC_SIMULATION_SERVICE_SIMULATION_RUNTIME_H_
#define INTRINSIC_SIMULATION_SERVICE_SIMULATION_RUNTIME_H_

#include <memory>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_set.h"
#include "absl/functional/any_invocable.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "google/protobuf/empty.pb.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/sync_stream.h"
#include "intrinsic/icon/cc_client/client.h"
#include "intrinsic/resources/client/resource_registry_client_interface.h"
#include "intrinsic/simulation/service/proto/first_party/simulation_service.pb.h"
#include "intrinsic/simulation/service/proto/v1/simulator_world_sync.pb.h"
#include "intrinsic/simulation/service/simulator.h"
#include "intrinsic/simulation/service/simulator_world_manager.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"
#include "intrinsic/world/service/updater/world_updater.grpc.pb.h"

namespace intrinsic {
namespace simulation {

// Internal runtime implementation of the simulation service API.
// Services the RPCs that are available when a solution is running in the
// cluster.
// This class is thread-safe.
class SimulationRuntime {
 public:
  // Type for functions that create ICON Application Layer Clients from target
  // addresses.
  using IconClientFactory = absl::AnyInvocable<absl::StatusOr<icon::Client>(
      const ConnectionParams& connection_params)>;

  using SimulatorClientFactory =
      absl::AnyInvocable<absl::StatusOr<std::unique_ptr<Simulator>>()>;

  // Creates a new instance from the given parameters:
  // `create_simulator_client`: A factory method to create instances of the
  //   simulator client.
  // `create_icon_client`: A factory method to create short-lived ICON
  //   Application Layer Clients. Must not be empty/nullptr.
  // `object_world_service_stub`: Required gRpc stub for the object world
  //   service.
  // `simulator_world_id`: The world from which the simulator scene will be
  //   seeded and sync'ed to.
  // `start_world_id`: Optional world from which the simulator world will be
  //   cloned. Create` will fail if the simulator world cannot be cloned from
  //   the specified world.
  // `simulator_world_state_updates_topic`: Optional topic name to subscribe to
  //   simulator pose updates. Forwarded to any connected simulator over the
  //   SyncSimulatorWorld stream initiation.
  // `resource_registry_client`: Optional resource registry client. If
  //   present/non-null, this is used to find ICON server endpoints at runtime.
  // `manual_application_layer_targets`: A list of explicit target addresses for
  //   Application Layer connections, used to replace or supplement those
  //   discovered via the resource registry.
  static absl::StatusOr<std::unique_ptr<SimulationRuntime>> Create(
      SimulatorClientFactory create_simulator_client,
      IconClientFactory create_icon_client,
      std::shared_ptr<intrinsic_proto::world::ObjectWorldService::
                          StubInterface> absl_nonnull object_world_service,
      std::shared_ptr<
          intrinsic_proto::world::WorldUpdater::StubInterface> absl_nullable
      world_updater_service,
      std::string_view simulator_world_id, std::string_view start_world_id,
      std::string_view simulator_world_state_updates_topic,
      std::unique_ptr<resources::ResourceRegistryClientInterface>
          resource_registry_client = nullptr,
      absl::Span<const ConnectionParams> manual_application_layer_targets = {});

  // Disallow copy and move.
  SimulationRuntime(SimulationRuntime&& other) = delete;
  SimulationRuntime& operator=(SimulationRuntime&& other) = delete;
  SimulationRuntime(const SimulationRuntime&) = delete;
  const SimulationRuntime& operator=(const SimulationRuntime&) = delete;

  std::string_view simulator_world_id() const {
    return simulator_world_manager_->simulator_world_id();
  }

  std::string_view simulator_world_state_updates_topic() const {
    return simulator_world_state_updates_topic_;
  }

  absl::Status GetSimulatorName(::grpc::ServerContext* context,
                                const google::protobuf::Empty* request,
                                google::protobuf::StringValue* response)
      ABSL_LOCKS_EXCLUDED(simulator_mutex_);

  absl::Status ResetSimulation(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::first_party::ResetSimulationRequest*
          request,
      google::protobuf::Empty* response) ABSL_LOCKS_EXCLUDED(simulator_mutex_);

  absl::Status PauseSimulation(::grpc::ServerContext* context,
                               const google::protobuf::Empty* request,
                               google::protobuf::Empty* response)
      ABSL_LOCKS_EXCLUDED(simulator_mutex_);

  absl::Status UnpauseSimulation(::grpc::ServerContext* context,
                                 const google::protobuf::Empty* request,
                                 google::protobuf::Empty* response)
      ABSL_LOCKS_EXCLUDED(simulator_mutex_);

  absl::Status GetSimulationStatus(
      ::grpc::ServerContext* context, const google::protobuf::Empty* request,
      intrinsic_proto::simulation::first_party::GetSimulationStatusResponse*
          response) ABSL_LOCKS_EXCLUDED(simulator_mutex_);

  absl::Status GetSimulatorStatus(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::first_party::GetSimulatorStatusRequest*
          request,
      intrinsic_proto::simulation::first_party::GetSimulatorStatusResponse*
          response) ABSL_LOCKS_EXCLUDED(simulator_mutex_);

  absl::Status RefreshConnectedSimulator(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::first_party::
          RefreshConnectedSimulatorRequest* request,
      intrinsic_proto::simulation::first_party::
          RefreshConnectedSimulatorResponse* response)
      ABSL_LOCKS_EXCLUDED(simulator_mutex_);

  absl::Status RunVisualization(::grpc::ServerContext* context,
                                const intrinsic_proto::simulation::first_party::
                                    VisualizeWorldUpdatesRequest* request,
                                google::protobuf::Empty* response);

  absl::Status SyncSimulatorWorld(
      ::grpc::ServerContext* absl_nonnull context,
      ::grpc::ServerReaderWriter<
          ::intrinsic_proto::simulation::v1::WorldMessage,
          ::intrinsic_proto::simulation::v1::
              SimulatorSceneMessage>* absl_nonnull stream,
      const ::intrinsic_proto::simulation::v1::SimulatorSceneMessage&
          first_msg);

  absl::Status GetSimulatorWorldInfo(
      ::grpc::ServerContext* absl_nonnull context,
      const ::intrinsic_proto::simulation::v1::
          GetSimulatorWorldInfoRequest* absl_nonnull request,
      ::intrinsic_proto::simulation::v1::SimulatorWorldInfo* absl_nonnull
          response);

 private:
  explicit SimulationRuntime(
      absl::StatusOr<std::unique_ptr<Simulator>> simulator,
      SimulatorClientFactory create_simulator_client,
      std::unique_ptr<SimulatorWorldManager> simulator_world_manager,
      IconClientFactory create_icon_client,
      std::shared_ptr<intrinsic_proto::world::ObjectWorldService::StubInterface>
          object_world_service,
      std::string_view simulator_world_state_updates_topic,
      std::unique_ptr<resources::ResourceRegistryClientInterface>
          resource_registry_client,
      absl::Span<const ConnectionParams> manual_application_layer_targets);

  // Returns a set parameters to connect to all running ICON instances in the
  // cluster which offer an Application Layer interface.
  absl::StatusOr<absl::flat_hash_set<ConnectionParams>> GetIconInstances()
      const;

  // Clears faults on all ICON instances.
  absl::Status ClearIconFaults(const ::grpc::ServerContext& context);

  // Restarts all ICON instances.
  absl::Status RestartIconServers(const ::grpc::ServerContext& context);

  // Resets the simulator with retries.
  absl::Status ResetSimulatorWithRetries(::grpc::ServerContext* context,
                                         bool start_paused,
                                         int num_reset_retries)
      ABSL_SHARED_LOCKS_REQUIRED(simulator_mutex_);

  template <typename ResponseType>
  absl::Status GetSimulatorStatusInternal(ResponseType* response)
      ABSL_LOCKS_EXCLUDED(simulator_mutex_);

  mutable absl::Mutex simulator_mutex_;
  absl::StatusOr<std::unique_ptr<Simulator>> simulator_
      ABSL_GUARDED_BY(simulator_mutex_);
  SimulatorClientFactory create_simulator_client_;
  const std::string simulator_world_state_updates_topic_;
  IconClientFactory create_icon_client_;
  const std::shared_ptr<
      intrinsic_proto::world::ObjectWorldService::StubInterface>
      object_world_service_;
  const std::unique_ptr<resources::ResourceRegistryClientInterface>
      resource_registry_client_;
  const std::vector<ConnectionParams> manual_application_layer_targets_;

  std::unique_ptr<SimulatorWorldManager> simulator_world_manager_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_SERVICE_SIMULATION_RUNTIME_H_
