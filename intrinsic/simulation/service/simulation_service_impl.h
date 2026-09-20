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

#ifndef INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_IMPL_H_
#define INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_IMPL_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/flags/declare.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "google/protobuf/empty.pb.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/sync_stream.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/resources/client/resource_registry_client_interface.h"
#include "intrinsic/simulation/service/proto/first_party/simulation_service.pb.h"
#include "intrinsic/simulation/service/proto/v1/simulator_world_sync.pb.h"
#include "intrinsic/simulation/service/simulation_runtime.h"
#include "intrinsic/storage/hot_shared_state/proto/application_service.grpc.pb.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"
#include "intrinsic/world/service/updater/world_updater.grpc.pb.h"

ABSL_DECLARE_FLAG(bool, use_high_consistency_kv_store_testonly);

namespace intrinsic {
namespace simulation {

// Implementation of the simulation service API.
// Note that the implementation for runtime gRPC calls (i.e. RPCs expected to be
// called when a solution is running) is dispatched out to a `SimulationRuntime`
// instance.
class SimulationServiceImpl {
 public:
  using ObjectWorldServiceClientFactory =
      std::function<absl::StatusOr<std::shared_ptr<
          intrinsic_proto::world::ObjectWorldService::StubInterface>>()>;

  using WorldUpdaterClientFactory = std::function<absl::StatusOr<
      std::shared_ptr<intrinsic_proto::world::WorldUpdater::StubInterface>>()>;

  using ResourceRegistryClientFactory = absl::AnyInvocable<absl::StatusOr<
      std::unique_ptr<resources::ResourceRegistryClientInterface>>()>;

  // Creates a new instance from the given parameters:
  // `create_simulator_client`: A factory method to create instances of the
  //   simulator client.
  // `create_icon_client`: A factory method to create short-lived ICON
  //   Application Layer Clients. Must not be empty/nullptr.
  // `connect_to_object_world_service`: Factory method for creating gRpc stubs
  //   for the Object World service.
  // `connect_to_world_updater_service`: Optional factory method for creating
  //   gRPC stubs for the World Updater service.
  // `hss_application_service_stub`: Optional gRPC stub for the Hot Shared State
  //   Application Service. If non-null, this is used to determine if an
  //   application is currently running, and connect to the simulator deployed
  //   as part of the application.
  // `create_resource_registry_client`: Optional factory method to create a
  //   resource registry client. If present/non-null, this is used to find
  //   ICON server endpoints at runtime.
  // `manual_application_layer_targets`: A list of explicit target addresses for
  //   Application Layer connections, used to replace or supplement those
  //   discovered via the resource registry.
  // `pubsub`: Optional Intrinsic pubsub client. If present, the service will
  //   store its runtime state in the pubsub KV store and read back from it at
  //   startup.
  static absl::StatusOr<std::unique_ptr<SimulationServiceImpl>> Create(
      SimulationRuntime::SimulatorClientFactory
      absl_nonnull create_simulator_client,
      SimulationRuntime::IconClientFactory absl_nonnull create_icon_client,
      ObjectWorldServiceClientFactory
      absl_nonnull connect_to_object_world_service,
      std::unique_ptr<intrinsic_proto::hot_shared_state::v1::
                          HotSharedStateApplicationService::Stub>
          hss_application_service_stub = nullptr,
      ResourceRegistryClientFactory create_resource_registry_client = nullptr,
      WorldUpdaterClientFactory connect_to_world_updater_service = nullptr,
      absl::Span<const ConnectionParams> manual_application_layer_targets = {},
      PubSub* pubsub = nullptr);

  virtual ~SimulationServiceImpl() = default;

  // Disallow copy and move.
  SimulationServiceImpl(SimulationServiceImpl&& other) = delete;
  SimulationServiceImpl& operator=(SimulationServiceImpl&& other) = delete;
  SimulationServiceImpl(const SimulationServiceImpl&) = delete;
  const SimulationServiceImpl& operator=(const SimulationServiceImpl&) = delete;

  absl::Status StartSolution(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::first_party::StartSolutionRequest*
          request,
      intrinsic_proto::simulation::first_party::StartSolutionResponse* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_, stream_context_mutex_);

  absl::Status StopSolution(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::first_party::StopSolutionRequest*
          request,
      intrinsic_proto::simulation::first_party::StopSolutionResponse* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_, stream_context_mutex_);

  absl::Status GetSimulatorName(::grpc::ServerContext* context,
                                const google::protobuf::Empty* request,
                                google::protobuf::StringValue* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_);

  absl::Status ResetSimulation(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::first_party::ResetSimulationRequest*
          request,
      google::protobuf::Empty* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_, stream_context_mutex_);

  absl::Status PauseSimulation(::grpc::ServerContext* context,
                               const google::protobuf::Empty* request,
                               google::protobuf::Empty* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_);

  absl::Status UnpauseSimulation(::grpc::ServerContext* context,
                                 const google::protobuf::Empty* request,
                                 google::protobuf::Empty* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_);

  absl::Status GetSimulationStatus(
      ::grpc::ServerContext* context, const google::protobuf::Empty* request,
      intrinsic_proto::simulation::first_party::GetSimulationStatusResponse*
          response) ABSL_LOCKS_EXCLUDED(runtime_mutex_);

  absl::Status GetSimulatorStatus(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::first_party::GetSimulatorStatusRequest*
          request,
      intrinsic_proto::simulation::first_party::GetSimulatorStatusResponse*
          response) ABSL_LOCKS_EXCLUDED(runtime_mutex_);

  absl::Status RefreshConnectedSimulator(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::first_party::
          RefreshConnectedSimulatorRequest* request,
      intrinsic_proto::simulation::first_party::
          RefreshConnectedSimulatorResponse* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_);

  absl::Status RunVisualization(::grpc::ServerContext* context,
                                const intrinsic_proto::simulation::first_party::
                                    VisualizeWorldUpdatesRequest* request,
                                google::protobuf::Empty* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_);

  absl::Status SyncSimulatorWorld(
      ::grpc::ServerContext* context,
      ::grpc::ServerReaderWriter<
          ::intrinsic_proto::simulation::v1::WorldMessage,
          ::intrinsic_proto::simulation::v1::SimulatorSceneMessage>* stream)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_, stream_context_mutex_);

  absl::Status GetSimulatorWorldInfo(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::simulation::v1::GetSimulatorWorldInfoRequest*
          request,
      ::intrinsic_proto::simulation::v1::SimulatorWorldInfo* response)
      ABSL_LOCKS_EXCLUDED(runtime_mutex_);

 private:
  // See `Create` for parameter details.
  explicit SimulationServiceImpl(
      SimulationRuntime::SimulatorClientFactory create_simulator_client,
      SimulationRuntime::IconClientFactory create_icon_client,
      ObjectWorldServiceClientFactory connect_to_object_world_service,
      WorldUpdaterClientFactory connect_to_world_updater_service,
      ResourceRegistryClientFactory create_resource_registry_client,
      absl::Span<const ConnectionParams> manual_application_layer_targets,
      PubSub* pubsub);

  // Creates the `SimulationRuntime` that services runtime RPCs.
  absl::Status InitializeRuntime(
      std::string_view simulator_world_id, std::string_view start_world_id,
      std::string_view simulator_world_state_updates_topic)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(runtime_mutex_);

  void ForceCancelStream() ABSL_EXCLUSIVE_LOCKS_REQUIRED(stream_context_mutex_);

  SimulationRuntime::SimulatorClientFactory create_simulator_client_;
  SimulationRuntime::IconClientFactory create_icon_client_;
  ObjectWorldServiceClientFactory connect_to_object_world_service_;
  WorldUpdaterClientFactory connect_to_world_updater_service_;
  std::shared_ptr<intrinsic_proto::world::ObjectWorldService::StubInterface>
      object_world_service_;
  ResourceRegistryClientFactory create_resource_registry_client_;
  std::vector<ConnectionParams> manual_application_layer_targets_;

  absl::Mutex runtime_mutex_;
  std::unique_ptr<SimulationRuntime> runtime_ ABSL_GUARDED_BY(runtime_mutex_);

  struct SyncSimulatorWorldStreamData {
    ::grpc::ServerContext* context = nullptr;
    // Assigned only after the initial handshake is completed successfully.
    // If nullopt, the handshake is still in progress.
    std::optional<std::string> simulator_name;
  };

  absl::Mutex stream_context_mutex_;
  std::optional<SyncSimulatorWorldStreamData> active_stream_data_
      ABSL_GUARDED_BY(stream_context_mutex_);

  // Externally owned.
  PubSub* pubsub_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_IMPL_H_
