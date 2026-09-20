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

#include "intrinsic/simulation/service/simulation_service_impl.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "google/protobuf/empty.pb.h"
#include "google/protobuf/wrappers.pb.h"
#include "grpcpp/client_context.h"
#include "intrinsic/config/proto/application.pb.h"
#include "intrinsic/config/proto/operation_mode.pb.h"
#include "intrinsic/resources/client/resource_registry_client_interface.h"
#include "intrinsic/simulation/service/proto/first_party/simulation_service.pb.h"
#include "intrinsic/simulation/service/runtime_state.pb.h"
#include "intrinsic/simulation/service/simulation_runtime.h"
#include "intrinsic/storage/hot_shared_state/proto/application_service.grpc.pb.h"
#include "intrinsic/storage/hot_shared_state/proto/application_service.pb.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

ABSL_FLAG(bool, use_high_consistency_kv_store_testonly, true,
          "Whether or not to use high consistency for writes to the KV store. "
          "Set to false in tests where the KV store may not be available to "
          "avoid long timeouts causing test slowdown.");

namespace intrinsic {
namespace simulation {

namespace {
constexpr std::string_view kNoRuntimeErrorMessage =
    "Solution is not running in simulation. Try re-deploying the solution in "
    "simulation, and file a support ticket if the error persists.";

constexpr std::string_view kRuntimeStateKVStoreKey =
    "simulation_service_runtime_state";

// The fallback simulator world id and state updates topic name that are used
// only if both the following conditions are true:
// - detected at service startup that a solution is already running (so a
// runtime must be initialized), and
// - the cached values cannot be recovered from the KV store.
constexpr std::string_view kFallbackDefaultSimulatorWorldId = "sim_world";
constexpr std::string_view kFallbackDefaultSimulatorStateUpdatesTopicName =
    "/worlds/sim_world/world_updates";

absl::StatusOr<::intrinsic_proto::config::Application> GetApplication(
    intrinsic_proto::hot_shared_state::v1::HotSharedStateApplicationService::
        StubInterface& hss_application_service_stub) {
  grpc::ClientContext context;
  intrinsic_proto::hot_shared_state::v1::GetCurrentApplicationRequest request;
  intrinsic_proto::hot_shared_state::v1::GetCurrentApplicationResponse response;
  INTR_RETURN_IF_ERROR(
      ToAbslStatus(hss_application_service_stub.GetCurrentApplication(
          &context, request, &response)));
  return response.application();
}

absl::Status SaveRuntimeStateToKVStore(
    PubSub& pubsub, std::string_view key,
    const intrinsic_proto::simulation::internal::RuntimeState& runtime_state) {
  INTR_ASSIGN_OR_RETURN(KeyValueStore kvstore, pubsub.KeyValueStore());

  const bool kHighConsistency =
      absl::GetFlag(FLAGS_use_high_consistency_kv_store_testonly);
  INTR_RETURN_IF_ERROR(kvstore.Set(key, runtime_state, kHighConsistency))
      .LogError();
  LOG(INFO) << "Saved runtime state to KV store at [" << key << "]";
  return absl::OkStatus();
}

absl::StatusOr<intrinsic_proto::simulation::internal::RuntimeState>
GetRuntimeStateFromKVStore(PubSub* pubsub, std::string_view key) {
  if (pubsub == nullptr) {
    return absl::FailedPreconditionError("Pubsub was not initialized.");
  }
  INTR_ASSIGN_OR_RETURN(KeyValueStore kvstore, pubsub->KeyValueStore());
  return kvstore.Get<intrinsic_proto::simulation::internal::RuntimeState>(key);
}

absl::Status ClearKVStore(PubSub& pubsub, std::string_view key) {
  INTR_ASSIGN_OR_RETURN(KeyValueStore kvstore, pubsub.KeyValueStore());
  INTR_RETURN_IF_ERROR(kvstore.Delete(key));
  LOG(INFO) << "Deleted KV store entry at [" << key << "]";
  return absl::OkStatus();
}

}  // namespace

// static
absl::StatusOr<std::unique_ptr<SimulationServiceImpl>>
SimulationServiceImpl::Create(
    SimulationRuntime::SimulatorClientFactory create_simulator_client,
    SimulationRuntime::IconClientFactory create_icon_client,
    ObjectWorldServiceClientFactory connect_to_object_world_service,
    std::unique_ptr<intrinsic_proto::hot_shared_state::v1::
                        HotSharedStateApplicationService::Stub>
        hss_application_service_stub,
    ResourceRegistryClientFactory create_resource_registry_client,
    WorldUpdaterClientFactory connect_to_world_updater_service,
    absl::Span<const ConnectionParams> manual_application_layer_targets,
    PubSub* pubsub) {
  auto simulation_service_impl = absl::WrapUnique(new SimulationServiceImpl(
      std::move(create_simulator_client), std::move(create_icon_client),
      std::move(connect_to_object_world_service),
      std::move(connect_to_world_updater_service),
      std::move(create_resource_registry_client),
      manual_application_layer_targets, pubsub));

  // Query the HSS Application service to check if a solution is already
  // deployed. If a solution is deployed in sim, connect to the simulator right
  // away. Otherwise wait for `StartSolution` to be called in sim before
  // connecting to the simulator.
  if (hss_application_service_stub != nullptr) {
    auto application = GetApplication(*hss_application_service_stub.get());
    if (absl::IsNotFound(application.status())) {
      LOG(INFO) << "No application is currently running in the cluster.";
    } else if (!application.ok()) {
      INTR_RETURN_IF_ERROR(application.status())
          << "Error querying current application from HSS.";
    } else if (application->operation_mode() ==
               intrinsic_proto::config::OperationMode::SIMULATION) {
      std::string resolved_simulator_world_id{kFallbackDefaultSimulatorWorldId};
      std::string resolved_sim_world_state_updates_topic{
          kFallbackDefaultSimulatorStateUpdatesTopicName};

      // If we are recovering from a crash, then the simulator world id and
      // pubsub topic should have been cached to the KV store. We attempt to
      // initialize from the cached entry on a best-effort basis. Otherwise we
      // start up with fallback values.
      if (auto runtime_state =
              GetRuntimeStateFromKVStore(pubsub, kRuntimeStateKVStoreKey);
          runtime_state.ok()) {
        resolved_simulator_world_id = runtime_state->simulator_world_id();
        resolved_sim_world_state_updates_topic =
            runtime_state->simulator_world_state_updates_pubsub_topic_name();
      } else {
        LOG(WARNING) << "Failed to restore runtime state from KV store: "
                     << runtime_state.status()
                     << ". Using fallback default world id ["
                     << kFallbackDefaultSimulatorWorldId << "] and topic ["
                     << kFallbackDefaultSimulatorStateUpdatesTopicName << "].";
      }

      // Initialize the runtime from the resolved simulator world id and topic.
      // Since the application is already running, a simulator world should have
      // been created already. So we don't need it to clone it from a start
      // world.
      absl::MutexLock l(simulation_service_impl->runtime_mutex_);
      INTR_RETURN_IF_ERROR(simulation_service_impl->InitializeRuntime(
          resolved_simulator_world_id, /*start_world_id=*/"",
          resolved_sim_world_state_updates_topic));
    }
  }

  return std::move(simulation_service_impl);
}

SimulationServiceImpl::SimulationServiceImpl(
    SimulationRuntime::SimulatorClientFactory create_simulator_client,
    SimulationRuntime::IconClientFactory create_icon_client,
    ObjectWorldServiceClientFactory connect_to_object_world_service,
    WorldUpdaterClientFactory connect_to_world_updater_service,
    ResourceRegistryClientFactory create_resource_registry_client,
    absl::Span<const ConnectionParams> manual_application_layer_targets,
    PubSub* pubsub)
    : create_simulator_client_(std::move(create_simulator_client)),
      create_icon_client_(std::move(create_icon_client)),
      connect_to_object_world_service_(
          std::move(connect_to_object_world_service)),
      connect_to_world_updater_service_(
          std::move(connect_to_world_updater_service)),
      create_resource_registry_client_(
          std::move(create_resource_registry_client)),
      manual_application_layer_targets_(
          manual_application_layer_targets.begin(),
          manual_application_layer_targets.end()),
      pubsub_(pubsub) {}

absl::Status SimulationServiceImpl::InitializeRuntime(
    std::string_view simulator_world_id, std::string_view start_world_id,
    std::string_view simulator_world_state_updates_topic) {
  runtime_mutex_.AssertHeld();

  std::unique_ptr<resources::ResourceRegistryClientInterface>
      resource_registry_client;
  if (create_resource_registry_client_ != nullptr) {
    INTR_ASSIGN_OR_RETURN(resource_registry_client,
                          create_resource_registry_client_());
  }

  std::shared_ptr<intrinsic_proto::world::ObjectWorldService::StubInterface>
      object_world_service;
  INTR_ASSIGN_OR_RETURN(object_world_service,
                        connect_to_object_world_service_());

  std::shared_ptr<intrinsic_proto::world::WorldUpdater::StubInterface>
      world_updater_service = nullptr;
  if (connect_to_world_updater_service_ != nullptr) {
    INTR_ASSIGN_OR_RETURN(world_updater_service,
                          connect_to_world_updater_service_());
  }

  INTR_ASSIGN_OR_RETURN(
      runtime_, SimulationRuntime::Create(
                    /*create_simulator_client=*/
                    [this]() { return create_simulator_client_(); },
                    /*create_icon_client=*/
                    [this](const ConnectionParams& connection_params) {
                      return create_icon_client_(connection_params);
                    },
                    std::move(object_world_service),
                    std::move(world_updater_service), simulator_world_id,
                    start_world_id, simulator_world_state_updates_topic,
                    std::move(resource_registry_client),
                    manual_application_layer_targets_));
  return absl::OkStatus();
}

absl::Status SimulationServiceImpl::StartSolution(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::StartSolutionRequest*
        request,
    intrinsic_proto::simulation::first_party::StartSolutionResponse* response) {
  // Cancel any active SyncWorld stream and guard against stream creation while
  // the runtime is being re-created by holding stream_context_mutex_ for the
  // entire duration of the RPC.
  absl::MutexLock stream_lock(stream_context_mutex_);
  LOG(INFO) << "StartSolution: Proactively cancelling active SyncWorld stream "
               "context.";
  ForceCancelStream();

  absl::MutexLock runtime_lock(runtime_mutex_);

  bool is_sim_deployment = request->operation_mode() ==
                           intrinsic_proto::config::OperationMode::SIMULATION;
  LOG(INFO) << "StartSolution called in "
            << (is_sim_deployment ? "sim" : "real hardware");

  if (is_sim_deployment) {
    if (!request->has_simulation_env()) {
      return InvalidArgumentErrorBuilder().LogError()
             << "Called StartSolution in sim operation mode, but simulation "
                "env is not set.";
    }

    std::string_view simulator_world_id =
        request->simulation_env().simulator_world_id();
    if (simulator_world_id.empty()) {
      return absl::InvalidArgumentError(
          "Simulator world id must be specified in sim.");
    }
    std::string_view start_world_id =
        request->simulation_env().start_world_id();
    if (start_world_id.empty()) {
      return absl::InvalidArgumentError(
          "Start world id must be specified in sim.");
    }

    std::string_view simulator_world_state_updates_topic =
        request->simulation_env()
            .simulator_world_state_updates_pubsub_topic_name();
    if (simulator_world_state_updates_topic.empty()) {
      return absl::InvalidArgumentError(
          "Simulator world state updates pubsub topic name must be specified "
          "in sim.");
    }

    INTR_RETURN_IF_ERROR(
        InitializeRuntime(simulator_world_id, start_world_id,
                          simulator_world_state_updates_topic));

    // Cache runtime state to KV store to recover from potential crashes.
    if (pubsub_ != nullptr) {
      intrinsic_proto::simulation::internal::RuntimeState runtime_state;
      runtime_state.set_simulator_world_id(simulator_world_id);
      runtime_state.set_simulator_world_state_updates_pubsub_topic_name(
          simulator_world_state_updates_topic);
      if (auto status = SaveRuntimeStateToKVStore(
              *pubsub_, kRuntimeStateKVStoreKey, runtime_state);
          !status.ok()) {
        LOG(ERROR) << "Failed to save runtime state to KV store: " << status
                   << ". Crash recovery may fail in future.";
      }
    }
  } else {
    // Real hardware deployment
    LOG_IF(INFO, runtime_ != nullptr) << "Removing existing runtime.";
    runtime_.reset();
    if (pubsub_ != nullptr) {
      if (auto status = ClearKVStore(*pubsub_, kRuntimeStateKVStoreKey);
          !status.ok()) {
        LOG(ERROR) << "Failed to clear runtime state in KV store: " << status
                   << ". Still proceeding with deployment.";
      }
    }
  }
  return absl::OkStatus();
}

absl::Status SimulationServiceImpl::StopSolution(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::StopSolutionRequest*
        request,
    intrinsic_proto::simulation::first_party::StopSolutionResponse* response) {
  // Cancel any active SyncWorld stream and guard against stream creation while
  // the runtime is being stopped by holding stream_context_mutex_ for the
  // entire duration of the RPC.
  absl::MutexLock stream_lock(stream_context_mutex_);
  LOG(INFO) << "StopSolution: Proactively cancelling active SyncWorld stream "
               "context.";
  ForceCancelStream();

  absl::MutexLock runtime_lock(runtime_mutex_);
  LOG(INFO) << "StopSolution called.";
  runtime_.reset();
  if (pubsub_ != nullptr) {
    return ClearKVStore(*pubsub_, kRuntimeStateKVStoreKey);
  }
  return absl::OkStatus();
}

absl::Status SimulationServiceImpl::GetSimulatorName(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    google::protobuf::StringValue* response) {
  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::FailedPreconditionError(kNoRuntimeErrorMessage);
  };

  return runtime_->GetSimulatorName(context, request, response);
}

absl::Status SimulationServiceImpl::ResetSimulation(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::ResetSimulationRequest*
        request,
    google::protobuf::Empty* response) {
  // Cancel any active SyncWorld stream and guard against stream creation while
  // the simulator is being reset by holding stream_context_mutex_ for the
  // entire duration of the RPC.
  absl::MutexLock stream_lock(stream_context_mutex_);
  LOG(INFO) << "ResetSimulation: Proactively cancelling active SyncWorld "
               "stream context.";
  ForceCancelStream();

  absl::MutexLock runtime_lock(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::FailedPreconditionError(kNoRuntimeErrorMessage);
  };

  return runtime_->ResetSimulation(context, request, response);
}

absl::Status SimulationServiceImpl::PauseSimulation(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    google::protobuf::Empty* response) {
  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::FailedPreconditionError(kNoRuntimeErrorMessage);
  };

  return runtime_->PauseSimulation(context, request, response);
}

absl::Status SimulationServiceImpl::UnpauseSimulation(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    google::protobuf::Empty* response) {
  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::FailedPreconditionError(kNoRuntimeErrorMessage);
  };

  return runtime_->UnpauseSimulation(context, request, response);
}

absl::Status SimulationServiceImpl::GetSimulationStatus(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    intrinsic_proto::simulation::first_party::GetSimulationStatusResponse*
        response) {
  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::FailedPreconditionError(kNoRuntimeErrorMessage);
  };

  return runtime_->GetSimulationStatus(context, request, response);
}

absl::Status SimulationServiceImpl::GetSimulatorStatus(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::GetSimulatorStatusRequest*
        request,
    intrinsic_proto::simulation::first_party::GetSimulatorStatusResponse*
        response) {
  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::OkStatus();
  }
  return runtime_->GetSimulatorStatus(context, request, response);
}

absl::Status SimulationServiceImpl::RefreshConnectedSimulator(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::
        RefreshConnectedSimulatorRequest* request,
    intrinsic_proto::simulation::first_party::RefreshConnectedSimulatorResponse*
        response) {
  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::OkStatus();
  }
  return runtime_->RefreshConnectedSimulator(context, request, response);
}

absl::Status SimulationServiceImpl::RunVisualization(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::
        VisualizeWorldUpdatesRequest* request,
    google::protobuf::Empty* response) {
  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::FailedPreconditionError(kNoRuntimeErrorMessage);
  };

  return runtime_->RunVisualization(context, request, response);
}

absl::Status SimulationServiceImpl::SyncSimulatorWorld(
    ::grpc::ServerContext* context,
    ::grpc::ServerReaderWriter<
        ::intrinsic_proto::simulation::v1::WorldMessage,
        ::intrinsic_proto::simulation::v1::SimulatorSceneMessage>* stream) {
  {
    absl::MutexLock l(stream_context_mutex_);
    auto handshake_not_in_progress = [this]() {
      stream_context_mutex_.AssertReaderHeld();
      return !active_stream_data_.has_value() ||
             active_stream_data_->simulator_name.has_value();
    };
    stream_context_mutex_.Await(absl::Condition(&handshake_not_in_progress));

    if (active_stream_data_.has_value()) {
      return absl::ResourceExhaustedError(absl::StrCat(
          "Another SyncWorld stream is already active (created by ",
          active_stream_data_->simulator_name.value_or("unknown"), ")."));
    }
    active_stream_data_ = SyncSimulatorWorldStreamData{
        .context = context,
        .simulator_name = std::nullopt,
    };
  }

  auto cleanup = absl::MakeCleanup([this, context]() {
    absl::MutexLock l(stream_context_mutex_);
    // We must verify the context matches before clearing. A solution transition
    // (StartSolution/StopSolution/ResetSimulation) might have proactively
    // cancelled this stream and cleared `active_stream_data_`, allowing a NEW
    // stream to take its place while this old stream is still shutting down.
    // We don't want to accidentally clear the new stream's data.
    if (active_stream_data_.has_value() &&
        active_stream_data_->context == context) {
      active_stream_data_ = std::nullopt;
    }
  });

  ::intrinsic_proto::simulation::v1::SimulatorSceneMessage first_msg;
  if (!stream->Read(&first_msg)) {
    return absl::InvalidArgumentError(
        "Failed to read first message from stream.");
  }

  if (!first_msg.has_session_init()) {
    return absl::InvalidArgumentError("First message must be session_init.");
  }

  {
    absl::MutexLock l(stream_context_mutex_);
    // Similar to the cleanup block, we must verify the context matches. If this
    // stream was cancelled by a transition during the blocking Read() above,
    // a new stream might already be active.
    if (!active_stream_data_.has_value() ||
        active_stream_data_->context != context) {
      return absl::AbortedError(
          "SyncWorld stream was superseded or cancelled.");
    }
    active_stream_data_->simulator_name =
        first_msg.session_init().simulator_name();
  }

  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::FailedPreconditionError(kNoRuntimeErrorMessage);
  }
  return runtime_->SyncSimulatorWorld(context, stream, first_msg);
}

absl::Status SimulationServiceImpl::GetSimulatorWorldInfo(
    ::grpc::ServerContext* context,
    const ::intrinsic_proto::simulation::v1::GetSimulatorWorldInfoRequest*
        request,
    ::intrinsic_proto::simulation::v1::SimulatorWorldInfo* response) {
  absl::ReaderMutexLock l(runtime_mutex_);
  if (runtime_ == nullptr) {
    return absl::FailedPreconditionError(kNoRuntimeErrorMessage);
  }
  return runtime_->GetSimulatorWorldInfo(context, request, response);
}

void SimulationServiceImpl::ForceCancelStream() {
  stream_context_mutex_.AssertHeld();
  if (active_stream_data_.has_value()) {
    active_stream_data_->context->TryCancel();
    // Setting active_stream_data_ to nullopt here is safe without waiting for
    // the stream thread to clean it up. The old stream's cleanup block verifies
    // `active_stream_data_->context == context`, so it will correctly ignore
    // this (or any subsequent new stream's data) when it runs. This guarantees
    // that when stream_context_mutex_ is released, a new stream can immediately
    // start without hitting a ResourceExhaustedError.
    active_stream_data_ = std::nullopt;
  }
}

}  // namespace simulation
}  // namespace intrinsic
