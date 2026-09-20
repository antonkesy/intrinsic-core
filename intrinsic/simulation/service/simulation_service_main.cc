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

// Startup code for the simulation service.
#include <memory>
#include <string>
#include <utility>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "grpc/grpc.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server.h"
#include "grpcpp/server_builder.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/simulation/service/simulation_service_collection.h"
#include "intrinsic/simulation/service/simulation_service_startup.h"

ABSL_FLAG(int, port, 8088, "Port to listen on");
ABSL_FLAG(std::string, resource_registry_address,
          "resource-registry.app-intrinsic-base.svc.cluster.local:8080",
          "gRpc target for the resource registry. Used to query runtime "
          "resource information.");
ABSL_FLAG(std::string, world_service_address,
          "world.app-intrinsic-base.svc.cluster.local:8080",
          "Required gRpc target for the World service.");
ABSL_FLAG(absl::Duration, grpc_connect_timeout,
          intrinsic::connect::kGrpcClientConnectDefaultTimeout,
          "Time to wait for other grpc services to become available.");
ABSL_FLAG(std::string, sim_control_address, "",
          "Simulation control server address. Leave unset if you are setting "
          "`simulator_from_resource_registry` instead.");
ABSL_FLAG(
    std::string, hss_application_service_address, "",
    "gRPC address of the Hot Shared State Application Service. If non-empty, "
    "this is used to determine if an application is currently running and "
    "connect to the simulator deployed as part of the application.");
ABSL_FLAG(
    bool, simulator_from_resource_registry, false,
    "Whether to query the resource registry for a simulator service asset.");

namespace {

// Configures the given ServerBuilder to run the given simulation service by
// listening on the given address.
void ConfigureServer(
    absl::string_view server_address,
    intrinsic::simulation::SimulationServiceCollection* service_collection,
    grpc::ServerBuilder* server_builder) {
  server_builder->AddListeningPort(std::string(server_address),
                                   grpc::InsecureServerCredentials());
  server_builder->AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);

  // Register all versions.
  server_builder->RegisterService(service_collection->first_party());
  server_builder->RegisterService(service_collection->v1());
  server_builder->RegisterService(service_collection->simulator_world_sync());
}

absl::Status RunServerFromServerBuilder(absl::string_view server_address,
                                        grpc::ServerBuilder* server_builder) {
  std::unique_ptr<grpc::Server> server = server_builder->BuildAndStart();
  if (server == nullptr) {
    return absl::InternalError(
        absl::StrCat("Couldn't start simulation service on ", server_address));
  }

  LOG(INFO) << "------------------------------------------------";
  LOG(INFO) << "-- Simulation service listening on " << server_address;
  LOG(INFO) << "------------------------------------------------";

  server->Wait();

  return absl::OkStatus();
}

}  // namespace

int main(int argc, char** argv) {
  ::InitIntrinsic(argv[0], argc, argv);

  intrinsic::PubSub pubsub;
  const intrinsic::simulation::SimulationServiceStartupOptions options{
      .resource_registry_address =
          absl::GetFlag(FLAGS_resource_registry_address),
      .world_service_address = absl::GetFlag(FLAGS_world_service_address),
      .hss_application_service_address =
          absl::GetFlag(FLAGS_hss_application_service_address),
      .sim_control_address = absl::GetFlag(FLAGS_sim_control_address),
      .simulator_from_resource_registry =
          absl::GetFlag(FLAGS_simulator_from_resource_registry),
      .grpc_connect_timeout = absl::GetFlag(FLAGS_grpc_connect_timeout),
      .pubsub = &pubsub,
  };

  auto simulation_service_collection =
      intrinsic::simulation::RunSimulationService(options);
  CHECK_OK(simulation_service_collection.status());

  const std::string server_address =
      absl::StrCat("[::]:", absl::GetFlag(FLAGS_port));
  grpc::ServerBuilder server_builder;
  ConfigureServer(server_address, simulation_service_collection->get(),
                  &server_builder);

  CHECK_OK(RunServerFromServerBuilder(server_address, &server_builder));

  return 0;
}
