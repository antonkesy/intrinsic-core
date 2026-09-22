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

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server_builder.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/extended_status_codes.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/proto_registry_service.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/registry_pubsub.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver_asset.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver_common.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver_skill.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/skills/internal/skill_registry_client.h"
#include "intrinsic/stats/opencensus.h"
#include "intrinsic/util/path_resolver/path_resolver.h"
#include "intrinsic/util/status/return.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_specs.h"
#include "intrinsic/util/thread/thread.h"

ABSL_FLAG(int32_t, port, 8080, "Port to listen on for gRPC service.");
ABSL_FLAG(absl::Duration, shutdown_grace_period, absl::Seconds(10),
          "Time to wait for graceful cancellation on shutdown");

ABSL_FLAG(std::string, skill_registry_address, "",
          "Address of the skill registry service in the form 'host:port'.");
ABSL_FLAG(std::string, installed_assets_service_address, "",
          "Address of the installed assets service in the form 'host:port'.");

namespace {
// std::atomic is safe, as long as it is lock-free
std::atomic<bool> g_shutdown_requested = false;
static_assert(std::atomic<bool>::is_always_lock_free);
}  // namespace

absl::Status Run() {
  const std::string server_address =
      absl::StrFormat("[::]:%d", absl::GetFlag(FLAGS_port));

  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address,
                           grpc::InsecureServerCredentials());  // NOLINT
  // "0" means no port reuse. Allowing other servers on the same port could
  // introduce hard-to-debug behavior or flaky tests.
  builder.AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);

  // Service dependency initialization
  INTR_ASSIGN_OR_RETURN(std::shared_ptr<intrinsic::skills::SkillRegistryClient>
                            skill_registry_client,
                        intrinsic::skills::CreateSkillRegistryClient(
                            absl::GetFlag(FLAGS_skill_registry_address)),
                        _ << "Failed to create skill registry client.");

  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> installed_assets_channel,
      intrinsic::connect::CreateClientChannel(
          absl::GetFlag(FLAGS_installed_assets_service_address),
          absl::Now() + intrinsic::connect::kGrpcClientConnectDefaultTimeout,
          intrinsic::connect::DefaultGrpcChannelArgs()));
  std::shared_ptr<
      intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface>
      installed_assets_stub =
          intrinsic_proto::assets::v1::InstalledAssetsReader::NewStub(
              installed_assets_channel);

  INTR_ASSIGN_OR_RETURN(auto pubsub_registry,
                        intrinsic::proto_registry::PubSubRegistry::Create(),
                        _ << "Failed to create pubsub registry");

  std::shared_ptr<intrinsic::proto_registry::SkillRegistryResolver>
      skill_resolver = intrinsic::proto_registry::SkillRegistryResolver::Create(
          skill_registry_client);
  std::shared_ptr<intrinsic::proto_registry::AssetResolver> asset_resolver =
      intrinsic::proto_registry::AssetResolver::Create(installed_assets_stub);
  std::shared_ptr<intrinsic::proto_registry::Resolver> pubsub_resolver =
      pubsub_registry->GetResolver();
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<intrinsic::proto_registry::CommonResolver>
          common_resolver,
      intrinsic::proto_registry::CommonResolver::Create(),
      _ << "Failed to create common resolver.");

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<intrinsic::proto_registry::ProtoResolverMap> resolver,
      intrinsic::proto_registry::ProtoResolverMap::Create(
          {skill_resolver, asset_resolver, pubsub_resolver, common_resolver}),
      _ << "Failed to create resolver map.");

  // Service creation and registration
  INTR_ASSIGN_OR_RETURN(
      auto proto_registry_service,
      intrinsic::proto_registry::ProtoRegistryService::CreateService(
          std::move(resolver)),
      _ << "Failed starting proto registry service.");
  builder.RegisterService(proto_registry_service.get());

  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  LOG_IF(QFATAL, server == nullptr)
      << "Cannot create ProtoRegistry server " << server_address;
  LOG(INFO) << "ProtoRegistry server listening on " << server_address;

  std::signal(SIGTERM, [](int) {
    // async-signal-safe implementation
    // for details, see here:
    // https://man7.org/linux/man-pages/man7/signal-safety.7.html
    // Note: To prevent undefined behavior, do not do any logging (unless using
    //   async-safe write) or else here without making sure that the function is
    //   async-signal-safe.
    g_shutdown_requested = true;
    g_shutdown_requested.notify_all();
  });

  intrinsic::Thread shutdown([&]() {
    g_shutdown_requested.wait(/*old=*/false);
    LOG(INFO) << "Shutdown requested";
    if (server->GetHealthCheckService()) {
      server->GetHealthCheckService()->SetServingStatus(false);
    }
    proto_registry_service->Shutdown();
    // Give grace period to gracefully cancel operations
    absl::SleepFor(absl::GetFlag(FLAGS_shutdown_grace_period));
    server->Shutdown();
  });

  server->Wait();
  shutdown.join();
  return absl::OkStatus();
}

int main(int argc, char* argv[]) {
  InitIntrinsic(argv[0], argc, argv);
  intrinsic::OpenCensusPlugin open_census;

  INTR_RETURN_IF_ERROR(intrinsic::InitExtendedStatusSpecs(
                           intrinsic::proto_registry::kExtendedStatusComponent,
                           intrinsic::PathResolver::ResolveRunfilesPath(
                               intrinsic::proto_registry::kExtendedStatusFile)))
      .With(intrinsic::ExtraMessage()
            << "Failed to initialize extended status specs.")
      .With(intrinsic::Return(EXIT_FAILURE));

  INTR_RETURN_IF_ERROR(Run()).LogError().With(intrinsic::Return(EXIT_FAILURE));

  return EXIT_SUCCESS;
}
