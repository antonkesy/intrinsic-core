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

#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#include <csignal>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/flags.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/server.h"
#include "grpcpp/server_builder.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/icon/hal/hardware_module_main_util.h"
#include "intrinsic/icon/hal/hardware_module_runtime.h"
#include "intrinsic/icon/hal/hardware_module_util.h"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/icon/hardware_modules/gazebo_hwm_stub/health_forwarder.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/icon/utils/shutdown_signals.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.grpc.pb.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/memory_lock.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

ABSL_FLAG(std::string, module_config_file, "",
          "Module prototext configuration file path.");
ABSL_FLAG(bool, realtime, false,
          "Indicating whether we run on a privileged RTPC.");
ABSL_FLAG(std::string, runtime_context_file, "/etc/intrinsic/runtime_config.pb",
          "The path to the runtime context file containing "
          "intrinsic_proto.config.RuntimeContext binary proto.");
ABSL_FLAG(std::optional<int>, realtime_core, std::nullopt,
          "The CPU core for all realtime threads. Is read from /proc/cmdline "
          "if not defined.");
ABSL_FLAG(std::string, shared_memory_namespace_testonly, "",
          "Prefix for all shared memory connections. Passing unique namespace "
          "is needed to make integration tests hermetic.");
ABSL_FLAG(
    std::optional<int>, grpc_server_port, std::nullopt,
    "The port to use for the grpc server. Only used if not in resource mode.");
ABSL_FLAG(std::optional<int>, aggregated_resource_health_port_testonly,
          std::nullopt,
          "The address that the AggregatedResourceHealth service runs on. The "
          "hostname for this service is `simulation_server_address` in the "
          "HardwareModuleConfig proto within the RuntimeContext. Only use this "
          "to inject a service address in tests (where we pick available ports "
          "at random). In real use, the port is always fixed");

namespace intrinsic::icon {

constexpr const char* kUsageString = R"(
Usage: my_hardware_module --module_config_file=<path> [--realtime] [--realtime_core=5] [--grpc_server_port=<port>]

Starts the hardware module and runs its realtime update loop.

If --realtime is specified, the update loop runs in a thread with realtime
priority. Otherwise, it runs in a normal thread.
)";

static constexpr absl::Duration kLoggerConnectionTimeout = absl::Seconds(1);

static constexpr absl::Duration kConnectToUpstreamHealthServiceTimeout =
    absl::Minutes(3);

absl::StatusOr<HardwareModuleExitCode> ForwardHealthServiceUntilShutdown(
    const HardwareModuleMainConfig& hwm_main_config) {
  if (hwm_main_config.runtime_context == std::nullopt) {
    return absl::InternalError(
        "ForwardHealthServiceUntilShutdown requires a HardwareModuleMainConfig "
        "RuntimeContext, but the RuntimeContext is nullopt");
  }
  // First of all, bring up a HealthForwarder service. We want to start
  // reporting health ASAP, even if we only respond to requests with UNAVAILABLE
  // at first.
  std::unique_ptr<::grpc::Server> server = nullptr;
  HealthForwarder health_forwarder(hwm_main_config.runtime_context->name());
  // TODO: b/388333650 - Remove the ResourceHealth once fully migrated to
  // ServiceState.
  INTR_ASSIGN_OR_RETURN(server,
                        CreateServer(hwm_main_config.runtime_context->port(),
                                     {&health_forwarder}));

  // Get a channel/stub to the HwmGrpcPortManager service (it runs on the
  // simulation server's pod, so use simulation_server_address +
  // HwmGrpcPortManager::kDefaultGrpcServerPort)
  const auto connect_deadline =
      absl::Now() + kConnectToUpstreamHealthServiceTimeout;
  std::string aggregated_resource_health_address = absl::StrCat(
      hwm_main_config.module_config.simulation_server_address(), ":",
      // cf. intrinsic/simulation/templates/gzserver.yaml
      absl::GetFlag(FLAGS_aggregated_resource_health_port_testonly)
          .value_or(12377));

  INTR_ASSIGN_OR_RETURN(
      auto aggregated_resource_health_channel,
      intrinsic::connect::CreateClientChannel(
          aggregated_resource_health_address, connect_deadline));
  // Pass that stub to the HealthForwarder
  health_forwarder.SetResourceHealthStub(
      intrinsic_proto::simulation::AggregatedResourceHealth::NewStub(
          aggregated_resource_health_channel));

  // Wait for shutdown
  while (IsShutdownRequested() == ShutdownType::kNotRequested) {
    absl::SleepFor(absl::Seconds(1));
  }
  return HardwareModuleExitCode::kNormalShutdown;
}

// Unlike regular hardware modules, a simulated HWM does its work inside of the
// Gazebo server (see
// intrinsic/simulation/gazebo/plugins/hardware_module_launcher.h).
//
// The realtime control service (aka ICON) directly connects to hardware
// interfaces exposed by the Gazebo server, *not* by this binary.
// This is possible because hardware interface communication works via shared
// memory segments that the control service can locate using the hardware
// module's name.
//
// The same is not true for the gRPC services that all hardware modules expose,
// namely ServiceState and ResourceHealth. Clients access those by looking up
// the HWM's name using cluster DNS, and that DNS resolves to this binary.
//
// So this main function serves only one small, but important, purpose:
// To forward ServiceState/ResourceHealth traffic between clients on one side,
// and the actual hardware module code within Gazebo on the other.
absl::StatusOr<HardwareModuleExitCode> ModuleMain(int argc, char** argv) {
  // Handle SIGTERM, sent by Kubernetes to shut down.
  std::signal(SIGTERM, ShutdownSignalHandler);
  // Handle Ctrl+C to shut down.
  std::signal(SIGINT, ShutdownSignalHandler);

  absl::StatusOr<HardwareModuleMainConfig> hwm_main_config = LoadConfig(
      absl::GetFlag(FLAGS_module_config_file),
      absl::GetFlag(FLAGS_runtime_context_file), absl::GetFlag(FLAGS_realtime));

  if (hwm_main_config.ok()) {
    if (absl::Status status = InitDataLogger(hwm_main_config->module_config,
                                             kLoggerConnectionTimeout);
        !status.ok()) {
      LOG(WARNING) << "Failed to connect to the Intrinsic Logger within "
                   << kLoggerConnectionTimeout << ": " << status;
    } else {
      LOG(INFO) << "Connected to the Intrinsic Logger";
    }
  }

  absl::StatusOr<
      absl_nonnull std::unique_ptr<intrinsic::icon::HardwareModuleRuntime>>
      runtime = absl::FailedPreconditionError("Config not OK");
  std::vector<int> cpu_affinity;
  if (!hwm_main_config.ok()) {
    // This does three things:
    // 1. It prepends some context in front of the error status
    // 2. It logs the whole thing when the StatusBuilder converts to a Status
    // 3. It creates a StatusOr of the correct type for the
    //    RunRuntimeWithGrpcServerAndWaitForShutdown() below. We can't quite
    //    inline it into that call, or make this variable just a Status, because
    //    the function takes an *lvalue reference* to a StatusOr.
    absl::StatusOr<
        absl_nonnull std::unique_ptr<intrinsic::icon::HardwareModuleRuntime>>
        wrapped_config_status =
            intrinsic::StatusBuilder(hwm_main_config.status())
                .SetPrepend()
                .LogError()
            << "Failed to load hardware module configuration: ";
    auto exit_code_promise =
        std::make_shared<SharedPromiseWrapper<HardwareModuleExitCode>>();
    // This function spins up some gRPC servers to serve the error from loading
    // the configuration, so that users can get a hint on how to fix it.
    INTR_ASSIGN_OR_RETURN(
        std::optional<HardwareModuleExitCode> exit_code,
        RunRuntimeWithGrpcServerAndWaitForShutdown(
            hwm_main_config, exit_code_promise,
            /*runtime=*/
            wrapped_config_status, absl::GetFlag(FLAGS_grpc_server_port),
            cpu_affinity));
    return exit_code.value_or(HardwareModuleExitCode::kNormalShutdown);
  }

  // Skip setting up actual hardware interfaces, and just wait for someone
  // to tell us to shut down.
  LOG(INFO) << "GazeboHwmStub '" << hwm_main_config->module_config.name()
            << "' entering ServiceState/ResourceHealth forwarding mode.";
  return ForwardHealthServiceUntilShutdown(*hwm_main_config);
}

}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  InitIntrinsic(intrinsic::icon::kUsageString, argc, argv);
  constexpr int prefault_memory = 256 * 1024;
  QCHECK_OK((intrinsic::LockMemory<prefault_memory, prefault_memory>()));
  absl::StatusOr<intrinsic::icon::HardwareModuleExitCode> exit_code =
      (intrinsic::icon::ModuleMain(argc, argv));
  if (!exit_code.ok()) {
    LOG(ERROR) << "PUBLIC: Hardware module main failed: " << exit_code.status();
    return 1;
  }
  LOG(INFO) << "PUBLIC: Hardware module shutdown complete with code "
            << static_cast<int>(exit_code.value());
  return static_cast<int>(exit_code.value());
}
