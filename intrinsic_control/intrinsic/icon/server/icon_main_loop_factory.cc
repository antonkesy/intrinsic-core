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

#include "intrinsic/icon/server/icon_main_loop_factory.h"

#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/assets/proto/status_spec.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/icon/hal/hardware_module_proxy.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/interprocess/remote_trigger/remote_trigger_client.h"
#include "intrinsic/icon/proto/v1/jogging_service.grpc.pb.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/server/create_icon_main_loop.h"
#include "intrinsic/icon/server/icon_api_service.h"
#include "intrinsic/icon/server/main_loop.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/shutdown_signals.h"
#include "intrinsic/resources/proto/resource_registry.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/rt_trace_internal.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::icon {

namespace {
constexpr absl::Duration kPollShutdownSignalEvery = absl::Milliseconds(250);

// Returns once `pause_duration` is over, or a shutdown was requested.
void PauseFor(const absl::Duration pause_duration) {
  for (absl::Duration time_passed;
       time_passed < pause_duration &&
       IsShutdownRequested() == ShutdownType::kNotRequested;
       time_passed += kPollShutdownSignalEvery) {
    absl::SleepFor(kPollShutdownSignalEvery);
  }
  LOG(INFO) << "Finished waiting for: " << pause_duration
            << " or received a shutdown request.";
}
}  // namespace

IconMainLoopImpl::~IconMainLoopImpl() {
  if (main_loop_ != nullptr) {
    main_loop_->RequestShutdown();
    main_loop_->Join();
  }
}

absl::StatusOr<icon::IconApiService* absl_nonnull>
IconMainLoopImpl::IconService() {
  if (main_loop_ == nullptr) {
    return absl::InternalError(
        "MainLoop is nullptr. Please report this as a bug.");
  }
  // GetExitStatus returns nullopt if the MainLoop is still running happily.
  std::optional<icon::RealtimeStatus> exit_status = main_loop_->GetExitStatus();
  if (exit_status.has_value()) {
    return exit_status.value();
  }
  return main_loop_->IconService();
}

absl::StatusOr<intrinsic_proto::gpio::v1::GPIOService::Service* absl_nonnull>
IconMainLoopImpl::GpioService() {
  if (main_loop_ == nullptr) {
    return absl::InternalError(
        "MainLoop is nullptr. Please report this as a bug.");
  }
  // GetExitStatus returns nullopt if the MainLoop is still running happily.
  std::optional<icon::RealtimeStatus> exit_status = main_loop_->GetExitStatus();
  if (exit_status.has_value()) {
    return exit_status.value();
  }
  return main_loop_->GpioService();
}

absl::StatusOr<intrinsic_proto::icon::v1::JoggingService::Service* absl_nonnull>
IconMainLoopImpl::JoggingService() {
  if (main_loop_ == nullptr) {
    return absl::InternalError(
        "MainLoop is nullptr. Please report this as a bug.");
  }
  // GetExitStatus returns nullopt if the MainLoop is still running happily.
  std::optional<icon::RealtimeStatus> exit_status = main_loop_->GetExitStatus();
  if (exit_status.has_value()) {
    return exit_status.value();
  }
  return main_loop_->JoggingService();
}

absl::StatusOr<std::unique_ptr<IconMainLoopImpl>> IconMainLoopImplFactory(
    absl::StatusOr<intrinsic::icon::MainLoopConfiguration> configuration) {
  INTR_RETURN_IF_ERROR(configuration.status());
  INTR_ASSIGN_OR_RETURN(intrinsic_proto::icon::IconMainConfig main_config,
                        configuration->config);
  intrinsic::Thread pause_tracing_thread;
  if (main_config.has_realtime_tracing_config()) {
    intrinsic::tracing::Tracer::EnableTracing();

    if (const auto status =
            intrinsic::tracing::Tracer::InitializeThreadLocalTracer(
                "timeslicer_main");
        !status.ok()) {
      LOG(WARNING) << "Failed to InitializeThreadLocalTracer: " << status;
    }
    // Optionally start tracing immediately if tracing is enabled.
    absl::Duration start_tracing_for = absl::ZeroDuration();
    if (absl::StatusOr<absl::Duration> duration = ToAbslDuration(
            main_config.realtime_tracing_config().start_tracing_for());
        !duration.ok()) {
      LOG(WARNING) << "Not starting tracing, because decoding of "
                      "start_tracing_for failed: "
                   << duration.status();
    } else {
      start_tracing_for = *duration;
    }

    if (start_tracing_for > absl::ZeroDuration()) {
      // Automatically pause tracing after `duration`.
      if (auto thread = CreateRealtimeCapableThread(
              intrinsic::ThreadOptions()
                  .SetSkipInitializingThreadLocalTracer()
                  .SetName("Pause Tracing"),
              [start_tracing_for]() {
                INTRINSIC_ASSERT_NON_REALTIME();
                LOG(INFO) << "Starting tracing for: " << start_tracing_for;
                PauseFor(start_tracing_for);
                LOG(INFO) << "Pausing initial tracing after it was active for: "
                          << start_tracing_for;
                intrinsic::tracing::Tracer::PauseTracing();
                return;
              });
          thread.ok()) {
        pause_tracing_thread = *std::move(thread);
      } else {
        LOG(ERROR) << "Failed to setup the automatic pause for tracing."
                   << thread.status();
      }
      intrinsic::tracing::Tracer::StartTracing();
    }
  }

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<MainLoop> main_loop,
                        CreateIconMainLoopFromProto(
                            main_config, configuration->server_runtime_options,
                            configuration->shared_memory_namespace,
                            configuration->use_runtime_asset_fallback));

  INTR_RETURN_IF_ERROR(main_loop->Start());
  LOG(INFO) << "ICON main loop started.";
  return std::make_unique<IconMainLoopImpl>(
      *configuration, std::move(pause_tracing_thread), std::move(main_loop));
}

absl::Status IconMainLoopShutdownForRestart(
    absl::Span<const std::string> hardware_module_names,
    absl::string_view shared_memory_namespace) {
  // Check if any hardware modules are in a init/fatal faulted state. If so,
  // request a restart.
  std::vector<std::pair<HardwareModuleProxy, RemoteTriggerClient::AsyncRequest>>
      async_requests;
  for (auto& hw_module_name : hardware_module_names) {
    // Skips the control_period check, so that we can restart a misconfigured
    // hwm.
    auto hw_module = WaitForHardwareModule(
        shared_memory_namespace, hw_module_name, absl::Seconds(1),
        /*expected_control_period=*/std::nullopt);
    if (hw_module.ok()) {
      if (hw_module->GetHardwareModuleState()->code() ==
              intrinsic_fbs::StateCode::kInitFailed ||
          hw_module->GetHardwareModuleState()->code() ==
              intrinsic_fbs::StateCode::kFatallyFaulted) {
        LOG(INFO) << "Requesting restart of hardware module " << hw_module_name
                  << ".";
        auto ret = hw_module->RestartAsync();
        if (ret.ok()) {
          // We need to store the HWM proxy as well since the async request is
          // invalid once the HWM proxy is destroyed.
          async_requests.push_back(
              std::make_pair(std::move(*hw_module), std::move(ret.value())));
        } else {
          LOG(WARNING) << "Failed to request restart of hardware module "
                       << hw_module_name << ": " << ret.status();
        }
      }
    } else {
      LOG(WARNING) << "Failed to connect to hardware module " << hw_module_name
                   << ": " << hw_module.status();
    }
  }
  const size_t num_async_requests = async_requests.size();
  auto deadline = absl::Now() + absl::Seconds(5);
  while (absl::Now() < deadline) {
    auto next_deadline = absl::Now() + absl::Milliseconds(100);
    for (auto it = async_requests.begin(); it != async_requests.end();) {
      if (auto status = it->second.WaitUntil(next_deadline);
          status.ok() || !absl::IsDeadlineExceeded(status)) {
        LOG(INFO) << "Hardware module '" << it->first.Name()
                  << "' restart request completed: " << status;
        it = async_requests.erase(it);
      } else {
        it++;
      }
    }
    if (async_requests.empty()) {
      break;
    }
  }
  LOG(INFO) << "Restart requested on " << num_async_requests
            << " hardware modules.";
  return absl::OkStatus();
}

}  // namespace intrinsic::icon
