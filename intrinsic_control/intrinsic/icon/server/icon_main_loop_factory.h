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

#ifndef INTRINSIC_ICON_SERVER_ICON_MAIN_LOOP_FACTORY_H_
#define INTRINSIC_ICON_SERVER_ICON_MAIN_LOOP_FACTORY_H_
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/assets/proto/status_spec.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/icon/proto/v1/jogging_service.grpc.pb.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/server/grpc_envelope.h"
#include "intrinsic/icon/server/icon_api_service.h"
#include "intrinsic/icon/server/main_loop.h"
#include "intrinsic/icon/server/runtime_options.h"
#include "intrinsic/resources/proto/resource_registry.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic::icon {

struct MainLoopConfiguration {
  absl::StatusOr<intrinsic_proto::icon::IconMainConfig> config;
  intrinsic::icon::ServerRuntimeOptions server_runtime_options;
  int32_t icon_server_port = 0;
  std::string shared_memory_namespace = "";
  std::string resource_registry_address = "";
  bool use_runtime_asset_fallback = false;
};

class IconMainLoopImpl final : public IconImplInterface {
 public:
  IconMainLoopImpl(const intrinsic::icon::MainLoopConfiguration& configuration,
                   intrinsic::Thread pause_tracing_thread,
                   std::unique_ptr<MainLoop> main_loop)
      : configuration_(configuration),
        pause_tracing_thread_(std::move(pause_tracing_thread)),
        main_loop_(std::move(main_loop)) {}

  ~IconMainLoopImpl() override;

  absl::StatusOr<icon::IconApiService* absl_nonnull> IconService()
      ABSL_ATTRIBUTE_LIFETIME_BOUND override;
  absl::StatusOr<intrinsic_proto::gpio::v1::GPIOService::Service* absl_nonnull>
  GpioService() ABSL_ATTRIBUTE_LIFETIME_BOUND override;

  absl::StatusOr<
      intrinsic_proto::icon::v1::JoggingService::Service* absl_nonnull>
  JoggingService() ABSL_ATTRIBUTE_LIFETIME_BOUND override;

 private:
  intrinsic::icon::MainLoopConfiguration configuration_;
  // This thread is for startup realtime tracing (with the `start_tracing_for`
  // CLI flag). It sleeps for a bit, and then disables realtime tracing.
  intrinsic::Thread pause_tracing_thread_;
  std::unique_ptr<MainLoop> main_loop_;
};

// Factory function to create an `IconMainLoopImpl`.
absl::StatusOr<std::unique_ptr<IconMainLoopImpl>> IconMainLoopImplFactory(
    absl::StatusOr<intrinsic::icon::MainLoopConfiguration> configuration);

// Performs additional shutdown tasks after the ICON instance is
// destroyed.
// Calls the hardware module restart API if any hardware modules are
// in a init/fatal faulted state. `hardware_module_names` is a list of hardware
// module names to check. `shared_memory_namespace` is the shared memory
// namespace to use for the hardware module restart API.
absl::Status IconMainLoopShutdownForRestart(
    absl::Span<const std::string> hardware_module_names,
    absl::string_view shared_memory_namespace);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_ICON_MAIN_LOOP_FACTORY_H_
