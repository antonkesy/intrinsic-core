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

#ifndef INTRINSIC_ICON_SERVER_CREATE_ICON_MAIN_LOOP_H_
#define INTRINSIC_ICON_SERVER_CREATE_ICON_MAIN_LOOP_H_

#include <memory>
#include <string>
#include <string_view>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/geometry/storage/geometry_library.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/icon/control/services/service_collection.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/server/config/realtime_control_config.pb.h"
#include "intrinsic/icon/server/config/services_config.pb.h"
#include "intrinsic/icon/server/main_loop.h"
#include "intrinsic/icon/server/runtime_options.h"
#include "intrinsic/icon/utils/duration.h"

namespace intrinsic::icon {

// Creates a MainLoop instance from a proto config.
absl::StatusOr<std::unique_ptr<MainLoop>> CreateIconMainLoopFromProto(
    const intrinsic_proto::icon::IconMainConfig& config,
    const ServerRuntimeOptions& options,
    absl::string_view shared_memory_namespace,
    bool use_runtime_asset_fallback = false);

// Internal helper for CreateIconMainLoopFromProto.
// Creates a ServiceCollection with all Services from the config.
absl::StatusOr<ServiceCollection> CreateServiceCollectionFromProto(
    const intrinsic_proto::icon::ServicesConfig& config,
    const Service::InitParameters& init_parameters,
    const intrinsic_proto::assets::v1::ResolvedDependency& intrinsic_runtime,
    bool use_runtime_asset_fallback = false);

// This attempts to connect the real-time control layer with each hardware
// module, making multiple attempts in case of a
// failed connection until 'module_wait_timeout' is reached.
// `hardware_module_name_to_config` is a map of hardware module names to their
// configuration.
absl::Status ConnectToHardwareModules(
    const absl::flat_hash_map<std::string,
                              intrinsic_proto::icon::HardwareConfig>&
        hardware_module_name_to_config,
    std::unique_ptr<HardwareModuleManager>& hardware_module_manager,
    std::string_view shared_memory_namespace,
    std::string_view hardware_module_that_drives_clock,
    absl::Duration module_wait_timeout,
    intrinsic::Duration expected_control_period = intrinsic::ZeroDuration());

absl::StatusOr<intrinsic_proto::icon::IconMainConfig>
ApplyDeactivatedHardwareConfiguration(
    const intrinsic_proto::icon::IconMainConfig& config);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_CREATE_ICON_MAIN_LOOP_H_
