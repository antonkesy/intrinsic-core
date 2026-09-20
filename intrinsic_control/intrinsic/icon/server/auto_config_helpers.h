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

#ifndef INTRINSIC_ICON_SERVER_AUTO_CONFIG_HELPERS_H_
#define INTRINSIC_ICON_SERVER_AUTO_CONFIG_HELPERS_H_

#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/resources/client/resource_registry_client.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/proto/object_world_service.pb.h"

namespace intrinsic::icon {

// Generates a suggested `IconMainConfig` based on the given
// `modules_and_interfaces` and an `object_world_client`.
//
// The `modules_and_interfaces` is a map of module name to a map of interface
// name to interface type. The `object_world_client` is used to query the
// `World` for objects and frames.
absl::StatusOr<intrinsic_proto::icon::IconMainConfig> AutoGenerateIconConfig(
    const absl::flat_hash_map<std::string,
                              absl::flat_hash_map<std::string, std::string>>&
        modules_and_interfaces,
    const absl::flat_hash_map<std::string, std::string>&
        module_name_to_resource_name,
    const resources::ResourceRegistryClient& resource_registry_client,
    world::ObjectWorldClient& object_world_client);

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_SERVER_AUTO_CONFIG_HELPERS_H_
