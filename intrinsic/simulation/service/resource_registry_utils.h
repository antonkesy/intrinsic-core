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

#ifndef INTRINSIC_SIMULATION_SERVICE_RESOURCE_REGISTRY_UTILS_H_
#define INTRINSIC_SIMULATION_SERVICE_RESOURCE_REGISTRY_UTILS_H_

#include <string>
#include <string_view>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/resources/client/resource_registry_client_interface.h"
#include "intrinsic/util/grpc/connection_params.h"

namespace intrinsic {
namespace simulation {

struct ResourceConnectionInfo {
  std::string name;
  ConnectionParams connection_params;
};

// Queries the resource registry for all installed assets matching the given
// capability name.
// Returns a `NotFoundError` if no matching assets are found in the resource
// registry.
absl::StatusOr<std::vector<ResourceConnectionInfo>> GetResourcesFromRegistry(
    std::string_view resource_registry_address,
    absl::Duration grpc_connect_timeout, std::string_view capability_name);

// Variant that takes a connected client to the resource registry service.
absl::StatusOr<std::vector<ResourceConnectionInfo>> GetResourcesFromRegistry(
    intrinsic::resources::ResourceRegistryClientInterface* absl_nonnull client,
    std::string_view capability_name);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_SERVICE_RESOURCE_REGISTRY_UTILS_H_
