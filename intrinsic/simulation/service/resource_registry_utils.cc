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

#include "intrinsic/simulation/service/resource_registry_utils.h"

#include <memory>
#include <string_view>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/resources/client/resource_registry_client.h"
#include "intrinsic/resources/proto/resource_registry.pb.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace simulation {

absl::StatusOr<std::vector<ResourceConnectionInfo>> GetResourcesFromRegistry(
    std::string_view resource_registry_address,
    absl::Duration grpc_connect_timeout, std::string_view capability_name) {
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<intrinsic::resources::ResourceRegistryClientInterface>
          client,
      resources::CreateResourceRegistryClient(
          resource_registry_address, absl::Seconds(60), grpc_connect_timeout));
  return GetResourcesFromRegistry(client.get(), capability_name);
}

absl::StatusOr<std::vector<ResourceConnectionInfo>> GetResourcesFromRegistry(
    intrinsic::resources::ResourceRegistryClientInterface* absl_nonnull client,
    std::string_view capability_name) {
  intrinsic_proto::resources::ListResourceInstanceRequest::StrictFilter filter;
  filter.add_capability_names(capability_name);
  INTR_ASSIGN_OR_RETURN(auto resources, client->ListResources(filter));
  if (resources.empty()) {
    return NotFoundErrorBuilder() << "No resources with capability ["
                                  << capability_name << "] installed.";
  }

  std::vector<ResourceConnectionInfo> result;
  for (const auto& instance : resources) {
    const auto& handle = instance.resource_handle();
    if (handle.has_connection_info() && handle.connection_info().has_grpc()) {
      result.push_back(ResourceConnectionInfo{
          .name = instance.name(),
          .connection_params = ConnectionParams{
              .address = handle.connection_info().grpc().address(),
              .instance_name =
                  handle.connection_info().grpc().server_instance(),
              .header = handle.connection_info().grpc().header(),
          }});
    }
  }

  if (result.empty()) {
    return NotFoundErrorBuilder()
           << "Found " << resources.size() << " resources with capability ["
           << capability_name << "] but none had grpc connection info.";
  }

  return result;
}

}  // namespace simulation
}  // namespace intrinsic
