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

#include "intrinsic/simulation/gazebo/asset_instances_client.h"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/client_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::simulation {

absl::StatusOr<std::unique_ptr<AssetInstancesClient>>
AssetInstancesClient::Create(std::string_view grpc_address,
                             absl::Duration connection_timeout,
                             absl::Duration rpc_timeout) {
  INTR_ASSIGN_OR_RETURN(auto channel,
                        connect::CreateClientChannel(
                            grpc_address,
                            /*deadline=*/absl::Now() + connection_timeout));
  auto stub = intrinsic_proto::assets::v1::AssetInstances::NewStub(channel);
  return absl::WrapUnique(new AssetInstancesClient(
      std::move(stub), connection_timeout, rpc_timeout));
}

AssetInstancesClient::AssetInstancesClient(
    std::unique_ptr<intrinsic_proto::assets::v1::AssetInstances::Stub> stub,
    absl::Duration connection_timeout, absl::Duration rpc_timeout)
    : stub_(std::move(stub)),
      connection_timeout_(connection_timeout),
      rpc_timeout_(rpc_timeout) {}

absl::StatusOr<std::vector<intrinsic_proto::assets::v1::AssetInstance>>
AssetInstancesClient::ListHardwareDevicesAndServices(
    intrinsic_proto::assets::v1::AssetInstanceView view) const {
  return ListAssetInstancesWithFilters(
      {
          intrinsic_proto::assets::AssetType::ASSET_TYPE_HARDWARE_DEVICE,
          intrinsic_proto::assets::AssetType::ASSET_TYPE_SERVICE,
      },
      view);
}

absl::StatusOr<std::vector<intrinsic_proto::assets::v1::AssetInstance>>
AssetInstancesClient::ListHardwareDevices(
    intrinsic_proto::assets::v1::AssetInstanceView view) const {
  return ListAssetInstancesWithFilters(
      {
          intrinsic_proto::assets::AssetType::ASSET_TYPE_HARDWARE_DEVICE,
      },
      view);
}

absl::StatusOr<std::vector<intrinsic_proto::assets::v1::AssetInstance>>
AssetInstancesClient::ListServiceAssets(
    intrinsic_proto::assets::v1::AssetInstanceView view) const {
  return ListAssetInstancesWithFilters(
      {
          intrinsic_proto::assets::AssetType::ASSET_TYPE_SERVICE,
      },
      view);
}

absl::StatusOr<std::vector<intrinsic_proto::assets::v1::AssetInstance>>
AssetInstancesClient::ListAssetInstancesWithFilters(
    absl::Span<const intrinsic_proto::assets::AssetType> asset_types,
    intrinsic_proto::assets::v1::AssetInstanceView view) const {
  std::vector<intrinsic_proto::assets::v1::AssetInstance> instances;
  std::string page_token;
  auto deadline = absl::ToChronoTime(absl::Now() + rpc_timeout_);

  do {
    ::grpc::ClientContext context;
    context.set_deadline(deadline);
    intrinsic::ConfigureClientContext(&context);

    intrinsic_proto::assets::v1::ListAssetInstancesRequest req;
    req.set_view(view);
    req.set_page_token(page_token);

    for (const auto& asset_type : asset_types) {
      auto* filter = req.add_strict_filters();
      filter->set_asset_type(asset_type);
    }

    intrinsic_proto::assets::v1::ListAssetInstancesResponse resp;
    INTR_RETURN_IF_ERROR(
        ToAbslStatus(stub_->ListAssetInstances(&context, req, &resp)));

    instances.insert(instances.end(),
                     std::make_move_iterator(resp.asset_instances().begin()),
                     std::make_move_iterator(resp.asset_instances().end()));
    page_token = resp.next_page_token();
  } while (!page_token.empty());

  return instances;
}

absl::StatusOr<intrinsic_proto::assets::v1::AssetInstance>
AssetInstancesClient::GetAssetInstance(
    std::string_view name,
    intrinsic_proto::assets::v1::AssetInstanceView view) const {
  ::grpc::ClientContext context;
  context.set_deadline(absl::ToChronoTime(absl::Now() + rpc_timeout_));
  intrinsic::ConfigureClientContext(&context);

  intrinsic_proto::assets::v1::GetAssetInstanceRequest req;
  req.set_name(std::string(name));
  req.set_view(view);

  intrinsic_proto::assets::v1::AssetInstance resp;
  INTR_RETURN_IF_ERROR(
      ToAbslStatus(stub_->GetAssetInstance(&context, req, &resp)));
  return resp;
}

absl::StatusOr<google::protobuf::Any>
AssetInstancesClient::ExtractServiceConfig(
    const intrinsic_proto::assets::v1::AssetInstance& asset_instance) {
  switch (asset_instance.config().variant_case()) {
    case intrinsic_proto::assets::v1::InstanceConfig::kHardwareDevice:
      return asset_instance.config()
          .hardware_device()
          .service()
          .service_config();
    case intrinsic_proto::assets::v1::InstanceConfig::kService:
      return asset_instance.config().service().service_config();
    case intrinsic_proto::assets::v1::InstanceConfig::kSceneObject:
    case intrinsic_proto::assets::v1::InstanceConfig::VARIANT_NOT_SET:
      break;
  }
  return absl::InvalidArgumentError(
      absl::StrCat("Asset instance '", asset_instance.name(),
                   "' is neither a hardware_device nor a service."));
}

}  // namespace intrinsic::simulation
