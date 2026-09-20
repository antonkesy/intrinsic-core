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

#ifndef INTRINSIC_SIMULATION_GAZEBO_ASSET_INSTANCES_CLIENT_H_
#define INTRINSIC_SIMULATION_GAZEBO_ASSET_INSTANCES_CLIENT_H_

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/assets/proto/v1/asset_instances.grpc.pb.h"
#include "intrinsic/assets/proto/v1/asset_instances.pb.h"
#include "intrinsic/connect/cc/grpc/channel.h"

namespace intrinsic::simulation {

// Encapsulates a gRPC client connection and requests to the
// AssetInstancesService.
class AssetInstancesClient {
 public:
  // Creates a client to connection.
  // `grpc_address` is the address of the remote gRPC AssetInstancesService.
  // `connection_timeout` is the timeout for initial connection.
  // `rpc_timeout` is the timeout for subsequent RPC requests made by this
  // client.
  static absl::StatusOr<std::unique_ptr<AssetInstancesClient>> Create(
      std::string_view grpc_address,
      absl::Duration connection_timeout =
          ::intrinsic::connect::kGrpcClientConnectDefaultTimeout,
      absl::Duration rpc_timeout = absl::Seconds(10));

  // Retrieves all Asset instances matching ASSET_TYPE_HARDWARE_DEVICE or
  // ASSET_TYPE_SERVICE using the specified view.
  absl::StatusOr<std::vector<intrinsic_proto::assets::v1::AssetInstance>>
  ListHardwareDevicesAndServices(
      intrinsic_proto::assets::v1::AssetInstanceView view = intrinsic_proto::
          assets::v1::AssetInstanceView::ASSET_INSTANCE_VIEW_DETAIL) const;

  // Retrieves all Asset instances matching ASSET_TYPE_HARDWARE_DEVICE using the
  // specified view.
  absl::StatusOr<std::vector<intrinsic_proto::assets::v1::AssetInstance>>
  ListHardwareDevices(intrinsic_proto::assets::v1::AssetInstanceView view =
                          intrinsic_proto::assets::v1::AssetInstanceView::
                              ASSET_INSTANCE_VIEW_DETAIL) const;

  // Retrieves all Asset instances matching ASSET_TYPE_SERVICE using the
  // specified view.
  absl::StatusOr<std::vector<intrinsic_proto::assets::v1::AssetInstance>>
  ListServiceAssets(intrinsic_proto::assets::v1::AssetInstanceView view =
                        intrinsic_proto::assets::v1::AssetInstanceView::
                            ASSET_INSTANCE_VIEW_DETAIL) const;

  // Resolves the configuration details for a single Asset instance by name
  // using the specified view.
  absl::StatusOr<intrinsic_proto::assets::v1::AssetInstance> GetAssetInstance(
      std::string_view name,
      intrinsic_proto::assets::v1::AssetInstanceView view = intrinsic_proto::
          assets::v1::AssetInstanceView::ASSET_INSTANCE_VIEW_DETAIL) const;

  // Extracts the Any proto configuration from the Asset instance's hardware
  // device or service component. Returns InvalidArgumentError if the Asset
  // instance has neither.
  static absl::StatusOr<google::protobuf::Any> ExtractServiceConfig(
      const intrinsic_proto::assets::v1::AssetInstance& asset_instance);

  // Templated helper that extracts and unpacks the configuration from the
  // Asset instance to the expected configuration proto type.
  template <typename ConfigType>
  static absl::StatusOr<ConfigType> ExtractServiceConfig(
      const intrinsic_proto::assets::v1::AssetInstance& asset_instance) {
    auto any = ExtractServiceConfig(asset_instance);
    if (!any.ok()) return any.status();
    ConfigType config;
    if (!any->UnpackTo(&config)) {
      return absl::InvalidArgumentError(
          absl::StrCat("Failed to unpack configuration for Asset instance '",
                       asset_instance.name(), "' to expected type '",
                       ConfigType::descriptor()->full_name(), "'"));
    }
    return config;
  }

 private:
  AssetInstancesClient(
      std::unique_ptr<intrinsic_proto::assets::v1::AssetInstances::Stub> stub,
      absl::Duration connection_timeout, absl::Duration rpc_timeout);

  absl::StatusOr<std::vector<intrinsic_proto::assets::v1::AssetInstance>>
  ListAssetInstancesWithFilters(
      absl::Span<const intrinsic_proto::assets::AssetType> asset_types,
      intrinsic_proto::assets::v1::AssetInstanceView view) const;

  std::unique_ptr<intrinsic_proto::assets::v1::AssetInstances::Stub> stub_;
  absl::Duration connection_timeout_;
  absl::Duration rpc_timeout_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_ASSET_INSTANCES_CLIENT_H_
