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

#include "intrinsic/perception/cameras/ros_image_source_lister.h"

#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/assets/proto/installed_assets.pb.h"
#include "intrinsic/assets/proto/view.pb.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/cameras/image_source_lister.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/pubsub_ros.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rmw/serialized_message.h"
#include "snapshot_interfaces/msg/detail/discovered_camera__struct.hpp"
#include "snapshot_interfaces/srv/detail/discover__struct.hpp"
#include "snapshot_interfaces/srv/detail/legacy_discover__struct.hpp"

namespace intrinsic::perception {

using ::snapshot_interfaces::srv::Discover;
using ::snapshot_interfaces::srv::LegacyDiscover;

RosImageSourceLister::RosImageSourceLister()
    : pubsub_(std::make_unique<PubSub>()) {}

absl::StatusOr<std::vector<CameraIdentifier>>
RosImageSourceLister::ListAvailableCameras() {
  INTR_ASSIGN_OR_RETURN(
      const bool use_legacy_discovery_service,
      RosImageSourceLister::ShouldUseLegacyDiscoveryService());
  LOG(INFO) << "RosImageSourceLister::ListAvailableCameras(): "
               "use_legacy_discovery_service: "
            << use_legacy_discovery_service;

  std::vector<CameraIdentifier> camera_identifiers;
  if (use_legacy_discovery_service) {
    absl::StatusOr<LegacyDiscover::Response> response =
        CallDiscoverService<LegacyDiscover::Response, LegacyDiscover::Request>(
            "0/ips2_discovery/discover/**", LegacyDiscover::Request());
    if (!response.ok()) {
      if (response.status().code() == absl::StatusCode::kUnavailable) {
        return camera_identifiers;
      }
      return response.status();
    }
    camera_identifiers.reserve(response->cameras.size());
    for (const std::string& camera : response->cameras) {
      camera_identifiers.push_back(
          {.driver = CameraIdentifier::Ros{.driver_type = "ips2",
                                           .device_id = camera}});
    }
  } else {
    absl::StatusOr<Discover::Response> response =
        CallDiscoverService<Discover::Response, Discover::Request>(
            "0/cameras/discover/**", Discover::Request());
    if (!response.ok()) {
      if (response.status().code() == absl::StatusCode::kUnavailable) {
        return camera_identifiers;
      }
      return response.status();
    }
    camera_identifiers.reserve(response->cameras.size());
    for (const snapshot_interfaces::msg::DiscoveredCamera& camera :
         response->cameras) {
      camera_identifiers.push_back(
          {.driver = CameraIdentifier::Ros{.driver_type = camera.driver_type,
                                           .device_id = camera.camera_id}});
    }
  }

  return camera_identifiers;
}

absl::StatusOr<bool> RosImageSourceLister::ShouldUseLegacyDiscoveryService() {
  // Query the assets service for the installed version, if any, of ips2_driver
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> assets_service_channel,
      connect::CreateClientChannel(
          "istio-ingressgateway.app-ingress.svc.cluster.local:80",
          absl::Now() + absl::Seconds(10)));
  std::unique_ptr<
      intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface>
      installed_assets_stub =
          intrinsic_proto::assets::v1::InstalledAssetsReader::NewStub(
              assets_service_channel);

  intrinsic_proto::assets::v1::ListInstalledAssetsRequest request;
  request.set_page_size(200);
  request.mutable_strict_filter()->add_asset_types(
      intrinsic_proto::assets::AssetType::ASSET_TYPE_SERVICE);
  request.set_view(
      intrinsic_proto::catalog::AssetViewType::ASSET_VIEW_TYPE_BASIC);

  do {
    // ClientContext instances may not be reused across rpcs!
    grpc::ClientContext context;
    ConfigureClientContext(&context);

    LOG(INFO) << "Requesting page of installed assets";
    intrinsic_proto::assets::v1::ListInstalledAssetsResponse response;

    INTR_RETURN_IF_ERROR(
        ToAbslStatus(installed_assets_stub->ListInstalledAssets(
            &context, request, &response)));
    for (const intrinsic_proto::assets::v1::InstalledAsset& asset :
         response.installed_assets()) {
      if (asset.metadata().id_version().id().package() == "ai.intrinsic" &&
          asset.metadata().id_version().id().name() == "ips2_driver") {
        const std::string& version = asset.metadata().id_version().version();
        LOG(INFO) << "Found ips2_driver with version " << version;
        // If the asset is sideloaded, it will have a plus ('+') in it, so we
        // can just immediately return false, since sideloaded versions will
        // not require legacy discovery.
        if (absl::StrContains(version, "+")) {
          return false;
        }
        // Otherwise, it should be a SemVer scheme that we need to split
        std::vector<std::string> version_parts = absl::StrSplit(version, '.');
        if (version_parts.size() != 3) {
          return absl::InternalError(
              absl::StrCat("Invalid ips2_driver version format: ", version));
        }
        int major, minor, patch;
        if (!absl::SimpleAtoi(version_parts[0], &major)) {
          return absl::InternalError(absl::StrCat(
              "Invalid ips2_driver version major part: ", version));
        }
        if (!absl::SimpleAtoi(version_parts[1], &minor)) {
          return absl::InternalError(absl::StrCat(
              "Invalid ips2_driver version minor part: ", version));
        }
        if (!absl::SimpleAtoi(version_parts[2], &patch)) {
          return absl::InternalError(absl::StrCat(
              "Invalid ips2_driver version patch part: ", version));
        }
        // all versions <= 0.9.7 require legacy discovery and rmw_zenoh comms
        if ((major == 0 && minor < 9) ||
            (major == 0 && minor == 9 && patch <= 7)) {
          return true;
        }
      }
    }
    request.set_page_token(response.next_page_token());
  } while (!request.page_token().empty());

  // If we get here, ips2_driver is not installed or is newer than 0.9.7, so
  // we don't need to use the legacy discovery service and rmw_zenoh comms.
  return false;
}

template <typename ResponseT, typename RequestT>
absl::StatusOr<ResponseT> RosImageSourceLister::CallDiscoverService(
    std::string_view service_name, const RequestT& request) {
  absl::MutexLock lock(mutex_);
  const PubSub::QueryOptions query_options = {};

  // Serialize the request.
  rclcpp::Serialization<RequestT> request_serialization;
  rclcpp::SerializedMessage serialized_request;
  try {
    request_serialization.serialize_message(static_cast<const void*>(&request),
                                            &serialized_request);
  } catch (std::exception& ex) {
    const std::string error_message =
        absl::StrCat("Failed to serialize request: ", ex.what(),
                     " for service ", service_name);
    return absl::InternalError(error_message);
  }

  // Call the ros service.
  INTR_ASSIGN_OR_RETURN(rclcpp::SerializedMessage serialized_response,
                        pubsub_->CallOne<rclcpp::SerializedMessage>(
                            service_name, serialized_request, query_options),
                        _.LogError()
                            << "Failed to call ros service " << service_name);
  if (serialized_response.size() == 0) {
    return absl::UnavailableError(
        absl::StrCat("Failed to call ros service ", service_name));
  }

  // Deserialize the response.
  rclcpp::Serialization<ResponseT> response_serialization;
  ResponseT response;
  try {
    response_serialization.deserialize_message(&serialized_response,
                                               static_cast<void*>(&response));
  } catch (std::exception& ex) {
    const std::string error_message =
        absl::StrCat("Failed to deserialize response: ", ex.what(),
                     " for service ", service_name);
    return absl::InternalError(error_message);
  }

  if (!response.success) {
    return absl::InternalError(
        absl::StrCat("Discover service ", service_name,
                     " returned with failure: ", response.error_message));
  }

  return std::move(response);
}

REGISTER_IMAGE_SOURCE_LISTER(RosImageSourceLister, "ros");

}  // namespace intrinsic::perception
