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

#include "intrinsic/motion_planning/service/motion_planner_service_asset_utils.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "grpcpp/channel.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/dependencies/utils.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/motion_planning/motion_planner_client.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.grpc.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.pb.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

absl::StatusOr<std::unique_ptr<motion_planning::MotionPlannerClient>>
GetMotionPlannerServiceAssetClient(
    std::string_view world_id,
    const intrinsic_proto::assets::v1::ResolvedDependency& mps_dependency) {
  if (mps_dependency.interfaces().empty()) {
    // TODO(b/524634328): Remove the fallback logic from the Skills.
    return nullptr;
  }
  // Try to connect to the `MotionPlannerService` (MPS) Asset.
  INTR_ASSIGN_OR_RETURN(std::shared_ptr<grpc::Channel> channel,
                        assets::dependencies::Connect(
                            mps_dependency, kMotionPlannerServiceInterface,
                            connect::UnlimitedMessageSizeGrpcChannelArgs()));
  INTR_RETURN_IF_ERROR(intrinsic::connect::WaitForChannelReady(channel));

  LOG(INFO) << "MotionPlannerService Asset was successfully retrieved... "
               "Creating MotionPlannerClient...";
  return std::make_unique<motion_planning::MotionPlannerClient>(
      std::string(world_id), std::make_shared<Channel>(channel));
}

absl::StatusOr<std::string> GetInstalledMotionPlannerServiceAssetVersion(
    intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface* stub,
    std::string_view motion_planner_service_asset_id_name) {
  if (stub == nullptr) {
    return absl::InvalidArgumentError("stub cannot be nullptr.");
  }
  intrinsic_proto::assets::v1::GetInstalledAssetRequest request;
  *request.mutable_id()->mutable_package() =
      kMotionPlannerServiceAssetIdPackage;
  *request.mutable_id()->mutable_name() =
      std::string(motion_planner_service_asset_id_name);
  request.set_view(
      intrinsic_proto::catalog::AssetViewType::ASSET_VIEW_TYPE_FULL);

  intrinsic_proto::assets::v1::InstalledAsset asset;
  grpc::ClientContext context;
  INTR_RETURN_IF_ERROR(
      ToAbslStatus(stub->GetInstalledAsset(&context, request, &asset)));
  return asset.metadata().id_version().version();
}

}  // namespace intrinsic
