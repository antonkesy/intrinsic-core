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

#include "intrinsic/perception/cameras/services/v1/camera_dynamic_reconfiguration_service.h"

#include <memory>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/log/log.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/perception/cameras/camera.h"
#include "intrinsic/perception/cameras/camera_config.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/proto/v1/camera_config.pb.h"
#include "intrinsic/util/status/status_macros_grpc.h"

namespace intrinsic::perception {

CameraDynamicReconfigurationService::CameraDynamicReconfigurationService(
    absl_nonnull std::shared_ptr<CameraManager> camera_manager)
    : camera_manager_(std::move(camera_manager)) {}

grpc::Status CameraDynamicReconfigurationService::ApplyConfiguration(
    grpc::ServerContext* absl_nonnull context,
    const intrinsic_proto::services::v1::ApplyConfigurationRequest* absl_nonnull
        request,
    intrinsic_proto::services::v1::ApplyConfigurationResponse* absl_nonnull
        response) {
  if (!request->has_configuration()) {
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "ApplyConfigurationRequest missing configuration");
  }
  INTR_ASSIGN_OR_RETURN_GRPC(
      const intrinsic_proto::perception::v1::CameraConfig camera_config_proto,
      UnpackCameraConfig(request->configuration()));
  INTR_ASSIGN_OR_RETURN_GRPC(
      const ActiveCameraConfig active_camera,
      ActiveCameraConfig::FromProto(camera_config_proto));
  LOG(INFO) << "Received new camera config in ApplyConfiguration():\n"
            << camera_config_proto;
  camera_manager_->SetActiveCamera(active_camera);

  return grpc::Status::OK;
}

}  // namespace intrinsic::perception
