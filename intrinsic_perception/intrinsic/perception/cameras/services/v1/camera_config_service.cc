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

#include "intrinsic/perception/cameras/services/v1/camera_config_service.h"

#include <memory>
#include <utility>

#include "absl/base/nullability.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/perception/cameras/camera.h"
#include "intrinsic/perception/cameras/camera_config.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/proto/v1/camera_config.pb.h"
#include "intrinsic/perception/proto/v1/camera_config_service.pb.h"
#include "intrinsic/util/status/status_macros_grpc.h"

namespace intrinsic::perception {

CameraConfigService::CameraConfigService(
    absl_nonnull std::shared_ptr<CameraManager> camera_manager)
    : camera_manager_(std::move(camera_manager)) {}

grpc::Status CameraConfigService::GetCameraConfig(
    grpc::ServerContext* absl_nonnull context,
    const intrinsic_proto::perception::v1::GetCameraConfigRequest* absl_nonnull
        request,
    intrinsic_proto::perception::v1::CameraConfig* absl_nonnull response) {
  INTR_ASSIGN_OR_RETURN_GRPC(const ActiveCameraConfig active_camera,
                             camera_manager_->GetActiveCameraConfig());
  *response = active_camera.ToProto();
  return grpc::Status::OK;
}

}  // namespace intrinsic::perception
