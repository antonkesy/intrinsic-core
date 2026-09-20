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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_CONFIG_SERVICE_H_
#define INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_CONFIG_SERVICE_H_

#include <memory>

#include "absl/base/nullability.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/perception/cameras/camera.h"
#include "intrinsic/perception/proto/v1/camera_config.pb.h"
#include "intrinsic/perception/proto/v1/camera_config_service.grpc.pb.h"
#include "intrinsic/perception/proto/v1/camera_config_service.pb.h"

namespace intrinsic::perception {

// Service providing access to the camera configuration.
class CameraConfigService
    : public intrinsic_proto::perception::v1::CameraConfigService::Service {
 public:
  explicit CameraConfigService(
      absl_nonnull std::shared_ptr<CameraManager> camera_manager);

  grpc::Status GetCameraConfig(
      grpc::ServerContext* absl_nonnull context,
      const intrinsic_proto::perception::v1::
          GetCameraConfigRequest* absl_nonnull request,
      intrinsic_proto::perception::v1::CameraConfig* absl_nonnull response)
      override;

 private:
  const absl_nonnull std::shared_ptr<CameraManager> camera_manager_;
};

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_CONFIG_SERVICE_H_
