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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_DYNAMIC_RECONFIGURATION_SERVICE_H_
#define INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_DYNAMIC_RECONFIGURATION_SERVICE_H_

#include <memory>

#include "absl/base/nullability.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/assets/services/proto/v1/dynamic_reconfiguration.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/dynamic_reconfiguration.pb.h"
#include "intrinsic/perception/cameras/camera.h"

namespace intrinsic::perception {

// Service that receives configuration updates whenever the corresponding asset
// instances configuration was changed.
class CameraDynamicReconfigurationService
    : public intrinsic_proto::services::v1::DynamicReconfiguration::Service {
 public:
  explicit CameraDynamicReconfigurationService(
      absl_nonnull std::shared_ptr<CameraManager> camera_manager);

  grpc::Status ApplyConfiguration(
      grpc::ServerContext* absl_nonnull context,
      const intrinsic_proto::services::v1::
          ApplyConfigurationRequest* absl_nonnull request,
      intrinsic_proto::services::v1::ApplyConfigurationResponse* absl_nonnull
          response) override;

 private:
  const absl_nonnull std::shared_ptr<CameraManager> camera_manager_;
};

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_DYNAMIC_RECONFIGURATION_SERVICE_H_
