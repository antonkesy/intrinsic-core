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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_HEALTH_SERVICE_H_
#define INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_HEALTH_SERVICE_H_

#include <memory>

#include "absl/base/nullability.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/perception/cameras/camera.h"

namespace intrinsic {
namespace perception {

class CameraHealthService
    : public intrinsic_proto::services::v1::ServiceState::Service {
 public:
  CameraHealthService() = delete;

  explicit CameraHealthService(
      absl_nonnull std::shared_ptr<CameraManager> camera_manager);

  grpc::Status GetState(
      grpc::ServerContext* absl_nonnull context,
      const intrinsic_proto::services::v1::GetStateRequest* absl_nonnull
          request,
      intrinsic_proto::services::v1::SelfState* absl_nonnull response) override;

  grpc::Status Enable(
      grpc::ServerContext* absl_nonnull context,
      const intrinsic_proto::services::v1::EnableRequest* absl_nonnull request,
      intrinsic_proto::services::v1::EnableResponse* absl_nonnull response)
      override;

  grpc::Status Disable(
      grpc::ServerContext* absl_nonnull context,
      const intrinsic_proto::services::v1::DisableRequest* absl_nonnull request,
      intrinsic_proto::services::v1::DisableResponse* absl_nonnull response)
      override;

 private:
  const absl_nonnull std::shared_ptr<CameraManager> camera_manager_;
};

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_HEALTH_SERVICE_H_
