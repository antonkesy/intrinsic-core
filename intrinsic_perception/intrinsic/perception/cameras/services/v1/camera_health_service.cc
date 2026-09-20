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

#include "intrinsic/perception/cameras/services/v1/camera_health_service.h"

#include <memory>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/perception/cameras/camera.h"
#include "intrinsic/util/status/status_builder_grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros_grpc.h"

namespace intrinsic {
namespace perception {

namespace {
grpc::Status SetDisabled(bool disabled, ConcurrentCamera& camera) {
  // As described in the proto comments, we need to return a failed
  // precondition error, if we are in a faulty state.
  const absl::Status faults_status =
      camera.Call</*error_on_disabled=*/false>(&Camera::GetFaultsStatus);
  if (!faults_status.ok() && !absl::IsUnimplemented(faults_status)) {
    return FailedPreconditionErrorBuilderGrpc() << faults_status.message();
  }
  camera.SetDisabled(disabled);
  return grpc::Status::OK;
}
}  // namespace

CameraHealthService::CameraHealthService(
    absl_nonnull std::shared_ptr<CameraManager> camera_manager)
    : camera_manager_(std::move(camera_manager)) {}

grpc::Status CameraHealthService::GetState(
    grpc::ServerContext* absl_nonnull context,
    const intrinsic_proto::services::v1::GetStateRequest* absl_nonnull request,
    intrinsic_proto::services::v1::SelfState* absl_nonnull response) {
  const absl::StatusOr<std::shared_ptr<ConcurrentCamera>> camera =
      camera_manager_->GetActiveCamera();
  if (!camera.ok()) {
    if (absl::IsFailedPrecondition(camera.status())) {
      response->set_state_code(
          intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED);
      response->mutable_extended_status()->set_title("Camera not connected");
      response->mutable_extended_status()->mutable_user_report()->set_message(
          "Please use the camera's settings panel to connect to a camera.");
      return grpc::Status::OK;
    }
    return ToGrpcStatus(camera.status());
  }
  const absl::Status faults_status =
      (*camera)->Call</*error_on_disabled=*/false>(&Camera::GetFaultsStatus);
  if (faults_status.ok()) {
    const bool is_disabled = (*camera)->IsDisabled();
    response->set_state_code(
        is_disabled
            ? intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED
            : intrinsic_proto::services::v1::SelfState::STATE_CODE_ENABLED);
    return grpc::Status::OK;
  }

  if (absl::IsUnimplemented(faults_status)) {
    response->set_state_code(
        intrinsic_proto::services::v1::SelfState::STATE_CODE_UNSPECIFIED);
    response->mutable_extended_status()->mutable_status_code()->set_code(
        faults_status.raw_code());
    return ToGrpcStatus(faults_status);
  }
  response->set_state_code(
      intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR);
  response->mutable_extended_status()->set_title("Camera errored");
  response->mutable_extended_status()->mutable_user_report()->set_message(
      faults_status.message());

  return grpc::Status::OK;
}

grpc::Status CameraHealthService::Enable(
    grpc::ServerContext* absl_nonnull context,
    const intrinsic_proto::services::v1::EnableRequest* absl_nonnull request,
    intrinsic_proto::services::v1::EnableResponse* absl_nonnull response) {
  INTR_ASSIGN_OR_RETURN_GRPC(const std::shared_ptr<ConcurrentCamera> camera,
                             camera_manager_->GetActiveCamera());
  // Check if the camera is in error and clear it if so.
  const absl::Status faults_status =
      camera->Call</*error_on_disabled=*/false>(&Camera::GetFaultsStatus);
  if (!faults_status.ok() && !absl::IsFailedPrecondition(faults_status)) {
    INTR_RETURN_IF_ERROR_GRPC(
        camera->Call</*error_on_disabled=*/false>(&Camera::ClearFaults))
        .LogError();
  }
  return intrinsic::perception::SetDisabled(/*disabled=*/false, *camera);
}

grpc::Status CameraHealthService::Disable(
    grpc::ServerContext* absl_nonnull context,
    const intrinsic_proto::services::v1::DisableRequest* absl_nonnull request,
    intrinsic_proto::services::v1::DisableResponse* absl_nonnull response) {
  INTR_ASSIGN_OR_RETURN_GRPC(const std::shared_ptr<ConcurrentCamera> camera,
                             camera_manager_->GetActiveCamera());
  return intrinsic::perception::SetDisabled(/*disabled=*/true, *camera);
}

}  // namespace perception
}  // namespace intrinsic
