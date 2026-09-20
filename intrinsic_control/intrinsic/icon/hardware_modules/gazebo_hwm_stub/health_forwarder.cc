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

#include "intrinsic/icon/hardware_modules/gazebo_hwm_stub/health_forwarder.h"

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/client_context.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/icon/utils/aggregated_health_to_state.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.grpc.pb.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros_grpc.h"

namespace intrinsic::icon {

void HealthForwarder::SetResourceHealthStub(
    std::unique_ptr<intrinsic_proto::simulation::AggregatedResourceHealth::Stub>
        new_stub) {
  absl::MutexLock l(mutex_);
  stub_ = std::move(new_stub);
}

grpc::Status HealthForwarder::GetState(
    grpc::ServerContext* context,
    const intrinsic_proto::services::v1::GetStateRequest* request,
    intrinsic_proto::services::v1::SelfState* response) {
  absl::MutexLock l(mutex_);
  if (stub_ == nullptr) {
    return ToGrpcStatus(
        absl::UnavailableError("Upstream ServiceState stub is unset, this "
                               "ideally shouldn't happen"));
  }
  std::unique_ptr<::grpc::ClientContext> ctx =
      grpc::ClientContext::FromServerContext(*context);
  intrinsic_proto::simulation::AggregatedResourceHealthStatusRequest
      upstream_request;
  upstream_request.set_resource_name(name_);
  intrinsic_proto::simulation::AggregatedResourceHealthStatusResponse
      upstream_response;
  INTR_RETURN_IF_ERROR_GRPC(
      stub_->CheckHealth(ctx.get(), upstream_request, &upstream_response));
  *response = ConvertAggregatedResourceHealthToServiceState(
      upstream_response.response());
  return ToGrpcStatus(absl::OkStatus());
}

grpc::Status HealthForwarder::Enable(
    grpc::ServerContext* context,
    const intrinsic_proto::services::v1::EnableRequest* request,
    intrinsic_proto::services::v1::EnableResponse* response) {
  absl::MutexLock l(mutex_);
  if (stub_ == nullptr) {
    return ToGrpcStatus(
        absl::UnavailableError("Upstream ServiceState stub is unset, this "
                               "ideally shouldn't happen"));
  }
  // Check the state to see if we need to clear faults
  std::unique_ptr<::grpc::ClientContext> ctx_check_health =
      grpc::ClientContext::FromServerContext(*context);
  intrinsic_proto::simulation::AggregatedResourceHealthStatusRequest
      upstream_check_health_request;
  upstream_check_health_request.set_resource_name(name_);
  intrinsic_proto::simulation::AggregatedResourceHealthStatusResponse
      upstream_check_health_response;
  INTR_RETURN_IF_ERROR_GRPC(
      stub_->CheckHealth(ctx_check_health.get(), upstream_check_health_request,
                         &upstream_check_health_response));

  if (upstream_check_health_response.response().status().state() ==
      intrinsic_proto::simulation::OperationalStatus::FAULTED) {
    std::unique_ptr<::grpc::ClientContext> ctx_clear_faults =
        grpc::ClientContext::FromServerContext(*context);
    intrinsic_proto::simulation::AggregatedResourceClearFaultsRequest
        clear_faults_request;
    clear_faults_request.set_resource_name(name_);
    intrinsic_proto::simulation::AggregatedResourceClearFaultsResponse
        clear_faults_response;
    INTR_RETURN_IF_ERROR_GRPC(stub_->ClearFaults(
        ctx_clear_faults.get(), clear_faults_request, &clear_faults_response));
  }

  std::unique_ptr<::grpc::ClientContext> ctx_enable =
      grpc::ClientContext::FromServerContext(*context);
  intrinsic_proto::simulation::AggregatedResourceEnableRequest
      upstream_enable_request;
  upstream_enable_request.set_resource_name(name_);
  intrinsic_proto::simulation::AggregatedResourceEnableResponse
      upstream_enable_response;
  INTR_RETURN_IF_ERROR_GRPC(stub_->Enable(
      ctx_enable.get(), upstream_enable_request, &upstream_enable_response));

  return ToGrpcStatus(absl::OkStatus());
}

grpc::Status HealthForwarder::Disable(
    grpc::ServerContext* context,
    const intrinsic_proto::services::v1::DisableRequest* request,
    intrinsic_proto::services::v1::DisableResponse* response) {
  absl::MutexLock l(mutex_);
  if (stub_ == nullptr) {
    return ToGrpcStatus(
        absl::UnavailableError("Upstream ServiceState stub is unset, this "
                               "ideally shouldn't happen"));
  }
  std::unique_ptr<::grpc::ClientContext> ctx =
      grpc::ClientContext::FromServerContext(*context);
  intrinsic_proto::simulation::AggregatedResourceDisableRequest
      upstream_request;
  upstream_request.set_resource_name(name_);
  intrinsic_proto::simulation::AggregatedResourceDisableResponse
      upstream_response;
  if (auto upstream_status =
          stub_->Disable(ctx.get(), upstream_request, &upstream_response);
      !upstream_status.ok()) {
    return upstream_status;
  }
  return ToGrpcStatus(absl::OkStatus());
}

}  // namespace intrinsic::icon
