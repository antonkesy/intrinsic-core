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

#include "intrinsic/hardware/gripper/service/health_service.h"

#include <memory>
#include <string>

#include "absl/strings/str_cat.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/hardware/gripper/service/pinch_gripper_server_async_callback_impl.h"
#include "intrinsic/resources/proto/resource_health.grpc.pb.h"
#include "intrinsic/resources/proto/resource_health.pb.h"

namespace intrinsic::gripper {

// TODO: b/388333075 - Remove ResourceHealth references.
namespace {

intrinsic_proto::resources::ResourceHealthStatusResponse
ConvertServiceStateToResourceHealth(
    const intrinsic_proto::services::v1::SelfState& state) {
  intrinsic_proto::resources::ResourceHealthStatusResponse health_status;
  if (state.state_code() ==
      intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED) {
    health_status.mutable_status()->set_state(
        intrinsic_proto::resources::OperationalStatus::DISABLED);
  } else if (state.state_code() ==
             intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR) {
    health_status.mutable_status()->set_state(
        intrinsic_proto::resources::OperationalStatus::FAULTED);
  } else if (state.state_code() ==
             intrinsic_proto::services::v1::SelfState::STATE_CODE_ENABLED) {
    health_status.mutable_status()->set_state(
        intrinsic_proto::resources::OperationalStatus::ENABLED);
  } else {
    health_status.mutable_status()->set_state(
        intrinsic_proto::resources::OperationalStatus::UNSPECIFIED);
  }

  std::string explanation = "";
  if (!state.extended_status().title().empty()) {
    absl::StrAppend(&explanation, "Title: ", state.extended_status().title());
  }
  if (!state.extended_status().user_report().message().empty()) {
    if (!explanation.empty()) {
      absl::StrAppend(&explanation, "; ");
    }
    absl::StrAppend(&explanation, "Message: ",
                    state.extended_status().user_report().message());
  }
  if (!state.extended_status().user_report().instructions().empty()) {
    if (!explanation.empty()) {
      absl::StrAppend(&explanation, "; ");
    }
    absl::StrAppend(&explanation, "Instructions: ",
                    state.extended_status().user_report().instructions());
  }
  health_status.mutable_status()->set_explanation(explanation);

  return health_status;
}

class HealthService
    : public intrinsic_proto::resources::ResourceHealth::Service,
      public intrinsic_proto::services::v1::ServiceState::Service {
 public:
  explicit HealthService(
      std::shared_ptr<PinchGripperServerAsyncCallbackImpl> gripper_impl)
      : gripper_impl_(gripper_impl) {}

  ~HealthService() override = default;

  // ResourceHealth implementation
  grpc::Status CheckHealth(
      grpc::ServerContext* context,
      const intrinsic_proto::resources::ResourceHealthStatusRequest* request,
      intrinsic_proto::resources::ResourceHealthStatusResponse* response)
      override {
    *response = ConvertServiceStateToResourceHealth(gripper_impl_->GetState());
    return grpc::Status::OK;
  }

  grpc::Status Enable(
      grpc::ServerContext* context,
      const intrinsic_proto::resources::ResourceEnableRequest* request,
      intrinsic_proto::resources::ResourceEnableResponse* response) override {
    return gripper_impl_->Enable();
  }

  grpc::Status Disable(
      grpc::ServerContext* context,
      const intrinsic_proto::resources::ResourceDisableRequest* request,
      intrinsic_proto::resources::ResourceDisableResponse* response) override {
    return gripper_impl_->Disable();
  }

  grpc::Status ClearFaults(
      grpc::ServerContext* context,
      const intrinsic_proto::resources::ResourceClearFaultsRequest* request,
      intrinsic_proto::resources::ResourceClearFaultsResponse* response)
      override {
    return gripper_impl_->ClearFaults();
  }

  // ServiceState implementation
  grpc::Status GetState(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::GetStateRequest* request,
      intrinsic_proto::services::v1::SelfState* response) override {
    *response = gripper_impl_->GetState();
    return grpc::Status::OK;
  }

  grpc::Status Enable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::EnableRequest* request,
      intrinsic_proto::services::v1::EnableResponse* response) override {
    if (intrinsic_proto::services::v1::SelfState state =
            gripper_impl_->GetState();
        state.state_code() ==
        intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR) {
      // If the gripper is faulted, clear the faults before enabling.
      if (auto status = gripper_impl_->ClearFaults(); !status.ok()) {
        return status;
      }
    }
    return gripper_impl_->Enable();
  }

  grpc::Status Disable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::DisableRequest* request,
      intrinsic_proto::services::v1::DisableResponse* response) override {
    return gripper_impl_->Disable();
  }

 private:
  std::shared_ptr<PinchGripperServerAsyncCallbackImpl> gripper_impl_;
};

}  // namespace

std::unique_ptr<intrinsic_proto::resources::ResourceHealth::Service>
MakeVariablePinchGripperHealthService(
    std::shared_ptr<PinchGripperServerAsyncCallbackImpl> gripper_impl) {
  return std::make_unique<HealthService>(gripper_impl);
}

std::unique_ptr<intrinsic_proto::services::v1::ServiceState::Service>
MakeVariablePinchGripperServiceState(
    std::shared_ptr<PinchGripperServerAsyncCallbackImpl> gripper_impl) {
  return std::make_unique<HealthService>(gripper_impl);
}

}  // namespace intrinsic::gripper
