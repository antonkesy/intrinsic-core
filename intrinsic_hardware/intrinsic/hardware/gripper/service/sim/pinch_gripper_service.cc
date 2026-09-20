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

#include "intrinsic/hardware/gripper/service/sim/pinch_gripper_service.h"

#include <memory>
#include <utility>

#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.grpc.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.pb.h"
#include "intrinsic/hardware/gripper/service/sim/sim_pinch_gripper_impl.h"
#include "intrinsic/util/status/status_conversion_grpc.h"

namespace intrinsic::gripper::simulation {

namespace {

class SimPinchGripperServiceImpl
    : public ::intrinsic_proto::gripper::PinchGripperServer::Service {
 public:
  explicit SimPinchGripperServiceImpl(
      std::shared_ptr<SimPinchGripperImpl> gripper_impl)
      : gripper_impl_(std::move(gripper_impl)) {}

  ~SimPinchGripperServiceImpl() override = default;

  grpc::Status CreatePinchGripper(
      grpc::ServerContext* context,
      const intrinsic_proto::gripper::CreatePinchGripperRequest* request,
      intrinsic_proto::gripper::CreatePinchGripperResponse* response) override {
    return ToGrpcStatus(gripper_impl_->CreatePinchGripper(*request, response));
  }

  grpc::Status ResetPinchGripper(
      grpc::ServerContext* context,
      const intrinsic_proto::gripper::ResetPinchGripperRequest* request,
      intrinsic_proto::gripper::ResetPinchGripperResponse* response) override {
    return ToGrpcStatus(gripper_impl_->ResetPinchGripper(*request, response));
  }

  grpc::Status CommandPinchGripper(
      grpc::ServerContext* context,
      const intrinsic_proto::gripper::CommandPinchGripperRequest* request,
      intrinsic_proto::gripper::CommandPinchGripperResponse* response)
      override {
    return ToGrpcStatus(gripper_impl_->CommandPinchGripper(*request, response));
  }

  grpc::Status GetPinchGripperStatus(
      grpc::ServerContext* context,
      const intrinsic_proto::gripper::GetPinchGripperStatusRequest* request,
      intrinsic_proto::gripper::GetPinchGripperStatusResponse* response)
      override {
    return ToGrpcStatus(
        gripper_impl_->GetPinchGripperStatus(*request, response));
  }

 private:
  std::shared_ptr<SimPinchGripperImpl> gripper_impl_;
};

}  // namespace

std::unique_ptr<intrinsic_proto::gripper::PinchGripperServer::Service>
MakeSimPinchGripperService(std::shared_ptr<SimPinchGripperImpl> gripper_impl) {
  return std::make_unique<SimPinchGripperServiceImpl>(std::move(gripper_impl));
}

}  // namespace intrinsic::gripper::simulation
