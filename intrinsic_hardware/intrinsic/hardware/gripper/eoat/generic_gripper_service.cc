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

#include "intrinsic/hardware/gripper/eoat/generic_gripper_service.h"

#include <memory>
#include <utility>

#include "absl/log/die_if_null.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_impl.h"
#include "intrinsic/hardware/gripper/service/proto/generic_gripper.grpc.pb.h"
#include "intrinsic/hardware/gripper/service/proto/generic_gripper.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros_grpc.h"

namespace intrinsic::gripper {

template <typename Config>
class GenericGripperServiceImpl
    : public intrinsic_proto::gripper::GenericGripper::Service {
 public:
  explicit GenericGripperServiceImpl(
      std::shared_ptr<GripperImpl<Config>> gripper_impl)
      : gripper_impl_(std::move(ABSL_DIE_IF_NULL(gripper_impl))) {}

  ~GenericGripperServiceImpl() override = default;

  grpc::Status Grasp(grpc::ServerContext* context,
                     const intrinsic_proto::gripper::GraspRequest*,
                     intrinsic_proto::gripper::GraspResponse*) override {
    return gripper_impl_->Grasp();
  }

  grpc::Status Release(grpc::ServerContext* context,
                       const intrinsic_proto::gripper::ReleaseRequest* req,
                       intrinsic_proto::gripper::ReleaseResponse*) override {
    INTR_RETURN_IF_ERROR_GRPC(ToAbslStatus(gripper_impl_->Release()));

    // Blow off if requested but only relevant for suction grippers.
    if constexpr (std::is_same_v<
                      Config, ::intrinsic_proto::eoat::SuctionGripperConfig>) {
      if (req->enable_blowoff()) {
        ::intrinsic_proto::eoat::BlowOffRequest blow_off_request;
        blow_off_request.set_turn_on(true);
        return gripper_impl_->BlowOff(blow_off_request);
      }
    }

    return grpc::Status::OK;
  }

  grpc::Status Command(grpc::ServerContext* context,
                       const intrinsic_proto::gripper::CommandRequest*,
                       intrinsic_proto::gripper::CommandResponse*) override {
    return grpc::Status(grpc::StatusCode::UNIMPLEMENTED,
                        "Command is not supported for DIO grippers.");
  }

  grpc::Status GrippingIndicated(
      grpc::ServerContext* context,
      const intrinsic_proto::gripper::GrippingIndicatedRequest*,
      intrinsic_proto::gripper::GrippingIndicatedResponse* response) override {
    intrinsic_proto::eoat::GrippingIndicatedResponse eoat_response;
    auto status = gripper_impl_->GrippingIndicated(eoat_response);
    if (status.ok()) {
      response->set_indicated(eoat_response.indicated());
    }
    return status;
  }

 private:
  std::shared_ptr<GripperImpl<Config>> gripper_impl_;
};

std::unique_ptr<intrinsic_proto::gripper::GenericGripper::Service>
MakeGenericGripperService(std::shared_ptr<::intrinsic::gripper::GripperImpl<
                              ::intrinsic_proto::eoat::PinchGripperConfig>>
                              gripper_impl) {
  return std::make_unique<
      GenericGripperServiceImpl<::intrinsic_proto::eoat::PinchGripperConfig>>(
      std::move(gripper_impl));
}

std::unique_ptr<intrinsic_proto::gripper::GenericGripper::Service>
MakeGenericGripperService(std::shared_ptr<::intrinsic::gripper::GripperImpl<
                              ::intrinsic_proto::eoat::SuctionGripperConfig>>
                              gripper_impl) {
  return std::make_unique<
      GenericGripperServiceImpl<::intrinsic_proto::eoat::SuctionGripperConfig>>(
      std::move(gripper_impl));
}

}  // namespace intrinsic::gripper
