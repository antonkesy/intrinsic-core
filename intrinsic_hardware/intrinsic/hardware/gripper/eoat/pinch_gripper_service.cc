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

#include "intrinsic/hardware/gripper/eoat/pinch_gripper_service.h"

#include <memory>
#include <utility>

#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.grpc.pb.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_impl.h"

namespace intrinsic::gripper {

using ::grpc::Status;

class PinchGripperImpl : public intrinsic_proto::eoat::PinchGripper::Service {
 public:
  explicit PinchGripperImpl(std::shared_ptr<::intrinsic::gripper::GripperImpl<
                                ::intrinsic_proto::eoat::PinchGripperConfig>>
                                gripper_impl)
      : gripper_impl_(std::move(gripper_impl)) {}

  ~PinchGripperImpl() override = default;

  Status Grasp(grpc::ServerContext* context,
               const intrinsic_proto::eoat::GraspRequest* request,
               intrinsic_proto::eoat::GraspResponse* response) override {
    return gripper_impl_->Grasp();
  }

  Status Release(grpc::ServerContext* context,
                 const intrinsic_proto::eoat::ReleaseRequest* request,
                 intrinsic_proto::eoat::ReleaseResponse* response) override {
    return gripper_impl_->Release();
  }

  Status GrippingIndicated(
      grpc::ServerContext* context,
      const intrinsic_proto::eoat::GrippingIndicatedRequest* request,
      intrinsic_proto::eoat::GrippingIndicatedResponse* response) override {
    return gripper_impl_->GrippingIndicated(*response);
  }

 private:
  std::shared_ptr<::intrinsic::gripper::GripperImpl<
      ::intrinsic_proto::eoat::PinchGripperConfig>>
      gripper_impl_;
};

std::unique_ptr<intrinsic_proto::eoat::PinchGripper::Service>
MakePinchGripperService(std::shared_ptr<::intrinsic::gripper::GripperImpl<
                            ::intrinsic_proto::eoat::PinchGripperConfig>>
                            gripper_impl) {
  return std::make_unique<PinchGripperImpl>(gripper_impl);
}

}  // namespace intrinsic::gripper
