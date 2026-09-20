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

#include "intrinsic/hardware/gripper/service/generic_gripper_service.h"

#include <cstdint>
#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/service/pinch_gripper_client.h"
#include "intrinsic/hardware/gripper/service/proto/generic_gripper.grpc.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.pb.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_macros_grpc.h"

namespace intrinsic::gripper {

namespace {

static constexpr double kGraspPercentage = 20;
static constexpr double kReleasePercentage = 100;
static constexpr uint64_t kCommandTimeoutSeconds = 10;

absl::Duration CommandTimeout() {
  return absl::Seconds(kCommandTimeoutSeconds);
}

absl::StatusOr<std::unique_ptr<PinchGripperClient>> CreatePinchGripperClient(
    absl::string_view grpc_address) {
  auto params =
      intrinsic::ConnectionParams{.address = std::string(grpc_address)};
  INTR_ASSIGN_OR_RETURN(std::shared_ptr<::intrinsic::Channel> channel,
                        intrinsic::Channel::MakeFromAddress(params));
  return std::make_unique<PinchGripperClient>(channel);
}

class GenericVariablePinchGripperServiceImpl
    : public intrinsic_proto::gripper::GenericGripper::Service {
 public:
  explicit GenericVariablePinchGripperServiceImpl(
      absl::string_view pinch_service_grpc_address)
      : grpc_address_(pinch_service_grpc_address) {}

  ~GenericVariablePinchGripperServiceImpl() override = default;

  grpc::Status Grasp(grpc::ServerContext*,
                     const intrinsic_proto::gripper::GraspRequest*,
                     intrinsic_proto::gripper::GraspResponse*) override {
    INTR_ASSIGN_OR_RETURN_GRPC(auto client,
                               CreatePinchGripperClient(grpc_address_));
    INTR_ASSIGN_OR_RETURN_GRPC(std::string handle,
                               client->CreatePinchGripper());

    intrinsic_proto::gripper::PinchGripperCommand command;
    command.set_position_percentage(kGraspPercentage);
    command.set_command_id(absl::ToUnixMillis(absl::Now()));

    absl::MutexLock lock(mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        last_status_,
        client->CommandPinchGripper(handle, command, CommandTimeout()));
    return grpc::Status::OK;
  }

  grpc::Status Release(grpc::ServerContext*,
                       const intrinsic_proto::gripper::ReleaseRequest*,
                       intrinsic_proto::gripper::ReleaseResponse*) override {
    INTR_ASSIGN_OR_RETURN_GRPC(auto client,
                               CreatePinchGripperClient(grpc_address_));
    INTR_ASSIGN_OR_RETURN_GRPC(std::string handle,
                               client->CreatePinchGripper());

    intrinsic_proto::gripper::PinchGripperCommand command;
    command.set_position_percentage(kReleasePercentage);
    command.set_command_id(absl::ToUnixMillis(absl::Now()));

    absl::MutexLock lock(mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        last_status_,
        client->CommandPinchGripper(handle, command, CommandTimeout()));
    return grpc::Status::OK;
  }

  grpc::Status Command(
      grpc::ServerContext*, const intrinsic_proto::gripper::CommandRequest* req,
      intrinsic_proto::gripper::CommandResponse* response) override {
    INTR_ASSIGN_OR_RETURN_GRPC(auto client,
                               CreatePinchGripperClient(grpc_address_));
    INTR_ASSIGN_OR_RETURN_GRPC(std::string handle,
                               client->CreatePinchGripper());

    const auto command = CreatePinchGripperCommand(*req);

    absl::MutexLock lock(mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        last_status_,
        client->CommandPinchGripper(handle, command, CommandTimeout()));

    response->set_position(last_status_.position());
    response->set_position_reached(last_status_.position_reached());
    return grpc::Status::OK;
  }

  grpc::Status GrippingIndicated(
      grpc::ServerContext*,
      const intrinsic_proto::gripper::GrippingIndicatedRequest*,
      intrinsic_proto::gripper::GrippingIndicatedResponse* response) override {
    absl::MutexLock lock(mutex_);
    // `object_detected` may not be very reliable for very thin objects.
    response->set_indicated(last_status_.object_detected());
    return grpc::Status::OK;
  }

 private:
  std::string grpc_address_;
  intrinsic_proto::gripper::PinchGripperStatus last_status_
      ABSL_GUARDED_BY(mutex_);
  absl::Mutex mutex_;
};

}  // namespace

std::unique_ptr<intrinsic_proto::gripper::GenericGripper::Service>
MakeGenericGripperService(absl::string_view pinch_service_grpc_address) {
  return std::make_unique<GenericVariablePinchGripperServiceImpl>(
      pinch_service_grpc_address);
}

intrinsic_proto::gripper::PinchGripperCommand CreatePinchGripperCommand(
    const intrinsic_proto::gripper::CommandRequest& req) {
  intrinsic_proto::gripper::PinchGripperCommand command;
  command.set_command_id(absl::ToUnixMillis(absl::Now()));

  switch (req.position_command_options_case()) {
    case intrinsic_proto::gripper::CommandRequest::kPositionPercentage:
      command.set_position_percentage(req.position_percentage());
      break;
    case intrinsic_proto::gripper::CommandRequest::kPosition:
      command.set_position(req.position());
      break;
    case intrinsic_proto::gripper::CommandRequest::
        POSITION_COMMAND_OPTIONS_NOT_SET:
      break;
  }

  switch (req.velocity_command_options_case()) {
    case intrinsic_proto::gripper::CommandRequest::kVelocityPercentage:
      command.set_velocity_percentage(req.velocity_percentage());
      break;
    case intrinsic_proto::gripper::CommandRequest::kVelocity:
      command.set_velocity(req.velocity());
      break;
    case intrinsic_proto::gripper::CommandRequest::
        VELOCITY_COMMAND_OPTIONS_NOT_SET:
      break;
  }

  switch (req.effort_command_options_case()) {
    case intrinsic_proto::gripper::CommandRequest::kEffortPercentage:
      command.set_effort_percentage(req.effort_percentage());
      break;
    case intrinsic_proto::gripper::CommandRequest::kEffort:
      command.set_effort(req.effort());
      break;
    case intrinsic_proto::gripper::CommandRequest::
        EFFORT_COMMAND_OPTIONS_NOT_SET:
      break;
  }

  return command;
}

}  // namespace intrinsic::gripper
