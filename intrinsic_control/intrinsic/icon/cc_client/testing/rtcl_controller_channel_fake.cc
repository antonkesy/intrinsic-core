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

#include "intrinsic/icon/cc_client/testing/rtcl_controller_channel_fake.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "grpcpp/channel.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/cc_client/testing/channel_fake.h"
#include "intrinsic/icon/cc_client/testing/rtcl_controller_fake_robot_connection.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/proto/v1/service.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/testing/in_process_server.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

RtclControllerChannelFake::Builder&
RtclControllerChannelFake::Builder::WithServerName(
    absl::string_view server_name) {
  server_name_ = std::string(server_name);
  return *this;
}

RtclControllerChannelFake::Builder&
RtclControllerChannelFake::Builder::WithInitialRobotStatus(
    const intrinsic_proto::icon::v1::GetStatusResponse& robot_status) {
  initial_robot_status_ = robot_status;
  return *this;
}

RtclControllerChannelFake::Builder&
RtclControllerChannelFake::Builder::StartFaulted() {
  start_state_ = OperationalState::kFaulted;
  return *this;
}

RtclControllerChannelFake::Builder&
RtclControllerChannelFake::Builder::StartDisabled() {
  start_state_ = OperationalState::kDisabled;
  return *this;
}

// Specifies the default behaviors for the next session. Subsequent calls
// create session expectations for subsequent sessions.
RtclControllerChannelFake::Builder&
RtclControllerChannelFake::Builder::OnNextSession(
    const SessionWill& session_behaviors) {
  session_expectations_.push_back(session_behaviors);
  return *this;
}

absl::StatusOr<std::shared_ptr<RtclControllerChannelFake>>
RtclControllerChannelFake::Builder::Build(
    absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs) {
  INTR_ASSIGN_OR_RETURN(auto connection,
                        RtclControllerFakeRobotConnection::Create(
                            server_name_, mode_, initial_robot_status_,
                            part_configs, session_expectations_));
  switch (start_state_) {
    case intrinsic::icon::OperationalState::kEnabled:
      // Nothing to do, fake connections start enabled.
      break;
    case OperationalState::kDisabled:
      INTR_RETURN_IF_ERROR(
          connection->MutableFakeOperationalState().SetDisabled());
      break;
    case OperationalState::kFaulted:
      connection->InduceFault("ChannelFake started in faulted state.");
      break;
  }
  return std::shared_ptr<RtclControllerChannelFake>(
      new RtclControllerChannelFake(std::move(connection), options_));
}

RtclControllerChannelFake::RtclControllerChannelFake(
    std::unique_ptr<RtclControllerFakeRobotConnection> robot_connection,
    InProcessApplicationLayerServer::Options options)
    : robot_connection_(std::move(robot_connection)),
      server_(std::make_unique<InProcessApplicationLayerServer>(
          *robot_connection_, options)) {}

std::shared_ptr<grpc::Channel> RtclControllerChannelFake::GetChannel() const {
  return server_->GetChannel();
}

void RtclControllerChannelFake::SetRobotStatus(
    const intrinsic_proto::icon::v1::GetStatusResponse& robot_status) {
  robot_connection_->UpdateCurrentStatus(robot_status);
}

PartPropertyMap RtclControllerChannelFake::GetPartPropertiesTestOnly() const {
  return robot_connection_->GetPartPropertiesTestOnly();
}

void RtclControllerChannelFake::SetPartPropertiesTestOnly(
    const PartPropertyMap& part_properties) {
  return robot_connection_->SetPartPropertiesTestOnly(part_properties);
}

int RtclControllerChannelFake::NumIconServiceRestarts() const {
  return server_ != nullptr ? server_->IconImplFactoryCalls() : 0;
}

}  // namespace intrinsic::icon
