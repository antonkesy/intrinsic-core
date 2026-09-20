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

#include "intrinsic/simulation/gazebo/plugins/grippers/pinch_gripper_server_impl.h"

#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "absl/base/no_destructor.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/strings/strip.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpc/grpc.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server.h"
#include "grpcpp/server_builder.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/topic_config.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/actuated_gripper_plugin_connection.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros_grpc.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic {
namespace simulation {
namespace {

using ::intrinsic_proto::gripper::CommandPinchGripperRequest;
using ::intrinsic_proto::gripper::CommandPinchGripperResponse;
using ::intrinsic_proto::gripper::CreatePinchGripperRequest;
using ::intrinsic_proto::gripper::CreatePinchGripperResponse;
using ::intrinsic_proto::gripper::GetPinchGripperStatusRequest;
using ::intrinsic_proto::gripper::GetPinchGripperStatusResponse;
using ::intrinsic_proto::gripper::PinchGripperCommand;
using ::intrinsic_proto::gripper::PinchGripperStatus;
using ::intrinsic_proto::gripper::ResetPinchGripperRequest;
using ::intrinsic_proto::gripper::ResetPinchGripperResponse;

constexpr absl::Duration kDefaultSimulatedGripperExecutionWaitTime =
    absl::Seconds(7);
constexpr absl::Duration kStatusPublishInterval =
    absl::Milliseconds(50);  // 20 Hz

}  // namespace

SimulatedPinchGripperServerImpl::SimulatedPinchGripperServerImpl()
    : status_publisher_thread_([this](StopToken stop_token) {
        RunPeriodicStatusPublisher(stop_token);
      }) {}

::grpc::Status SimulatedPinchGripperServerImpl::CreatePinchGripper(
    ::grpc::ServerContext* context, const CreatePinchGripperRequest* request,
    CreatePinchGripperResponse* response) {
  auto gripper_handle = request->config().name();
  absl::ReaderMutexLock lock(mutex_);
  if (!pinch_grippers_.contains(gripper_handle)) {
    const auto error_msg = absl::StrFormat(
        "Couldn't find pinch gripper with handle: %s", gripper_handle);
    LOG(ERROR) << error_msg;
    return ToGrpcStatus(absl::NotFoundError(error_msg));
  }
  response->set_gripper_handle(gripper_handle);
  response->set_created_before(
      true);  // for simulation case, we never really create the
              // gripper/connection in the RPC (it has always been existed
              // before)
  LOG(INFO) << "Simulated creation of pinch gripper with handle: "
            << gripper_handle;
  return ::grpc::Status::OK;
}

::grpc::Status SimulatedPinchGripperServerImpl::ResetPinchGripper(
    ::grpc::ServerContext* context, const ResetPinchGripperRequest* request,
    ResetPinchGripperResponse* response) {
  LOG(INFO) << "Executing simulated ResetPinchGripper request: " << *request;
  INTR_ASSIGN_OR_RETURN_GRPC(
      auto pinch_gripper_connection,
      GetSimulatedPinchGripperConnection(request->gripper_handle()),
      _.LogError());

  pinch_gripper_connection->Reset();
  return ::grpc::Status::OK;
}

::grpc::Status SimulatedPinchGripperServerImpl::CommandPinchGripper(
    ::grpc::ServerContext* context, const CommandPinchGripperRequest* request,
    CommandPinchGripperResponse* response) {
  LOG(INFO) << "Executing CommandPinchGripper request: " << *request;
  INTR_ASSIGN_OR_RETURN_GRPC(
      auto pinch_gripper_connection,
      GetSimulatedPinchGripperConnection(request->gripper_handle()),
      _.LogError());

  // TODO(haukeheibel): What happens if the gRPC times out earlier than this?
  absl::Duration execution_timeout;
  if (request->has_execution_timeout()) {
    execution_timeout = absl::Seconds(request->execution_timeout().seconds());
  } else {
    execution_timeout = kDefaultSimulatedGripperExecutionWaitTime;
  }
  const absl::Time deadline = absl::Now() + execution_timeout;

  // The plugin is waiting internally.
  LOG(INFO) << "Commanding gripper motion.";
  INTR_ASSIGN_OR_RETURN_GRPC(
      *response->mutable_status(),
      pinch_gripper_connection->ExecuteCommand(request->command(), deadline));

  return ::grpc::Status::OK;
}

::grpc::Status SimulatedPinchGripperServerImpl::GetPinchGripperStatus(
    ::grpc::ServerContext* context, const GetPinchGripperStatusRequest* request,
    GetPinchGripperStatusResponse* response) {
  INTR_ASSIGN_OR_RETURN_GRPC(
      auto pinch_gripper_connection,
      GetSimulatedPinchGripperConnection(request->gripper_handle()),
      _.LogError());
  *response->mutable_status() =
      pinch_gripper_connection->GetPinchGripperStatus();
  return ::grpc::Status::OK;
}

absl::Status SimulatedPinchGripperServerImpl::RegisterPinchGripper(
    ActuatedGripperConnection* pinch_gripper) {
  absl::MutexLock lock(mutex_);
  const auto& pinch_gripper_handle = pinch_gripper->pinch_gripper_handle();
  LOG(INFO) << "Registering simulated pinch gripper with handle:\n\t"
            << pinch_gripper_handle;
  if (pinch_grippers_.contains(pinch_gripper_handle)) {
    return absl::InvalidArgumentError(
        "A simulated pinch gripper with the provided handle has already been "
        "registered. Please verify that the same pinch gripper (with the "
        "same handle/identifier) is not being added twice in the simulation.");
  }

  // Determine status and command topic names for PubSub
  std::string status_topic;
  std::string command_topic;
  const auto& config = pinch_gripper->pinch_gripper_config();
  if (config.has_generic_pinch_gripper_config() &&
      config.generic_pinch_gripper_config().has_additional_config()) {
    const auto& add_cfg =
        config.generic_pinch_gripper_config().additional_config();
    status_topic = add_cfg.status_topic();
    command_topic = add_cfg.command_topic();
  }
  if (status_topic.empty()) {
    status_topic = absl::StrCat(pinch_gripper_handle, "/gripper/status");
  }
  if (command_topic.empty()) {
    command_topic = absl::StrCat(pinch_gripper_handle, "/gripper/command");
  }

  GripperConnections connections;
  connections.actuated_gripper = pinch_gripper;

  // Create status publisher
  auto pub_or = pubsub_.CreatePublisher(status_topic, TopicConfig());
  if (pub_or.ok()) {
    connections.publisher = std::move(*pub_or);
    LOG(INFO) << "Created PubSub status publisher for '" << pinch_gripper_handle
              << "' on topic: " << status_topic;
  } else {
    LOG(ERROR) << "Failed to create PubSub status publisher on '"
               << status_topic << "': " << pub_or.status();
  }

  // Create command subscriber
  auto sub_or = pubsub_.CreateSubscription<PinchGripperCommand>(
      command_topic, TopicConfig(),
      [this, handle = pinch_gripper_handle](const PinchGripperCommand& cmd) {
        OnPubSubCommandReceived(handle, cmd);
      },
      [](absl::string_view, absl::Status err) {
        LOG_EVERY_N_SEC(ERROR, 3)
            << "Error receiving gripper command packet: " << err;
      });
  if (sub_or.ok()) {
    connections.subscription = std::move(*sub_or);
    LOG(INFO) << "Created PubSub command subscriber for '"
              << pinch_gripper_handle << "' on topic: " << command_topic;
  } else {
    LOG(ERROR) << "Failed to create PubSub command subscriber on '"
               << command_topic << "': " << sub_or.status();
  }

  pinch_grippers_[pinch_gripper_handle] = std::move(connections);

  return absl::OkStatus();
}

absl::Status SimulatedPinchGripperServerImpl::UnRegisterPinchGripper(
    absl::string_view pinch_gripper_handle) {
  LOG(INFO) << "Un-Registering simulated pinch gripper with handle:\n"
            << pinch_gripper_handle;
  {
    absl::MutexLock lock(mutex_);
    auto it = pinch_grippers_.find(pinch_gripper_handle);
    if (it == pinch_grippers_.end()) {
      return absl::NotFoundError(
          "A simulated pinch gripper with the provided identifier has not been "
          "found.");
    }
    pinch_grippers_.erase(it);
  }
  return absl::OkStatus();
}

void SimulatedPinchGripperServerImpl::OnPubSubCommandReceived(
    absl::string_view handle, const PinchGripperCommand& command) {
  absl::ReaderMutexLock lock(mutex_);
  const auto it = pinch_grippers_.find(handle);
  if (it == pinch_grippers_.end() || it->second.actuated_gripper == nullptr) {
    return;
  }
  if (auto status = it->second.actuated_gripper->SetCommand(command);
      !status.ok()) {
    LOG(WARNING) << "PubSub command execution failed on '" << handle
                 << "': " << status;
  }
}

void SimulatedPinchGripperServerImpl::RunPeriodicStatusPublisher(
    StopToken stop_token) {
  while (!stop_token.stop_requested()) {
    {
      absl::ReaderMutexLock lock(mutex_);
      for (const auto& [handle, connections] : pinch_grippers_) {
        if (connections.publisher.has_value() &&
            connections.actuated_gripper != nullptr) {
          PinchGripperStatus status =
              connections.actuated_gripper->GetPinchGripperStatus();
          // TODO(b/555012436): Unify ServiceState::SelfState with this state
          // for simulated grippers.
          status.set_gripper_enabled(true);
          auto pub_status = connections.publisher->Publish(status);
          if (!pub_status.ok()) {
            LOG_EVERY_N_SEC(WARNING, 3)
                << "Failed to publish status on '"
                << connections.publisher->TopicName() << "': " << pub_status;
          }
        }
      }
    }
    absl::SleepFor(kStatusPublishInterval);
  }
}

absl::StatusOr<ActuatedGripperConnection*>
SimulatedPinchGripperServerImpl::GetSimulatedPinchGripperConnection(
    absl::string_view pinch_gripper_handle) {
  absl::ReaderMutexLock lock(mutex_);
  const auto iter = pinch_grippers_.find(pinch_gripper_handle);
  if (iter == pinch_grippers_.end() ||
      iter->second.actuated_gripper == nullptr) {
    return NotFoundErrorBuilder() << "Couldn't find pinch gripper with handle: "
                                  << pinch_gripper_handle;
  }
  return iter->second.actuated_gripper;
}

void SimulatedPinchGripperServerImpl::StartServerAndBlock() {
  const char* server_address = getenv("PINCH_GRIPPER_SERVICE_ADDRESS");
  if (server_address == nullptr) {
    server_address = "0.0.0.0:12393";
  }
  ::grpc::ServerBuilder builder;
  builder.AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);
  builder.AddListeningPort(
      server_address,
      ::grpc::InsecureServerCredentials());  // NOLINT (insecure)
  builder.RegisterService(this);
  server_ = builder.BuildAndStart();
  CHECK(server_) << "Can't set up SimulatedPinchGripperServerImpl";
  LOG(INFO) << "Started simulated pinch gripper service at: " << server_address;
  server_->Wait();
}

SimulatedPinchGripperServerImpl&
SimulatedPinchGripperServerImpl::StartSimulatedPinchGripperServiceSingleton() {
  struct BackgroundPinchgripperServer {
    BackgroundPinchgripperServer()
        : thread(std::bind_front(
              &SimulatedPinchGripperServerImpl::StartServerAndBlock, &server)) {
    }
    SimulatedPinchGripperServerImpl server;
    Thread thread;
  };
  static absl::NoDestructor<BackgroundPinchgripperServer> kPinchGripperService;
  return kPinchGripperService->server;
}

}  // namespace simulation
}  // namespace intrinsic
