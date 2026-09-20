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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_PINCH_GRIPPER_SERVER_IMPL_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_PINCH_GRIPPER_SERVER_IMPL_H_

#include <memory>
#include <optional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/completion_queue.h"
#include "grpcpp/server.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/gripper_equipment.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.grpc.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.pb.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/subscription.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/actuated_gripper_plugin_connection.h"
#include "intrinsic/util/thread/stop_token.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic {
namespace simulation {

// gRPC service and PubSub simulating a Pinch Gripper.
// Here we use a Synchronous Server model (instead of the Callback-based
// Asynchronous Server model as what the hardware counterpart use) because
// here we do not need to periodically send the last command to the simulated
// gripper (e.g. to maintain connection), hence will never block on an RPC.
class SimulatedPinchGripperServerImpl final
    : public ::intrinsic_proto::gripper::PinchGripperServer::Service {
 public:
  SimulatedPinchGripperServerImpl();

  // Not actually creating a simulated pinch gripper based on the specified
  // config, but instead only checks if such gripper has already been registered
  // and return a handle to the gripper.
  ::grpc::Status CreatePinchGripper(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::gripper::CreatePinchGripperRequest* request,
      ::intrinsic_proto::gripper::CreatePinchGripperResponse* response) override
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Reset a simulated gripper as specified by the gripper handle.
  ::grpc::Status ResetPinchGripper(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::gripper::ResetPinchGripperRequest* request,
      ::intrinsic_proto::gripper::ResetPinchGripperResponse* response) override
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Send a command to a simulated gripper as specified by the gripper handle,
  // and return a status of the execution.
  ::grpc::Status CommandPinchGripper(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::gripper::CommandPinchGripperRequest* request,
      ::intrinsic_proto::gripper::CommandPinchGripperResponse* response)
      override ABSL_LOCKS_EXCLUDED(mutex_);

  // Get the status of a simulated gripper as specified by the gripper handle.
  ::grpc::Status GetPinchGripperStatus(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::gripper::GetPinchGripperStatusRequest* request,
      ::intrinsic_proto::gripper::GetPinchGripperStatusResponse* response)
      override ABSL_LOCKS_EXCLUDED(mutex_);

  // Register a new pinch gripper. Plugins will register their connections with
  // this method.
  absl::Status RegisterPinchGripper(ActuatedGripperConnection* pinch_gripper)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Unregisters a pinch gripper. This function will be automatically called
  // during the destruction of a SimulatedPinchGripperConnection.
  absl::Status UnRegisterPinchGripper(absl::string_view pinch_gripper_handle)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Creates and starts a singleton gRPC service, PubSub subscriptions for
  // commands and periodic status publishers.
  static SimulatedPinchGripperServerImpl&
  StartSimulatedPinchGripperServiceSingleton();

 private:
  struct GripperConnections {
    ActuatedGripperConnection* actuated_gripper = nullptr;
    std::optional<Publisher> publisher;
    std::optional<Subscription> subscription;
  };

  absl::Mutex mutex_;

  absl::StatusOr<ActuatedGripperConnection*> GetSimulatedPinchGripperConnection(
      absl::string_view pinch_gripper_handle) ABSL_LOCKS_EXCLUDED(mutex_);

  void OnPubSubCommandReceived(
      absl::string_view handle,
      const ::intrinsic_proto::gripper::PinchGripperCommand& command)
      ABSL_LOCKS_EXCLUDED(mutex_);

  void RunPeriodicStatusPublisher(StopToken stop_token);

  // Available pinch grippers by handle.
  absl::flat_hash_map<std::string, GripperConnections> pinch_grippers_
      ABSL_GUARDED_BY(mutex_);

  PubSub pubsub_;
  Thread status_publisher_thread_;

  std::unique_ptr<::grpc::Server> server_;
  void StartServerAndBlock();
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_PINCH_GRIPPER_SERVER_IMPL_H_
