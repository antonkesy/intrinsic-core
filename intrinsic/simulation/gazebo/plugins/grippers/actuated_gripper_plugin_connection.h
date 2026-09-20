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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_ACTUATED_GRIPPER_PLUGIN_CONNECTION_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_ACTUATED_GRIPPER_PLUGIN_CONNECTION_H_

#include <functional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.pb.h"

namespace intrinsic::simulation {

// Defines the connection between the SimulatedPinchGripperServerImpl and
// Actuated Gripper plugins.
class ActuatedGripperConnection {
 public:
  using PinchGripperCommand = ::intrinsic_proto::gripper::PinchGripperCommand;
  using PinchGripperConfig = ::intrinsic_proto::gripper::PinchGripperConfig;
  using PinchGripperStatus = ::intrinsic_proto::gripper::PinchGripperStatus;
  using SetCommandCallback =
      std::function<absl::Status(const PinchGripperCommand&)>;

  // Constructs a connection to a Gazebo pinch gripper plugin with the
  // associated gripper configuration.
  // The 'set_command' parameter allows the connection to forward commands to
  // the actual plugin. All other information (gripper status, command
  // execution) flows from the plugin to the channel through it's public
  // interface.
  explicit ActuatedGripperConnection(
      const PinchGripperConfig& pinch_gripper_config,
      SetCommandCallback set_command);

  absl::StatusOr<PinchGripperStatus> ExecuteCommand(
      const PinchGripperCommand& command, absl::Time deadline)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Forwards a command directly to the plugin without blocking.
  absl::Status SetCommand(const PinchGripperCommand& command);

  void NotifyCommandExecuted(const PinchGripperStatus& status)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // The gRPC service will call this method to get the status whenever
  // CommandPinchGripper() RPC is invoked, and send the acquired status as the
  // response of the RPC.
  PinchGripperStatus GetPinchGripperStatus() ABSL_LOCKS_EXCLUDED(status_mutex_);

  // The plugin will call this method to set the pinch gripper command execution
  // status.
  void SetPinchGripperStatus(const PinchGripperStatus& status)
      ABSL_LOCKS_EXCLUDED(status_mutex_);

  // Reset the pinch gripper.
  void Reset();

  const PinchGripperConfig& pinch_gripper_config() const;

  const std::string& pinch_gripper_handle() const;

 private:
  PinchGripperConfig pinch_gripper_config_;

  // This function is used to set a received command on the gripper plugin.
  SetCommandCallback set_command_;

  absl::Mutex mutex_;
  bool command_executed_ ABSL_GUARDED_BY(mutex_) = false;
  PinchGripperStatus status_ ABSL_GUARDED_BY(mutex_);

  absl::Mutex status_mutex_;
  PinchGripperStatus last_status_ ABSL_GUARDED_BY(status_mutex_);
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_ACTUATED_GRIPPER_PLUGIN_CONNECTION_H_
