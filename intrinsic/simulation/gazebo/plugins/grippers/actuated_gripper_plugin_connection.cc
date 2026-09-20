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

#include "intrinsic/simulation/gazebo/plugins/grippers/actuated_gripper_plugin_connection.h"

#include <functional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/substitute.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::simulation {

namespace {

using PinchGripperCommand = ActuatedGripperConnection::PinchGripperCommand;
using PinchGripperStatus = ActuatedGripperConnection::PinchGripperStatus;
}  // namespace

ActuatedGripperConnection::ActuatedGripperConnection(
    const PinchGripperConfig& pinch_gripper_config,
    SetCommandCallback set_command)
    : pinch_gripper_config_(pinch_gripper_config),
      set_command_(std::move(set_command)) {}

absl::StatusOr<PinchGripperStatus> ActuatedGripperConnection::ExecuteCommand(
    const PinchGripperCommand& command, absl::Time deadline) {
  // Store the command and the flag indicating successful execution internally.
  // After releasing the mutex, these parameters can be modified through a
  // different member function and thread. After the mutex has been released,
  // Gazebo can notify the connection object about the successful execution.
  {
    absl::MutexLock lock(mutex_);
    command_executed_ = false;
  }

  // Set the command on the plugin. After this, the plugin will update the
  // gripper state in Gazebo.
  INTR_RETURN_IF_ERROR(set_command_(command));

  // Wait for the execution to terminate or for a time-out.
  const absl::Time start = absl::Now();
  if (mutex_.LockWhenWithDeadline(absl::Condition(&command_executed_),
                                  deadline)) {
    // Copy the status before returning it since it is also protected by this
    // mutex.
    auto local_status = status_;
    mutex_.unlock();
    return local_status;
  } else {
    mutex_.unlock();
    const absl::Duration lock_duration = absl::Now() - start;
    return absl::DeadlineExceededError(absl::Substitute(
        "Failed to execute gripper command within deadline. Tried for $0 ms.",
        absl::ToDoubleMilliseconds(lock_duration)));
  }
}

absl::Status ActuatedGripperConnection::SetCommand(
    const PinchGripperCommand& command) {
  return set_command_(command);
}

void ActuatedGripperConnection::NotifyCommandExecuted(
    const PinchGripperStatus& status) {
  absl::MutexLock lock(mutex_);
  status_ = status;
  command_executed_ = true;
}

PinchGripperStatus ActuatedGripperConnection::GetPinchGripperStatus() {
  absl::ReaderMutexLock lock(status_mutex_);
  return last_status_;
}

void ActuatedGripperConnection::SetPinchGripperStatus(
    const PinchGripperStatus& status) {
  absl::MutexLock lock(status_mutex_);
  last_status_ = status;
}

void ActuatedGripperConnection::Reset() {
  // This is a no-op.
}

const ::intrinsic_proto::gripper::PinchGripperConfig&
ActuatedGripperConnection::pinch_gripper_config() const {
  return pinch_gripper_config_;
}

const std::string& ActuatedGripperConnection::pinch_gripper_handle() const {
  return pinch_gripper_config_.name();
}

}  // namespace intrinsic::simulation
