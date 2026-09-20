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

#ifndef INTRINSIC_HARDWARE_GRIPPER_SERVICE_PINCH_GRIPPER_CLIENT_H_
#define INTRINSIC_HARDWARE_GRIPPER_SERVICE_PINCH_GRIPPER_CLIENT_H_

#include <sys/types.h>

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "grpcpp/grpcpp.h"  // IWYU pragma: keep
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.grpc.pb.h"
#include "intrinsic/util/grpc/channel.h"

namespace intrinsic::gripper {

// A thread-safe synchronous client for the Pinch Gripper server.
class PinchGripperClient {
 public:
  static constexpr absl::Duration kDefaultTimeout = absl::Seconds(5);

  explicit PinchGripperClient(std::shared_ptr<::intrinsic::Channel> channel);

  PinchGripperClient(const PinchGripperClient&) = delete;
  PinchGripperClient& operator=(const PinchGripperClient&) = delete;

  // Establish a connection to the Pinch Gripper specified by the config.
  absl::StatusOr<std::string> CreatePinchGripper(bool test_only = false);

  // Reset the Pinch Gripper identified with the specified handle.
  absl::Status ResetPinchGripper(
      absl::string_view gripper_handle,
      absl::Duration pinch_gripper_client_timeout = kDefaultTimeout);

  // Send a command to the Pinch Gripper with the specified handle.
  absl::StatusOr<intrinsic_proto::gripper::PinchGripperStatus>
  CommandPinchGripper(absl::string_view gripper_handle,
                      intrinsic_proto::gripper::PinchGripperCommand command,
                      absl::Duration pinch_gripper_client_timeout);

  // Get the status of the Pinch Gripper with the specified handle.
  absl::StatusOr<intrinsic_proto::gripper::PinchGripperStatus>
  GetPinchGripperStatus(
      absl::string_view gripper_handle,
      absl::Duration pinch_gripper_client_timeout = kDefaultTimeout);

 private:
  std::unique_ptr<::intrinsic_proto::gripper::PinchGripperServer::Stub> stub_;
  std::shared_ptr<::intrinsic::Channel> channel_;
};

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_SERVICE_PINCH_GRIPPER_CLIENT_H_
