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

#ifndef INTRINSIC_HARDWARE_GRIPPER_SERVICE_GENERIC_GRIPPER_SERVICE_H__
#define INTRINSIC_HARDWARE_GRIPPER_SERVICE_GENERIC_GRIPPER_SERVICE_H__

#include <memory>

#include "absl/strings/string_view.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/service/proto/generic_gripper.grpc.pb.h"

namespace intrinsic::gripper {

// Returns Generic Gripper service given the address of the Pinch Gripper
// service.
std::unique_ptr<intrinsic_proto::gripper::GenericGripper::Service>
MakeGenericGripperService(absl::string_view pinch_service_grpc_address);

// Creates a PinchGripperCommand from a CommandRequest.
intrinsic_proto::gripper::PinchGripperCommand CreatePinchGripperCommand(
    const intrinsic_proto::gripper::CommandRequest& req);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_SERVICE_GENERIC_GRIPPER_SERVICE_H__
