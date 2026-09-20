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

#ifndef INTRINSIC_HARDWARE_GRIPPER_SERVICE_HEALTH_SERVICE_H_
#define INTRINSIC_HARDWARE_GRIPPER_SERVICE_HEALTH_SERVICE_H_

#include <memory>

#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"
#include "intrinsic/hardware/gripper/service/pinch_gripper_server_async_callback_impl.h"
#include "intrinsic/resources/proto/resource_health.grpc.pb.h"

namespace intrinsic::gripper {

// TODO(b/388333075): Remove ResourceHealth references.
std::unique_ptr<intrinsic_proto::resources::ResourceHealth::Service>
MakeVariablePinchGripperHealthService(
    std::shared_ptr<PinchGripperServerAsyncCallbackImpl> gripper_impl);

std::unique_ptr<intrinsic_proto::services::v1::ServiceState::Service>
MakeVariablePinchGripperServiceState(
    std::shared_ptr<PinchGripperServerAsyncCallbackImpl> gripper_impl);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_SERVICE_HEALTH_SERVICE_H_
