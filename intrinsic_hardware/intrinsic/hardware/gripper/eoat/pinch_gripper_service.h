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

#ifndef INTRINSIC_HARDWARE_GRIPPER_EOAT_PINCH_GRIPPER_SERVICE_H_
#define INTRINSIC_HARDWARE_GRIPPER_EOAT_PINCH_GRIPPER_SERVICE_H_

#include <memory>

#include "intrinsic/hardware/gripper/eoat/eoat_service.grpc.pb.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_impl.h"

namespace intrinsic::gripper {

// Creates a `PinchGripper` service. The shared pointer to `GripperImpl` object
// is required since it may be shared among different GRPC services.
std::unique_ptr<intrinsic_proto::eoat::PinchGripper::Service>
MakePinchGripperService(std::shared_ptr<::intrinsic::gripper::GripperImpl<
                            ::intrinsic_proto::eoat::PinchGripperConfig>>
                            gripper_impl);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_EOAT_PINCH_GRIPPER_SERVICE_H_
