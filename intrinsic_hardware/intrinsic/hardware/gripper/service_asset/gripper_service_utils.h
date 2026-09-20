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

#ifndef INTRINSIC_HARDWARE_GRIPPER_SERVICE_ASSET_GRIPPER_SERVICE_UTILS_H_
#define INTRINSIC_HARDWARE_GRIPPER_SERVICE_ASSET_GRIPPER_SERVICE_UTILS_H_

#include "absl/status/statusor.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/gripper_service_signals.pb.h"
#include "intrinsic/hardware/gripper/service_asset/pinch_gripper_opcua_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/pinch_gripper_realtime_control_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/suction_gripper_opcua_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/suction_gripper_realtime_control_service_config.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"

namespace intrinsic {
namespace gripper {

// Create gripper config for a suction gripper that is controlled using the
// realtime control service.
absl::StatusOr<intrinsic_proto::eoat::GripperConfig>
MakeSuctionGripperRealtimeControlConfig(
    const intrinsic_proto::gripper_service::
        SuctionGripperRealtimeControlServiceConfig& config);

// Create gripper config for a suction gripper that is connected and controlled
// using an OPC UA server.
absl::StatusOr<intrinsic_proto::eoat::GripperConfig>
MakeSuctionGripperOpcuaConfig(
    const intrinsic_proto::gripper_service::SuctionGripperOpcuaServiceConfig&
        config);

// Create gripper config for a pinch gripper that is controlled using the real
// time control service.
absl::StatusOr<intrinsic_proto::eoat::GripperConfig>
MakePinchGripperRealtimeControlConfig(
    const intrinsic_proto::gripper_service::
        PinchGripperRealtimeControlServiceConfig& config);

// Create gripper config for a pinch gripper that is connected and controlled
// using an OPC UA server.
absl::StatusOr<intrinsic_proto::eoat::GripperConfig>
MakePinchGripperOpcuaConfig(
    const intrinsic_proto::gripper_service::PinchGripperOpcuaServiceConfig&
        config);

}  // namespace gripper
}  // namespace intrinsic

#endif  // INTRINSIC_HARDWARE_GRIPPER_SERVICE_ASSET_GRIPPER_SERVICE_UTILS_H_
