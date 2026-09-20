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

#ifndef INTRINSIC_HARDWARE_GRIPPER_EOAT_GRIPPER_UTILS_H_
#define INTRINSIC_HARDWARE_GRIPPER_EOAT_GRIPPER_UTILS_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"

namespace intrinsic::gripper {

absl::Status GpioSrvEndpointIsValid(
    const intrinsic_proto::eoat::GpioServiceEndpoint& srv_endpoint);

// Returns a union of all the write signals specified in the config
absl::flat_hash_set<std::string> SignalsToClaim(
    const intrinsic_proto::eoat::PinchGripperConfig& config);

// Returns a union of all the signals to read specified in the config.
absl::flat_hash_set<std::string> SignalsToRead(
    const intrinsic_proto::eoat::PinchGripperConfig& config);

absl::Status GripperConfigIsValid(
    const intrinsic_proto::eoat::PinchGripperConfig& config);

// Returns a union of all the write signals specified in the config
absl::flat_hash_set<std::string> SignalsToClaim(
    const intrinsic_proto::eoat::SuctionGripperConfig& config);

// Returns a union of all the signals to read specified in the config.
absl::flat_hash_set<std::string> SignalsToRead(
    const intrinsic_proto::eoat::SuctionGripperConfig& config);

absl::Status GripperConfigIsValid(
    const intrinsic_proto::eoat::SuctionGripperConfig& config);

// Returns absl::OkStatus() if all the signals specified in `config` are valid.
absl::Status SignalsAreValid(
    const intrinsic_proto::eoat::PinchGripperConfig& config,
    const intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse&
        valid_signals);

// Returns absl::OkStatus() if all the signals specified in `config` are valid.
absl::Status SignalsAreValid(
    const intrinsic_proto::eoat::SuctionGripperConfig& config,
    const intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse&
        valid_signals);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_EOAT_GRIPPER_UTILS_H_
