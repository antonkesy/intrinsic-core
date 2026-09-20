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

#ifndef INTRINSIC_HARDWARE_GRIPPER_SERVICE_GENERIC_PINCH_GRIPPER_UTILS_H_
#define INTRINSIC_HARDWARE_GRIPPER_SERVICE_GENERIC_PINCH_GRIPPER_UTILS_H_

#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/perception/proto/v1/settings.pb.h"

namespace intrinsic::gripper {

// Convert `in_value` which is typically a physical quantity either specified in
// SI units (e.g. meters for position, m/s for velocity, etc.) or in percentage
// with bounds specified as `in_min` (minimum value) and `in_max`
// (maximum value) into a hardware value with bounds specified as
// `out_min` (minimum value) and `out_max` (maximum value). If `in_min` and
// `in_max` are equal, the function returns the average between `out_min` and
// `out_max`.
int64_t ConvertToHardwareValue(double in_value, double in_min, double in_max,
                               int64_t out_min, int64_t out_max);

// Convert quantities in SI units (`si_in_value`) to hardware values based on
// `config`.
absl::StatusOr<int64_t> ConvertSiToHardwareValue(
    double si_in_value,
    const intrinsic_proto::gripper::PhysicalQuantityFormatProperties& config);

// Convert quantities in percentage (`percentage_in_value`) to hardware values
// based on `config`.
absl::StatusOr<int64_t> ConvertPercentageToHardwareValue(
    double percentage_in_value,
    const intrinsic_proto::gripper::PhysicalQuantityFormatProperties& config);

absl::Status CheckConfigValidity(
    const intrinsic_proto::gripper::PinchGripperPhysicalConfig& config,
    bool validate_si_position_config, bool validate_si_velocity_config,
    bool validate_si_effort_config);

double PositionFromHardwareValue(
    const intrinsic_proto::gripper::PinchGripperPhysicalConfig& config,
    uint8_t hardware_value);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_SERVICE_GENERIC_PINCH_GRIPPER_UTILS_H_
