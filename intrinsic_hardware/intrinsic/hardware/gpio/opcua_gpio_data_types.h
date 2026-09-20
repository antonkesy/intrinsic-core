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

#ifndef INTRINSIC_HARDWARE_GPIO_OPCUA_GPIO_DATA_TYPES_H_
#define INTRINSIC_HARDWARE_GPIO_OPCUA_GPIO_DATA_TYPES_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gpio/v1/signal.pb.h"
#include "intrinsic/hardware/opcua/opcua_write_request.h"
#include "open62541/types.h"

namespace intrinsic {
namespace gpio {

// Checks that data types for UA_Variant and SignalValue match.
absl::Status VariantAndSignalTypesMatch(
    const UA_Variant* ua_variant,
    const intrinsic_proto::gpio::v1::SignalValue& signal_value);

// Creates UA_Variant with data populated from the corresponding SignalValue.
// ** Important notes about memory lifetime **
// - `ua_variant` should not contain any heap-allocated data when passed as an
//   argument to this function. Since `ua_variant` would get overwritten with
//   the new value, any previously heap-allocated data would leak.
// - If absl::okStatus() is returned, the caller is responsible for freeing up
//   the heap-allocated value using `UA_Variant_clear()`.
absl::Status MakeVariantValueFromSignalValue(
    const intrinsic_proto::gpio::v1::SignalValue& signal_value,
    UA_Variant* ua_variant);

// Returns SignalValue with data populated from the corresponding UA_Variant.
absl::StatusOr<intrinsic_proto::gpio::v1::SignalValue>
MakeSignalValueFromVariantValue(const UA_Variant& ua_variant);

// Returns a vector of SignalValues with data populated from the corresponding
// UA_Variant.
// Should only be called if `ua_variant` contains a single dimension array.
absl::StatusOr<std::vector<intrinsic_proto::gpio::v1::SignalValue>>
MakeArraySignalValueFromVariantValue(const UA_Variant& ua_variant);

// Returns SignalValue type for the corresponding UA_Variant type.
absl::StatusOr<intrinsic_proto::gpio::v1::SignalType> VariantTypeToSignalType(
    const UA_Variant* ua_variant);

// Adds a value for the scalar node in the write request.
// Assumes that the node at `node_index` has already been initialized in the
// write request.
absl::Status AddScalarValueToRequest(
    int node_index, const intrinsic_proto::gpio::v1::SignalValue& value,
    ::intrinsic::opcua::WriteRequest& request);

// Adds a value to the given index for the array node in the write request.
// Assumes that the node at `node_index` has already been initialized in the
// write request.
absl::Status AddArrayValueToRequest(
    int node_index, const intrinsic_proto::gpio::v1::SignalValue& value,
    int array_index, ::intrinsic::opcua::WriteRequest& request);

};  // namespace gpio
};  // namespace intrinsic

#endif  // INTRINSIC_HARDWARE_GPIO_OPCUA_GPIO_DATA_TYPES_H_
