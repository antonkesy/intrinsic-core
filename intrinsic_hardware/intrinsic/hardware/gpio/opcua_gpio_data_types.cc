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

#include "intrinsic/hardware/gpio/opcua_gpio_data_types.h"

#include <cstdint>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gpio/v1/signal.pb.h"
#include "intrinsic/hardware/opcua/opcua_write_request.h"
#include "intrinsic/util/status/status_macros.h"
#include "open62541/types.h"
#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"

namespace intrinsic {
namespace gpio {

absl::Status VariantAndSignalTypesMatch(
    const UA_Variant* ua_variant,
    const intrinsic_proto::gpio::v1::SignalValue& signal_value) {
  if (UA_Variant_isEmpty(ua_variant)) {
    return absl::InvalidArgumentError("UA_Variant is empty.");
  }

  if (signal_value.value_case() ==
      intrinsic_proto::gpio::v1::SignalValue::ValueCase::VALUE_NOT_SET) {
    return absl::InvalidArgumentError("SignalValue is not set.");
  }

  auto error_msg = [&signal_value](absl::string_view ua_type) -> std::string {
    const auto& signal_type_name =
        intrinsic_proto::gpio::v1::SignalValue::descriptor()
            ->FindFieldByNumber(signal_value.value_case())
            ->name();
    return absl::StrCat("UA_Variant has type ", ua_type,
                        " but SignalValue contains ", signal_type_name);
  };

  if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_BOOLEAN])) {
    if (!signal_value.has_bool_value()) {
      return absl::InvalidArgumentError(error_msg("bool"));
    }
  } else if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_UINT32])) {
    if (!signal_value.has_unsigned_int_value()) {
      return absl::InvalidArgumentError(error_msg("uint32"));
    }
  } else if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_INT32])) {
    if (!signal_value.has_int_value()) {
      return absl::InvalidArgumentError(error_msg("int32"));
    }
  } else if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_FLOAT])) {
    if (!signal_value.has_float_value()) {
      return absl::InvalidArgumentError(error_msg("float"));
    }
  } else if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_DOUBLE])) {
    if (!signal_value.has_double_value()) {
      return absl::InvalidArgumentError(error_msg("double"));
    }
  } else if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_SBYTE])) {
    if (!signal_value.has_int8_value()) {
      return absl::InvalidArgumentError(error_msg("int8"));
    }
  } else if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_BYTE])) {
    if (!signal_value.has_unsigned_int8_value()) {
      return absl::InvalidArgumentError(error_msg("uint8"));
    }
  }

  return absl::OkStatus();
}

absl::Status MakeVariantValueFromSignalValue(
    const intrinsic_proto::gpio::v1::SignalValue& signal_value,
    UA_Variant* ua_variant) {
  switch (signal_value.value_case()) {
    case intrinsic_proto::gpio::v1::SignalValue::kBoolValue: {
      auto* data_ptr = UA_Boolean_new();
      *data_ptr = signal_value.bool_value();
      UA_Variant_setScalar(ua_variant, data_ptr, &UA_TYPES[UA_TYPES_BOOLEAN]);
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::kUnsignedIntValue: {
      auto* data_ptr = UA_UInt32_new();
      *data_ptr = signal_value.unsigned_int_value();
      UA_Variant_setScalar(ua_variant, data_ptr, &UA_TYPES[UA_TYPES_UINT32]);
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::kIntValue: {
      auto* data_ptr = UA_Int32_new();
      *data_ptr = signal_value.int_value();
      UA_Variant_setScalar(ua_variant, data_ptr, &UA_TYPES[UA_TYPES_INT32]);
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::kFloatValue: {
      auto* data_ptr = UA_Float_new();
      *data_ptr = signal_value.float_value();
      UA_Variant_setScalar(ua_variant, data_ptr, &UA_TYPES[UA_TYPES_FLOAT]);
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::kDoubleValue: {
      auto* data_ptr = UA_Double_new();
      *data_ptr = signal_value.double_value();
      UA_Variant_setScalar(ua_variant, data_ptr, &UA_TYPES[UA_TYPES_DOUBLE]);
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::kInt8Value: {
      const auto value = signal_value.int8_value().value();
      auto CheckForValidRange = [](const decltype(value) v) {
        if (v < -128 || v > 127) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Expected int8 value within [-128, 127]. Got: ", v, "."));
        }
        return absl::OkStatus();
      };

      INTR_RETURN_IF_ERROR(CheckForValidRange(value));

      auto* data_ptr = UA_SByte_new();
      *data_ptr = value;
      UA_Variant_setScalar(ua_variant, data_ptr, &UA_TYPES[UA_TYPES_SBYTE]);
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::kUnsignedInt8Value: {
      const auto value = signal_value.unsigned_int8_value().value();
      auto CheckForValidRange = [](const decltype(value) v) {
        if (v > 255) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Expected unsigned int8 value within [0, 255]. Got: ", v, "."));
        }
        return absl::OkStatus();
      };

      INTR_RETURN_IF_ERROR(CheckForValidRange(value));
      auto* data_ptr = UA_Byte_new();
      *data_ptr = value;
      UA_Variant_setScalar(ua_variant, data_ptr, &UA_TYPES[UA_TYPES_BYTE]);
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::VALUE_NOT_SET:
      break;
  }

  return absl::InvalidArgumentError(
      "Did not receive a signal value proto with one of: bool, uint32, int32, "
      "float or double value.");
}

absl::StatusOr<intrinsic_proto::gpio::v1::SignalValue>
MakeSignalValueFromVariantValue(const UA_Variant& ua_variant) {
  if (UA_Variant_isEmpty(&ua_variant)) {
    return absl::InvalidArgumentError("UA_Variant is empty.");
  }

  // Casting a void pointer to the contained UA_Variant type is the supported
  // data extraction technique from the docs, see
  // http://www.open62541.org/doc/0.2/tutorial_client_firststeps.html

  if (UA_Variant_hasScalarType(&ua_variant, &UA_TYPES[UA_TYPES_BOOLEAN])) {
    intrinsic_proto::gpio::v1::SignalValue signal_value;
    signal_value.set_bool_value(*(UA_Boolean*)ua_variant.data);
    return signal_value;
  }

  if (UA_Variant_hasScalarType(&ua_variant, &UA_TYPES[UA_TYPES_UINT32])) {
    intrinsic_proto::gpio::v1::SignalValue signal_value;
    signal_value.set_unsigned_int_value(*(UA_UInt32*)ua_variant.data);
    return signal_value;
  }

  if (UA_Variant_hasScalarType(&ua_variant, &UA_TYPES[UA_TYPES_INT32])) {
    intrinsic_proto::gpio::v1::SignalValue signal_value;
    signal_value.set_int_value(*(UA_Int32*)ua_variant.data);
    return signal_value;
  }

  if (UA_Variant_hasScalarType(&ua_variant, &UA_TYPES[UA_TYPES_FLOAT])) {
    intrinsic_proto::gpio::v1::SignalValue signal_value;
    signal_value.set_float_value(*(UA_Float*)ua_variant.data);
    return signal_value;
  }

  if (UA_Variant_hasScalarType(&ua_variant, &UA_TYPES[UA_TYPES_DOUBLE])) {
    intrinsic_proto::gpio::v1::SignalValue signal_value;
    signal_value.set_double_value(*(UA_Double*)ua_variant.data);
    return signal_value;
  }

  if (UA_Variant_hasScalarType(&ua_variant, &UA_TYPES[UA_TYPES_BYTE])) {
    intrinsic_proto::gpio::v1::SignalValue signal_value;
    signal_value.mutable_unsigned_int8_value()->set_value(
        *(UA_Byte*)ua_variant.data);
    return signal_value;
  }

  if (UA_Variant_hasScalarType(&ua_variant, &UA_TYPES[UA_TYPES_SBYTE])) {
    intrinsic_proto::gpio::v1::SignalValue signal_value;
    signal_value.mutable_int8_value()->set_value(*(UA_SByte*)ua_variant.data);
    return signal_value;
  }

  return absl::InvalidArgumentError(
      absl::StrCat("Variant must be of type bool, uint32, int32, float or "
                   "double to convert to SignalValue. Received: ",
                   ua_variant.type->typeName));
}

absl::StatusOr<std::vector<intrinsic_proto::gpio::v1::SignalValue>>
MakeArraySignalValueFromVariantValue(const UA_Variant& ua_variant) {
  if (UA_Variant_isEmpty(&ua_variant)) {
    return absl::InvalidArgumentError("UA_Variant is empty.");
  }
  if (UA_Variant_isScalar(&ua_variant)) {
    return absl::InvalidArgumentError("UA_Variant is scalar, expected array.");
  }

  if (ua_variant.arrayLength == 0) {
    return absl::InvalidArgumentError("UA_Variant array is empty.");
  }

  // An opcua server may set the array dimensions to 0 for single dimensional
  // arrays.
  if (ua_variant.arrayDimensionsSize > 1) {
    return absl::InvalidArgumentError(
        absl::StrCat("UA_Variant array has ", ua_variant.arrayDimensionsSize,
                     " dimensions, expected 0 or 1."));
  }

  std::vector<intrinsic_proto::gpio::v1::SignalValue> values;
  values.reserve(ua_variant.arrayLength);
  for (int i = 0; i < ua_variant.arrayLength; ++i) {
    // The lifetime of `scalar_variant` is limited within the loop. Hence, we
    // don't need to dynamically allocate any memory and free it up later.
    UA_Variant scalar_variant;
    UA_Variant_init(&scalar_variant);

    const void* data_ptr =
        (char*)ua_variant.data + (i * ua_variant.type->memSize);
    scalar_variant.type = ua_variant.type;
    scalar_variant.data = const_cast<void*>(data_ptr);

    INTR_ASSIGN_OR_RETURN(auto signal_value,
                          MakeSignalValueFromVariantValue(scalar_variant));
    values.push_back(signal_value);
  }

  return values;
}

absl::StatusOr<intrinsic_proto::gpio::v1::SignalType> VariantTypeToSignalType(
    const UA_Variant* ua_variant) {
  if (UA_Variant_isEmpty(ua_variant)) {
    return absl::InvalidArgumentError("UA_Variant is empty.");
  }

  if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_BOOLEAN])) {
    return intrinsic_proto::gpio::v1::SIGNAL_TYPE_BOOL;
  }

  if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_UINT32])) {
    return intrinsic_proto::gpio::v1::SIGNAL_TYPE_UNSIGNED_INT;
  }

  if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_INT32])) {
    return intrinsic_proto::gpio::v1::SIGNAL_TYPE_INT;
  }

  if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_FLOAT])) {
    return intrinsic_proto::gpio::v1::SIGNAL_TYPE_FLOAT;
  }

  if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_DOUBLE])) {
    return intrinsic_proto::gpio::v1::SIGNAL_TYPE_DOUBLE;
  }

  if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_BYTE])) {
    return intrinsic_proto::gpio::v1::SIGNAL_TYPE_UNSIGNED_INT8;
  }

  if (UA_Variant_hasScalarType(ua_variant, &UA_TYPES[UA_TYPES_SBYTE])) {
    return intrinsic_proto::gpio::v1::SIGNAL_TYPE_INT8;
  }

  return absl::InvalidArgumentError(
      absl::StrCat("Only bool, uint32, int32, float and double node types"
                   "are supported. Instead got type: ",
                   ua_variant->type->typeName));
}

absl::Status AddScalarValueToRequest(
    const int node_index, const intrinsic_proto::gpio::v1::SignalValue& value,
    ::intrinsic::opcua::WriteRequest& request) {
  if (node_index < 0 || node_index >= request.request.nodesToWriteSize) {
    return absl::InvalidArgumentError(
        absl::StrCat("Index ", node_index, " out of range [0, ",
                     request.request.nodesToWriteSize, ")."));
  }

  INTR_RETURN_IF_ERROR(
      intrinsic::gpio::MakeVariantValueFromSignalValue(
          value, &request.request.nodesToWrite[node_index].value.value))
      .LogError();

  request.request.nodesToWrite[node_index].value.hasValue = true;

  return absl::OkStatus();
}

absl::Status AddArrayValueToRequest(
    const int node_index, const intrinsic_proto::gpio::v1::SignalValue& value,
    const int array_index, ::intrinsic::opcua::WriteRequest& request) {
  if (node_index < 0 || node_index >= request.request.nodesToWriteSize) {
    return absl::InvalidArgumentError(
        absl::StrCat("Index ", node_index, " out of range [0, ",
                     request.request.nodesToWriteSize, ")."));
  }

  switch (value.value_case()) {
    case intrinsic_proto::gpio::v1::SignalValue::kBoolValue:
      if (!request.SetArrayValue<bool>(node_index, {value.bool_value()},
                                       array_index)) {
        return absl::InvalidArgumentError("Failed to set bool array value.");
      }
      return absl::OkStatus();
    case intrinsic_proto::gpio::v1::SignalValue::kUnsignedIntValue:
      if (!request.SetArrayValue<uint32_t>(
              node_index, {value.unsigned_int_value()}, array_index)) {
        return absl::InvalidArgumentError("Failed to set uint32 array value.");
      }
      return absl::OkStatus();
    case intrinsic_proto::gpio::v1::SignalValue::kIntValue:
      if (!request.SetArrayValue<int32_t>(node_index, {value.int_value()},
                                          array_index)) {
        return absl::InvalidArgumentError("Failed to set int32 array value.");
      }
      return absl::OkStatus();
    case intrinsic_proto::gpio::v1::SignalValue::kFloatValue:
      if (!request.SetArrayValue<float>(node_index, {value.float_value()},
                                        array_index)) {
        return absl::InvalidArgumentError("Failed to set float array value.");
      }
      return absl::OkStatus();
    case intrinsic_proto::gpio::v1::SignalValue::kDoubleValue:
      if (!request.SetArrayValue<double>(node_index, {value.double_value()},
                                         array_index)) {
        return absl::InvalidArgumentError("Failed to set double array value.");
      }
      return absl::OkStatus();
    case intrinsic_proto::gpio::v1::SignalValue::kInt8Value: {
      const auto v8 = value.int8_value().value();
      auto CheckForValidRange = [](const decltype(v8) v) {
        if (v < -128 || v > 127) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Expected int8 value within [-128, 127]. Got: ", v, "."));
        }
        return absl::OkStatus();
      };

      INTR_RETURN_IF_ERROR(CheckForValidRange(v8));
      if (!request.SetArrayValue<int8_t>(node_index, {static_cast<int8_t>(v8)},
                                         array_index)) {
        return absl::InvalidArgumentError("Failed to set int8 array value.");
      }
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::kUnsignedInt8Value: {
      const auto v8 = value.unsigned_int8_value().value();
      auto CheckForValidRange = [](const decltype(v8) v) {
        if (v > 255) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Expected unsigned int8 value within [0, 255]. Got: ", v, "."));
        }
        return absl::OkStatus();
      };

      INTR_RETURN_IF_ERROR(CheckForValidRange(v8));
      if (!request.SetArrayValue<uint8_t>(
              node_index, {static_cast<uint8_t>(v8)}, array_index)) {
        return absl::InvalidArgumentError("Failed to set uint8 array value.");
      }
      return absl::OkStatus();
    }
    case intrinsic_proto::gpio::v1::SignalValue::VALUE_NOT_SET:
      break;
  }

  return absl::InvalidArgumentError("Signal value not set.");
}

};  // namespace gpio
};  // namespace intrinsic
