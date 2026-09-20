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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_TYPE_TRAITS_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_TYPE_TRAITS_H_

#include <cstdint>

#include "open62541/types.h"
#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"

template <class OpcuaT>
struct OpcuaToC;

template <>
struct OpcuaToC<UA_Boolean> {
  using type = bool;
};

template <>
struct OpcuaToC<UA_Int16> {
  using type = std::int16_t;
};

template <>
struct OpcuaToC<UA_UInt16> {
  using type = std::uint16_t;
};

template <>
struct OpcuaToC<UA_Int32> {
  using type = std::int32_t;
};

template <>
struct OpcuaToC<UA_UInt32> {
  using type = std::uint32_t;
};

template <>
struct OpcuaToC<UA_Int64> {
  using type = std::int64_t;
};

template <>
struct OpcuaToC<UA_UInt64> {
  using type = std::uint64_t;
};

template <>
struct OpcuaToC<UA_Float> {
  using type = float;
};

template <>
struct OpcuaToC<UA_Double> {
  using type = double;
};

template <>
struct OpcuaToC<UA_String> {
  using type = const char*;
};

template <>
struct OpcuaToC<UA_SByte> {
  using type = int8_t;
};

template <>
struct OpcuaToC<UA_Byte> {
  using type = uint8_t;
};

template <class T>
struct CToOpcua;

template <>
struct CToOpcua<bool> {
  using type = UA_Boolean;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_BOOLEAN];
  static constexpr auto kCreate = []() { return UA_Boolean_new(); };
  static constexpr auto kDelete = [](UA_Boolean* ptr) {
    return UA_Boolean_clear(ptr);
  };
};

template <>
struct CToOpcua<int8_t> {
  using type = UA_SByte;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_SBYTE];
  static constexpr auto kCreate = []() { return UA_SByte_new(); };
  static constexpr auto kDelete = [](UA_SByte* ptr) {
    return UA_SByte_clear(ptr);
  };
};

template <>
struct CToOpcua<uint8_t> {
  using type = UA_Byte;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_BYTE];
  static constexpr auto kCreate = []() { return UA_Byte_new(); };
  static constexpr auto kDelete = [](UA_Byte* ptr) {
    return UA_Byte_clear(ptr);
  };
};

template <>
struct CToOpcua<std::int16_t> {
  using type = UA_Int16;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_INT16];
  static constexpr auto kCreate = []() { return UA_Int16_new(); };
  static constexpr auto kDelete = [](UA_Int16* ptr) {
    return UA_Int16_clear(ptr);
  };
};

template <>
struct CToOpcua<std::uint16_t> {
  using type = UA_UInt16;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_UINT16];
  static constexpr auto kCreate = []() { return UA_UInt16_new(); };
  static constexpr auto kDelete = [](UA_UInt16* ptr) {
    return UA_UInt16_clear(ptr);
  };
};

template <>
struct CToOpcua<std::int32_t> {
  using type = UA_Int32;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_INT32];
  static constexpr auto kCreate = []() { return UA_Int32_new(); };
  static constexpr auto kDelete = [](UA_Boolean* ptr) {
    return UA_Boolean_clear(ptr);
  };
};

template <>
struct CToOpcua<std::uint32_t> {
  using type = UA_UInt32;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_UINT32];
  static constexpr auto kCreate = []() { return UA_UInt32_new(); };
  static constexpr auto kDelete = [](UA_Boolean* ptr) {
    return UA_Boolean_clear(ptr);
  };
};

template <>
struct CToOpcua<std::int64_t> {
  using type = UA_Int64;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_INT64];
  static constexpr auto kCreate = []() { return UA_Int64_new(); };
  static constexpr auto kDelete = [](UA_Boolean* ptr) {
    return UA_Boolean_clear(ptr);
  };
};

template <>
struct CToOpcua<std::uint64_t> {
  using type = UA_UInt64;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_UINT64];
  static constexpr auto kCreate = []() { return UA_UInt64_new(); };
  static constexpr auto kDelete = [](UA_Boolean* ptr) {
    return UA_Boolean_clear(ptr);
  };
};

template <>
struct CToOpcua<float> {
  using type = UA_Float;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_FLOAT];
  static constexpr auto kCreate = []() { return UA_Float_new(); };
  static constexpr auto kDelete = [](UA_Boolean* ptr) {
    return UA_Boolean_clear(ptr);
  };
};

template <>
struct CToOpcua<double> {
  using type = UA_Double;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_DOUBLE];
  static constexpr auto kCreate = []() { return UA_Double_new(); };
  static constexpr auto kDelete = [](UA_Boolean* ptr) {
    return UA_Boolean_clear(ptr);
  };
};

template <>
struct CToOpcua<const char*> {
  using type = UA_String;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_STRING];
  static constexpr auto kCreate = []() { return UA_String_new(); };
};

template <>
struct CToOpcua<char*> {
  using type = UA_String;
  static constexpr UA_DataType const* value = &UA_TYPES[UA_TYPES_STRING];
  static constexpr auto kCreate = []() { return UA_String_new(); };
};

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_TYPE_TRAITS_H_
