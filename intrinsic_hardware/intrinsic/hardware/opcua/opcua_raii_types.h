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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_RAII_TYPES_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_RAII_TYPES_H_

#include <cstring>
#include <string>

#include "absl/strings/string_view.h"
#include "intrinsic/hardware/opcua/opcua_read_request.h"    // IWYU pragma: keep
#include "intrinsic/hardware/opcua/opcua_read_response.h"   // IWYU pragma: keep
#include "intrinsic/hardware/opcua/opcua_type_traits.h"     // IWYU pragma: keep
#include "intrinsic/hardware/opcua/opcua_write_request.h"   // IWYU pragma: keep
#include "intrinsic/hardware/opcua/opcua_write_response.h"  // IWYU pragma: keep
#include "open62541/types.h"
#include "open62541/types_generated_handling.h"

// This header contains RAII types for OPCUA types so that de-allocation of
// resources automatically happens in the destructor.
//
// Opcua types are exposed as public members to allow use of opcua functions.
// This means they shouldn't be overwritten without explicitly de-allocating the
// heap allocated memory.
namespace intrinsic::opcua {

inline std::string ToString(const UA_String& ua_string) {
  if (ua_string.data == nullptr) {
    return {};
  }
  std::string str;
  str.resize(ua_string.length);
  std::memcpy(str.data(), ua_string.data, ua_string.length);

  return str;
}

struct LocalizedText {
  LocalizedText() { UA_LocalizedText_init(&text); }
  ~LocalizedText() { UA_LocalizedText_clear(&text); }

  // If needed, copy constructor and assignment can be provided using
  // `UA_LocalizedText_copy`.
  LocalizedText(const LocalizedText& other) = delete;
  LocalizedText& operator=(const LocalizedText& other) = delete;

  LocalizedText(LocalizedText&& other) = delete;
  LocalizedText& operator=(LocalizedText&& other) = delete;

  absl::string_view GetString() const {
    return {reinterpret_cast<char*>(text.text.data), text.text.length};
  }
  UA_LocalizedText text;
};

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_RAII_TYPES_H_
