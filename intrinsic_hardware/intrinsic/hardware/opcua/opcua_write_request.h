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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_WRITE_REQUEST_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_WRITE_REQUEST_H_

#include <initializer_list>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "intrinsic/hardware/opcua/opcua_arrays.h"
#include "intrinsic/hardware/opcua/opcua_type_traits.h"
#include "open62541/types.h"
#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"

// This header contains RAII types for OPCUA types so that de-allocation of
// resources automatically happens in the destructor.
//
// Opcua types are exposed as public members to allow use of opcua functions.
// This means they shouldn't be overwritten without explicitly de-allocating the
// heap allocated memory.
namespace intrinsic::opcua {

struct WriteRequest {
  WriteRequest() { UA_WriteRequest_init(&request); }

  explicit WriteRequest(const char* node_id) noexcept;

  // Initializes the request using given nodes.
  // WARNING: The request stores pointer to the nodes' string identifiers, so
  // the lifetime of the given nodes' string identifiers should exceed that of
  // the request.
  WriteRequest(const std::initializer_list<UA_NodeId>& nodes);
  explicit WriteRequest(const std::vector<UA_NodeId>& nodes);

  explicit WriteRequest(const std::vector<std::string>& nodes) noexcept;

  ~WriteRequest() { UA_WriteRequest_clear(&request); }

  WriteRequest(const WriteRequest& other) {
    UA_WriteRequest_copy(&other.request, &this->request);
  }

  WriteRequest(WriteRequest&& other) {
    UA_WriteRequest_copy(&other.request, &this->request);
  }

  WriteRequest& operator=(const WriteRequest& other) = delete;
  WriteRequest& operator=(WriteRequest&& other) = delete;

  // Sets the value of the node at `request_index` to the given value.
  // Returns true if the value was set successfully.
  // Assumes that the node at `request_index` has already been initialized.
  template <class T>
  bool SetScalarValue(int request_index, T value) {
    if (request_index < 0 || request.nodesToWriteSize <= request_index) {
      return false;
    }
    request.nodesToWrite[request_index].value.hasValue = true;
    T* val_ptr = CToOpcua<T>::kCreate();
    *val_ptr = value;
    UA_Variant_setScalar(&request.nodesToWrite[request_index].value.value,
                         val_ptr, CToOpcua<T>::value);

    return true;
  }

  // Sets the value of the node at `request_index` to the given value.
  // Returns true if the value was set successfully.
  // Assumes that the node at `request_index` has already been initialized.
  template <>
  bool SetScalarValue<const char*>(int request_index, const char* value) {
    if (request_index < 0 || request.nodesToWriteSize <= request_index) {
      return false;
    }
    request.nodesToWrite[0].value.hasValue = true;
    UA_String* val_ptr = CToOpcua<const char*>::kCreate();
    *val_ptr = UA_String_fromChars(value);
    UA_Variant_setScalar(&request.nodesToWrite[request_index].value.value,
                         val_ptr, CToOpcua<const char*>::value);

    return true;
  }

  // Sets the value of the node at `request_index` to the given values at the
  // given offset in the array.
  // Returns true if the value was set successfully.
  // Assumes that the node at `request_index` has already been initialized.
  template <class T>
  bool SetArrayValue(const int request_index, const std::vector<T>& value,
                     const int offset = 0) {
    if (request_index < 0 || request.nodesToWriteSize <= request_index) {
      return false;
    }
    if (offset < 0) {
      return false;
    }
    if (value.empty()) {
      return false;
    }

    const std::string range = [offset, &value]() {
      if (value.size() == 1) {
        // Syntax: [offset]
        return absl::StrCat(offset);
      } else {
        // Syntax: [min_index:max_index]
        return absl::StrCat(offset, ":", (offset + value.size() - 1));
      }
    }();

    request.nodesToWrite[request_index].value.hasValue = true;
    auto [array_ptr, array_size] = ToOpcuaArray<T>(value);

    UA_Variant_init(&request.nodesToWrite[request_index].value.value);
    UA_Variant_setArray(&request.nodesToWrite[request_index].value.value,
                        array_ptr, array_size, CToOpcua<T>::value);
    request.nodesToWrite[request_index].indexRange =
        UA_String_fromChars(range.c_str());

    return true;
  }

  UA_WriteRequest request;
};

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_WRITE_REQUEST_H_
