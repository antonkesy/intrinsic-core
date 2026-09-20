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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_WRITE_RESPONSE_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_WRITE_RESPONSE_H_

#include <utility>

#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"

// This header contains RAII types for OPCUA types so that de-allocation of
// resources automatically happens in the destructor.
//
// Opcua types are exposed as public members to allow use of opcua functions.
// This means they shouldn't be overwritten without explicitly de-allocating the
// heap allocated memory.
namespace intrinsic::opcua {

struct WriteResponse {
  WriteResponse() { UA_WriteResponse_init(&response); }
  ~WriteResponse() { UA_WriteResponse_clear(&response); }

  WriteResponse(const WriteResponse& other) {
    UA_WriteResponse_copy(&other.response, &this->response);
  }

  WriteResponse(WriteResponse&& other) {
    this->response = std::move(other.response);
    // Initialize the moved-from response to avoid use-after-move errors and
    // double frees.
    UA_WriteResponse_init(&other.response);
  }

  WriteResponse& operator=(const WriteResponse& other) {
    if (&other == this) {
      return *this;
    }
    // Free the memory of the existing response before copying.
    UA_WriteResponse_clear(&this->response);
    UA_WriteResponse_copy(&other.response, &this->response);
    return *this;
  }

  WriteResponse& operator=(WriteResponse&& other) {
    if (&other == this) {
      return *this;
    }

    // Free the memory of the existing response before moving.
    UA_WriteResponse_clear(&this->response);
    this->response = std::move(other.response);
    // Initialize the moved-from response to avoid use-after-move errors and
    // double frees.
    UA_WriteResponse_init(&other.response);
    return *this;
  }

  UA_WriteResponse response;
};

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_WRITE_RESPONSE_H_
