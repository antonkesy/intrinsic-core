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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_READ_RESPONSE_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_READ_RESPONSE_H_

#include <cstddef>

#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"

// This header contains RAII types for OPCUA types so that de-allocation of
// resources automatically happens in the destructor.
//
// Opcua types are exposed as public members to allow use of opcua functions.
// This means they shouldn't be overwritten without explicitly de-allocating the
// heap allocated memory.
namespace intrinsic::opcua {

struct ReadResponse {
  ReadResponse() { UA_ReadResponse_init(&response); }
  ~ReadResponse() { UA_ReadResponse_clear(&response); }

  ReadResponse(const ReadResponse& other) {
    UA_ReadResponse_copy(&other.response, &this->response);
  }

  ReadResponse(ReadResponse&& other) {
    UA_ReadResponse_copy(&other.response, &this->response);
  }

  ReadResponse& operator=(const ReadResponse& other) {
    UA_ReadResponse_copy(&other.response, &this->response);
    return *this;
  }

  ReadResponse& operator=(ReadResponse&& other) {
    UA_ReadResponse_copy(&other.response, &this->response);
    return *this;
  }

  size_t ResultsSize() const { return response.resultsSize; }

  template <class DataTypeT>
  const DataTypeT* Value() const {
    if (response.resultsSize == 0) {
      return nullptr;
    }
    return reinterpret_cast<DataTypeT*>(response.results->value.data);
  }

  UA_ReadResponse response;
};

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_READ_RESPONSE_H_
