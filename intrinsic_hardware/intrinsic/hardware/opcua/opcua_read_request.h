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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_READ_REQUEST_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_READ_REQUEST_H_

#include <cstddef>
#include <initializer_list>
#include <string>
#include <vector>

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

struct ReadRequest {
  ReadRequest() { UA_ReadRequest_init(&request); }

  // Initializes the request using given nodes.
  // WARNING: The request stores pointer to the nodes' string identifiers, so
  // the lifetime of the given nodes' string identifiers should exceed that of
  // the request.
  explicit ReadRequest(const char* node_id) noexcept;

  explicit ReadRequest(size_t num_nodes);

  // Initializes the request using given nodes.
  // WARNING: The request stores pointer to the nodes' string identifiers, so
  // the lifetime of the given nodes' string identifiers should exceed that of
  // the request.
  ReadRequest(const std::initializer_list<UA_NodeId>& nodes);

  // Initializes the request using given nodes.
  // WARNING: The request stores pointer to the nodes' string identifiers, so
  // the lifetime of the given nodes' string identifiers should exceed that of
  // the request.
  explicit ReadRequest(const std::vector<std::string>& nodes) noexcept;

  ~ReadRequest() { UA_ReadRequest_clear(&request); }

  ReadRequest(const ReadRequest& other) {
    UA_ReadRequest_copy(&other.request, &this->request);
  }

  ReadRequest(ReadRequest&& other) {
    UA_ReadRequest_copy(&other.request, &this->request);
  }

  ReadRequest& operator=(const ReadRequest& other) {
    UA_ReadRequest_copy(&other.request, &this->request);
    return *this;
  }

  ReadRequest& operator=(ReadRequest&& other) {
    UA_ReadRequest_copy(&other.request, &this->request);
    return *this;
  }

  UA_ReadRequest request;
};

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_READ_REQUEST_H_
