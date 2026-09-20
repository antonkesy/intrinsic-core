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

#include "intrinsic/hardware/opcua/opcua_write_request.h"

#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <string>
#include <type_traits>
#include <vector>

#include "open62541/common.h"
#include "open62541/types.h"
#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"

namespace intrinsic::opcua {

namespace {

UA_WriteRequest InitializeWriteRequest(size_t num_nodes) {
  UA_WriteRequest request;
  UA_WriteRequest_init(&request);
  request.nodesToWrite = static_cast<UA_WriteValue*>(
      UA_Array_new(num_nodes, &UA_TYPES[UA_TYPES_WRITEVALUE]));
  request.nodesToWriteSize = num_nodes;

  return request;
}

template <class ForwardIterator>
UA_WriteRequest InitializeWriteRequest(ForwardIterator begin,
                                       ForwardIterator end) {
  static_assert(
      std::is_same<typename std::iterator_traits<ForwardIterator>::value_type,
                   UA_NodeId>::value);

  size_t num_nodes = std::distance(begin, end);
  auto request = InitializeWriteRequest(num_nodes);

  for (size_t i = 0; begin < end; ++i, ++begin) {
    UA_WriteValue_init(&request.nodesToWrite[i]);
    request.nodesToWrite[i].attributeId = UA_ATTRIBUTEID_VALUE;
    // If the given node id has a valid string identifier, then the request for
    // would copy those pointers as well.
    UA_NodeId_copy(&*begin, &request.nodesToWrite[i].nodeId);
    UA_DataValue_init(&request.nodesToWrite[i].value);
    request.nodesToWrite[i].value.hasValue = false;
  }
  return request;
}

}  // namespace

WriteRequest::WriteRequest(const char* node_id) noexcept {
  UA_NodeId node_id_obj = UA_NODEID(node_id);
  std::initializer_list<UA_NodeId> vec = {node_id_obj};
  request = InitializeWriteRequest(vec.begin(), vec.end());
  UA_NodeId_clear(&node_id_obj);
}

// Initializes the request using given nodes.
// WARNING: The request stores pointer to the nodes' string identifiers, so
// the lifetime of the given nodes' string identifiers should exceed that of
// the request.
WriteRequest::WriteRequest(const std::initializer_list<UA_NodeId>& nodes) {
  request = InitializeWriteRequest(nodes.begin(), nodes.end());
}

WriteRequest::WriteRequest(const std::vector<UA_NodeId>& nodes) {
  request = InitializeWriteRequest(nodes.begin(), nodes.end());
}

WriteRequest::WriteRequest(const std::vector<std::string>& nodes) noexcept {
  std::vector<UA_NodeId> node_array(nodes.size());
  std::transform(nodes.cbegin(), nodes.cend(), node_array.begin(),
                 [](const std::string& node) -> UA_NodeId {
                   return UA_NODEID(node.c_str());
                 });
  request = InitializeWriteRequest(node_array.begin(), node_array.end());
}

}  // namespace intrinsic::opcua
