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

#include "intrinsic/hardware/opcua/opcua_read_request.h"

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

UA_ReadRequest InitializeReadRequest(size_t num_nodes) {
  UA_ReadRequest request;
  UA_ReadRequest_init(&request);
  request.nodesToRead = static_cast<UA_ReadValueId*>(
      UA_Array_new(num_nodes, &UA_TYPES[UA_TYPES_READVALUEID]));
  request.nodesToReadSize = num_nodes;

  return request;
}

template <class ForwardIterator>
UA_ReadRequest InitializeReadRequest(ForwardIterator begin,
                                     ForwardIterator end) {
  static_assert(
      std::is_same<typename std::iterator_traits<ForwardIterator>::value_type,
                   UA_NodeId>::value);

  size_t num_nodes = std::distance(begin, end);
  auto request = InitializeReadRequest(num_nodes);

  for (size_t i = 0; begin < end; ++i, ++begin) {
    UA_ReadValueId_init(&request.nodesToRead[i]);
    request.nodesToRead[i].attributeId = UA_ATTRIBUTEID_VALUE;
    // If the given node id has a valid string identifier, then the request for
    // would copy those pointers as well.
    UA_NodeId_copy(&*begin, &request.nodesToRead[i].nodeId);
  }
  return request;
}

}  // namespace

ReadRequest::ReadRequest(const char* node_id) noexcept {
  UA_NodeId node_id_obj = UA_NODEID(node_id);
  std::initializer_list<UA_NodeId> vec = {node_id_obj};
  request = InitializeReadRequest(vec.begin(), vec.end());
  UA_NodeId_clear(&node_id_obj);
}

ReadRequest::ReadRequest(size_t num_nodes) {
  request = InitializeReadRequest(num_nodes);
}

ReadRequest::ReadRequest(const std::initializer_list<UA_NodeId>& nodes) {
  request = InitializeReadRequest(nodes.begin(), nodes.end());
}

ReadRequest::ReadRequest(const std::vector<std::string>& nodes) noexcept {
  std::vector<UA_NodeId> node_array(nodes.size());
  std::transform(nodes.cbegin(), nodes.cend(), node_array.begin(),
                 [](const std::string& node) -> UA_NodeId {
                   return UA_NODEID(node.c_str());
                 });

  request = InitializeReadRequest(node_array.begin(), node_array.end());
}

}  // namespace intrinsic::opcua
