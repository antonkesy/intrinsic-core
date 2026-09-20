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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_SERVER_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_SERVER_H_

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/container/node_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/hardware/opcua/opcua_arrays.h"
#include "intrinsic/hardware/opcua/opcua_type_traits.h"
#include "intrinsic/util/thread/thread.h"
#include "open62541/common.h"
#include "open62541/nodeids.h"
#include "open62541/types.h"
#include "open62541/types_generated_handling.h"
#include "open62541/util.h"

namespace intrinsic::opcua {

// A very simple opcua server that provides APIs to add scalar values (bool for
// now). Unlike open62541 library, this class actually manages its memory (i.e.
// keeps track of heap allocated data and de-allocates it on
// destruction) as well as provides thread safe APIs. On the other hand,
// open62541 data types store raw pointers to heap allocated memory and requires
// explicit de-allocation to prevent memory leaks.
//
// The main goal of this class it to write end-to-end integration tests for
// users of opcua gpio service (e.g. gripper). It is not meant to be deployed
// in production.
class OpcuaServer {
 public:
  static constexpr int kDefaultPort = 4840;

  struct AddVariableNodeParams {
    // The node id can be either a numeric id or a string id.
    std::variant<uint32_t, std::string> node_id;
    // The display name of the node.
    absl::string_view display_name;
    // The namespace index of the node. Namespaces start from 0.
    UA_UInt16 ns_index;
    // The parent node of the node.
    const UA_NodeId& parent_node;
    // The access level mask of the node.  More values are defined here:
    // https://www.open62541.org/doc/0.2/constants.html#access-level-masks
    uint8_t access_level_mask =
        (UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE);
  };

  // Starts the opcua server on `port`
  explicit OpcuaServer(int port = kDefaultPort);

  // Stops the server if already running
  ~OpcuaServer();

  // Returns the index of the added namespace.
  uint16_t AddNamespace(absl::string_view namespace_name);

  // Returns status code and id for the added node. The id is only valid
  // if status code is UA_STATUSCODE_GOOD.
  std::pair<UA_StatusCode, UA_NodeId> AddObjectNode(
      absl::string_view node_name, UA_UInt16 ns_index = 0,
      const UA_NodeId& parent_node = UA_NODEID_NUMERIC(0,
                                                       UA_NS0ID_OBJECTSFOLDER));
  std::pair<UA_StatusCode, UA_NodeId> AddObjectNode(
      uint32_t node_id, absl::string_view node_display_name,
      UA_UInt16 ns_index = 0,
      const UA_NodeId& parent_node = UA_NODEID_NUMERIC(0,
                                                       UA_NS0ID_OBJECTSFOLDER));

  // Add a boolean variable node. `scalar_value` is the initial value of the
  // variable.
  //
  // Returns status code and id for the added node in the given namespace. The
  // id is only valid if status code is UA_STATUSCODE_GOOD.
  std::pair<UA_StatusCode, UA_NodeId> AddBooleanVariableNode(
      const AddVariableNodeParams& params, bool scalar_value);

  // Add a int16 variable node. `scalar_value` is the initial value of the
  // variable.
  //
  // Returns status code and id for the added node in the given namespace. The
  // id is only valid if status code is UA_STATUSCODE_GOOD.
  std::pair<UA_StatusCode, UA_NodeId> AddInt16VariableNode(
      const AddVariableNodeParams& params, int16_t scalar_value);

  // Add a uint16 variable node. `scalar_value` is the initial value of the
  // variable.
  //
  // Returns status code and id for the added node in the given namespace. The
  // id is only valid if status code is UA_STATUSCODE_GOOD.
  std::pair<UA_StatusCode, UA_NodeId> AddUInt16VariableNode(
      const AddVariableNodeParams& params, uint16_t scalar_value);

  // Add a int32 variable node. `scalar_value` is the initial value of the
  // variable.
  //
  // Returns status code and id for the added node in the given namespace. The
  // id is only valid if status code is UA_STATUSCODE_GOOD.
  std::pair<UA_StatusCode, UA_NodeId> AddInt32VariableNode(
      const AddVariableNodeParams& params, int32_t scalar_value);

  // Add a uint32 variable node. `scalar_value` is the initial value of the
  // variable.
  //
  // Returns status code and id for the added node in the given namespace. The
  // id is only valid if status code is UA_STATUSCODE_GOOD.
  std::pair<UA_StatusCode, UA_NodeId> AddUInt32VariableNode(
      const AddVariableNodeParams& params, uint32_t scalar_value);

  // Add a double variable node. `scalar_value` is the initial value of the
  // variable.
  //
  // Returns status code and id for the added node in the given namespace. The
  // id is only valid if status code is UA_STATUSCODE_GOOD.
  std::pair<UA_StatusCode, UA_NodeId> AddDoubleVariableNode(
      const AddVariableNodeParams& params, double scalar_value);
  // Add a string variable node. `string_value` is the initial value of the
  // variable.
  //
  // Returns status code and id for the added node in the given namespace. The
  // id is only valid if status code is UA_STATUSCODE_GOOD.
  std::pair<UA_StatusCode, UA_NodeId> AddStringVariableNode(
      const AddVariableNodeParams& params, absl::string_view string_value);

  // Add an array variable node. `array_value` is the initial value of the
  // variable.
  // Returns status code and id for the array variable node in given namespace.
  // The id is only valid if status code is UA_STATUSCODE_GOOD.
  template <typename T>
  std::pair<UA_StatusCode, UA_NodeId> AddArrayVariableNode(
      const AddVariableNodeParams& params, const std::vector<T>& array_value);
  // Convenience function for std::vector<std::string> instead of const char*.
  std::pair<UA_StatusCode, UA_NodeId> AddArrayVariableNode(
      const AddVariableNodeParams& params,
      const std::vector<std::string>& array_value);

  // Starts the opcua main loop in a separate thread to handle requests
  UA_StatusCode StartServerAsync();

  // Blocks till the server has stopped
  void StopServerAndWait();

  // Returns the currently active sessions count.
  int ActiveSessionCount() const;

  // Lets the user interact thread-safely with the server.
  // While `function` is running, the server is guaranteed to not be stopped if
  // it is already started, but the server is also not iterating.
  // For example:
  //   ASSERT_OK(opcua_server.Interact([](UA_Server *server) -> absl::Status {
  //     if (const auto status_code = UA_Server_run_startup(server);
  //         status_code != UA_STATUSCODE_GOOD) {
  //       return absl::InternalError(UA_StatusCode_name(status_code));
  //     }
  //     return absl::OkStatus();
  //   }));
  absl::Status CallServerHandle(
      std::function<absl::Status(UA_Server*)> function);

 private:
  // Returns a pointer to an internally managed string
  const char* GetManagedString(absl::string_view str);

  std::pair<UA_StatusCode, UA_NodeId> AddVariableNode(
      const AddVariableNodeParams& params, UA_VariableAttributes& attr);

  UA_Server* server_ ABSL_GUARDED_BY(server_mutex_) = nullptr;

  // Keep track of whether server is running or not
  bool server_running_ ABSL_GUARDED_BY(server_mutex_) = false;

  // Request to stop the server
  bool server_requested_to_stop_ ABSL_GUARDED_BY(server_mutex_) = false;

  // Thread to run UA server on
  ::intrinsic::Thread server_thread_;

  // Mutex to protect all operations that access `server_` including its current
  // state as well request to stop it.
  mutable absl::Mutex server_mutex_;

  // Containers to keep track of all the heap allocated opcua scalar values used
  // by the server. Currently, this gets de-allocated in the destructor and not
  // before that. `Set` was chosen to easily remove a pointer that may have
  // already been free'd up (though this functionality is not used right now).
  absl::flat_hash_set<void*> data_memory_ ABSL_GUARDED_BY(data_memory_mutex_);
  // Mutex to protect `data_memory_`.
  mutable absl::Mutex data_memory_mutex_;

  // Container to keep track of all the string allocations. `node_hash_set` is
  // used to guarantee pointer stability for the strings (including samll
  // strings that may not incur heap allocation) since these strings are
  // referred by open62541's data structures using pointers.
  absl::node_hash_set<std::string> ua_strings_;
};

template <typename T>
std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddArrayVariableNode(
    const AddVariableNodeParams& params, const std::vector<T>& array_value) {
  UA_VariableAttributes attr = UA_VariableAttributes_default;

  UA_UInt32* dims = UA_UInt32_new();
  // Cleanup the dims pointer in all code paths, especially in
  // case of error and later code changes.
  auto cleanup_dims = absl::MakeCleanup([dims] { UA_UInt32_delete(dims); });
  *dims = array_value.size();

  const UA_DataType* dataType = CToOpcua<T>::value;
  auto [array_ptr, array_size] = ToOpcuaArray<T>(array_value);
  if (array_ptr == nullptr) {
    return std::make_pair(UA_STATUSCODE_BAD, UA_NODEID_NUMERIC(0, 0));
  }
  auto cleanup_array =
      absl::MakeCleanup([&array_ptr, size = array_value.size(), dataType] {
        if (array_ptr) {
          UA_Array_delete(array_ptr, size, dataType);
        }
      });
  UA_Variant_setArray(&attr.value, array_ptr, array_value.size(), dataType);
  attr.value.arrayDimensions = dims;
  attr.value.arrayDimensionsSize = 1;

  return AddVariableNode(params, attr);
}

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_SERVER_H_
