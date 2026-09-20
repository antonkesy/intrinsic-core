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

#include "intrinsic/hardware/opcua/opcua_server.h"

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/log/log.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/util/thread/thread.h"
#include "open62541/common.h"
#include "open62541/nodeids.h"
#include "open62541/server.h"
#include "open62541/server_config_default.h"
#include "open62541/types.h"
#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"
#include "open62541/util.h"

namespace {

char* Language() {
  static constexpr char kLang[] = {"en-US"};
  return const_cast<char*>(kLang);
}

}  // namespace

namespace intrinsic::opcua {

OpcuaServer::OpcuaServer(int port) {
  // Creates a server that listens on `port`.
  server_ = UA_Server_new();
  UA_ServerConfig_setMinimal(UA_Server_getConfig(server_), port, nullptr);
}

OpcuaServer::~OpcuaServer() {
  StopServerAndWait();
  if (server_) {
    UA_Server_delete(server_);
  }

  for (auto* ptr : data_memory_) {
    free(ptr);
  }
}

uint16_t OpcuaServer::AddNamespace(absl::string_view namespace_name) {
  char* name = const_cast<char*>(GetManagedString(namespace_name));
  absl::MutexLock lock(server_mutex_);
  return UA_Server_addNamespace(server_, name);
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddObjectNode(
    uint32_t node_id, absl::string_view node_display_name, UA_UInt16 ns_index,
    const UA_NodeId& parent_node) {
  char* name = const_cast<char*>(GetManagedString(node_display_name));

  UA_ObjectAttributes attr = UA_ObjectAttributes_default;
  attr.displayName = UA_LOCALIZEDTEXT(Language(), name);
  attr.description = attr.displayName;
  UA_NodeId objectNodeId = UA_NODEID_NUMERIC(ns_index, node_id);
  UA_NodeId new_node_id;

  absl::MutexLock lock(server_mutex_);
  const auto status =
      UA_Server_addObjectNode(server_, objectNodeId, parent_node,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                              UA_QUALIFIEDNAME(ns_index, name),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                              attr, nullptr, &new_node_id);
  return {status, new_node_id};
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddObjectNode(
    absl::string_view node_name, UA_UInt16 ns_index,
    const UA_NodeId& parent_node) {
  char* name = const_cast<char*>(GetManagedString(node_name));

  UA_ObjectAttributes attr = UA_ObjectAttributes_default;
  attr.displayName = UA_LOCALIZEDTEXT(Language(), name);
  attr.description = attr.displayName;
  UA_NodeId objectNodeId = UA_NODEID_STRING(ns_index, name);
  UA_NodeId new_node_id;

  absl::MutexLock lock(server_mutex_);
  const auto status =
      UA_Server_addObjectNode(server_, objectNodeId, parent_node,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                              UA_QUALIFIEDNAME(ns_index, name),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                              attr, nullptr, &new_node_id);
  {
    absl::MutexLock lock(data_memory_mutex_);
    data_memory_.insert(new_node_id.identifier.string.data);
  }
  return {status, new_node_id};
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddBooleanVariableNode(
    const AddVariableNodeParams& params, const bool scalar_value) {
  // OPCUA data types store pointer to dynamically allocated data. To avoid
  // memory leaks, keep track of these pointers in a container to free them up
  // when this class object goes out of scope.
  UA_Boolean* data_ptr = UA_Boolean_new();
  {
    absl::MutexLock lock(data_memory_mutex_);
    data_memory_.insert(data_ptr);
  }
  *data_ptr = scalar_value;

  // Creates a variable node with the scalar value
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  UA_Variant_setScalar(&attr.value, data_ptr, &UA_TYPES[UA_TYPES_BOOLEAN]);
  return AddVariableNode(params, attr);
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddUInt16VariableNode(
    const AddVariableNodeParams& params, const uint16_t scalar_value) {
  // OPCUA data types store pointer to dynamically allocated data. To avoid
  // memory leaks, keep track of these pointers in a container to free them up
  // when this class object goes out of scope.
  UA_UInt16* data_ptr = UA_UInt16_new();
  {
    absl::MutexLock lock(data_memory_mutex_);
    data_memory_.insert(data_ptr);
  }
  *data_ptr = scalar_value;

  // Creates a variable node with the scalar value
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  UA_Variant_setScalar(&attr.value, data_ptr, &UA_TYPES[UA_TYPES_UINT16]);
  return AddVariableNode(params, attr);
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddInt32VariableNode(
    const AddVariableNodeParams& params, const int32_t scalar_value) {
  // OPCUA data types store pointer to dynamically allocated data. To avoid
  // memory leaks, keep track of these pointers in a container to free them up
  // when this class object goes out of scope.
  UA_Int32* data_ptr = UA_Int32_new();
  {
    absl::MutexLock lock(data_memory_mutex_);
    data_memory_.insert(data_ptr);
  }
  *data_ptr = scalar_value;

  // Creates a variable node with the scalar value
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  UA_Variant_setScalar(&attr.value, data_ptr, &UA_TYPES[UA_TYPES_INT32]);
  return AddVariableNode(params, attr);
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddUInt32VariableNode(
    const AddVariableNodeParams& params, const uint32_t scalar_value) {
  // OPCUA data types store pointer to dynamically allocated data. To avoid
  // memory leaks, keep track of these pointers in a container to free them up
  // when this class object goes out of scope.
  UA_UInt32* data_ptr = UA_UInt32_new();
  {
    absl::MutexLock lock(data_memory_mutex_);
    data_memory_.insert(data_ptr);
  }
  *data_ptr = scalar_value;

  // Creates a variable node with the scalar value
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  UA_Variant_setScalar(&attr.value, data_ptr, &UA_TYPES[UA_TYPES_UINT32]);
  return AddVariableNode(params, attr);
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddInt16VariableNode(
    const AddVariableNodeParams& params, const int16_t scalar_value) {
  // OPCUA data types store pointer to dynamically allocated data. To avoid
  // memory leaks, keep track of these pointers in a container to free them up
  // when this class object goes out of scope.
  UA_Int16* data_ptr = UA_Int16_new();
  {
    absl::MutexLock lock(data_memory_mutex_);
    data_memory_.insert(data_ptr);
  }
  *data_ptr = scalar_value;

  // Creates a variable node with the scalar value
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  UA_Variant_setScalar(&attr.value, data_ptr, &UA_TYPES[UA_TYPES_INT16]);
  return AddVariableNode(params, attr);
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddDoubleVariableNode(
    const AddVariableNodeParams& params, const double scalar_value) {
  // OPCUA data types store pointer to dynamically allocated data. To avoid
  // memory leaks, keep track of these pointers in a container to free them up
  // when this class object goes out of scope.
  UA_Double* data_ptr = UA_Double_new();
  {
    absl::MutexLock lock(data_memory_mutex_);
    data_memory_.insert(data_ptr);
  }
  *data_ptr = scalar_value;

  // Creates a variable node with the scalar value
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  UA_Variant_setScalar(&attr.value, data_ptr, &UA_TYPES[UA_TYPES_DOUBLE]);
  return AddVariableNode(params, attr);
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddStringVariableNode(
    const AddVariableNodeParams& params, absl::string_view string_value) {
  // OPCUA data types store pointer to dynamically allocated data. To avoid
  // memory leaks, keep track of these pointers in a container to free them up
  // when this class object goes out of scope.
  UA_String data_ptr = UA_String_fromChars(string_value.data());
  {
    absl::MutexLock lock(data_memory_mutex_);
    data_memory_.insert(data_ptr.data);
  }

  // Creates a variable node with the scalar value
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  attr.dataType = UA_TYPES[UA_TYPES_STRING].typeId;
  UA_Variant_setScalar(&attr.value, &data_ptr, &UA_TYPES[UA_TYPES_STRING]);
  return AddVariableNode(params, attr);
}

UA_StatusCode OpcuaServer::StartServerAsync() {
  // Startups the server if not already done
  {
    absl::MutexLock lock(server_mutex_);
    if (server_running_) {
      return UA_STATUSCODE_BADNOTHINGTODO;
    }
    if (const auto status_code = UA_Server_run_startup(server_);
        status_code != UA_STATUSCODE_GOOD) {
      return status_code;
    }
    server_running_ = true;
  }
  LOG(INFO) << "UA Server successfully started.";

  server_thread_ = intrinsic::Thread([this]() {
    // Runs the server's main loop
    while (true) {
      {
        const bool kWaitTillTimeout = true;
        absl::MutexLock lock(server_mutex_);
        if (this->server_requested_to_stop_) {
          break;
        }
        // Ignores the next wait time returned by this function since it causes
        // random timeouts in read calls made to this server. Even the blocking
        // `UA_Server_run()` (which internally calls iterate function) ignores
        // this return value.
        UA_Server_run_iterate(this->server_, kWaitTillTimeout);
      }
      absl::SleepFor(absl::Milliseconds(1));
    }

    // Shutdowns the server
    absl::MutexLock lock(server_mutex_);
    const auto shutdown_status = UA_Server_run_shutdown(this->server_);
    this->server_running_ = false;
    LOG(INFO) << "UA Server shutdown with status: " << shutdown_status;
  });

  return UA_STATUSCODE_GOOD;
}

void OpcuaServer::StopServerAndWait() {
  {
    absl::MutexLock lock(server_mutex_);
    if (!server_running_) {
      LOG(INFO) << "Nothing to stop: UA server not running.";
      return;
    }
    if (server_requested_to_stop_) {
      LOG(WARNING) << "Request to stop the server is already being handled.";
      return;
    }
    server_requested_to_stop_ = true;
  }
  server_thread_.join();
}

int OpcuaServer::ActiveSessionCount() const {
  absl::MutexLock lock(server_mutex_);
  auto ua_server_statistics = UA_Server_getStatistics(server_);
  return ua_server_statistics.ss.currentSessionCount;
}

const char* OpcuaServer::GetManagedString(absl::string_view str) {
  // Opcua only stores a pointer to the underlying string, so we need to
  // manage the strings ourselves to avoid leaks.
  auto iter = ua_strings_.insert(std::string(str));
  return iter.first->c_str();
}

absl::Status OpcuaServer::CallServerHandle(
    std::function<absl::Status(UA_Server*)> function) {
  absl::MutexLock lock(server_mutex_);
  return function(server_);
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddVariableNode(
    const AddVariableNodeParams& params, UA_VariableAttributes& attr) {
  UA_NodeId node_id;
  if (std::holds_alternative<uint32_t>(params.node_id)) {
    node_id =
        UA_NODEID_NUMERIC(params.ns_index, std::get<uint32_t>(params.node_id));
  } else if (std::holds_alternative<std::string>(params.node_id)) {
    char* name = const_cast<char*>(
        GetManagedString(std::get<std::string>(params.node_id)));
    node_id = UA_NODEID_STRING(params.ns_index, name);
  } else {
    return {UA_STATUSCODE_BADINVALIDARGUMENT, UA_NODEID_NUMERIC(0, 0)};
  }

  char* display = const_cast<char*>(GetManagedString(params.display_name));

  attr.displayName = UA_LOCALIZEDTEXT(Language(), display);
  attr.description = attr.displayName;
  attr.accessLevel = params.access_level_mask;
  UA_QualifiedName browse_name = UA_QUALIFIEDNAME(1, display);
  UA_NodeId parent_relation = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);

  // Adds the node.
  absl::MutexLock lock(server_mutex_);
  const auto status = UA_Server_addVariableNode(
      server_, node_id, params.parent_node, parent_relation, browse_name,
      UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, nullptr,
      nullptr);
  return {status, node_id};
}

std::pair<UA_StatusCode, UA_NodeId> OpcuaServer::AddArrayVariableNode(
    const AddVariableNodeParams& params,
    const std::vector<std::string>& array_value) {
  std::vector<const char*> ua_string_vector;
  ua_string_vector.reserve(array_value.size());
  for (const auto& s : array_value) {
    ua_string_vector.push_back(s.data());
  }

  return AddArrayVariableNode(params, ua_string_vector);
}
}  // namespace intrinsic::opcua
