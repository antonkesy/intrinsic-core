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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_CLIENT_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_CLIENT_H_

#include <functional>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/hardware/opcua/opcua_read_request.h"
#include "intrinsic/hardware/opcua/opcua_read_response.h"
#include "intrinsic/hardware/opcua/opcua_write_request.h"
#include "intrinsic/hardware/opcua/opcua_write_response.h"
#include "open62541/client.h"
#include "open62541/common.h"
#include "open62541/types.h"
#include "open62541/types_generated.h"

namespace intrinsic::opcua {

// Default timeout to wait for responses for each opcua request.
constexpr absl::Duration kResponseTimeout = absl::Seconds(50);
// Default interval to check connectivity with the server.
constexpr absl::Duration kConnectivityCheckInterval = absl::Seconds(60);

// Convenience opcua client that supports re-connection with the opcua server if
// the connection drops due to unknown reasons.
// None of the methods are thread-safe.
class OpcuaClient {
 public:
  using StateCallbackType = std::function<void(
      UA_Client* client, UA_SecureChannelState channelState,
      UA_SessionState sessionState, UA_StatusCode connectStatus)>;

  // Indicates the desired behavior when the connection with opcua server drops
  // unexpectedly.
  enum class OnConnectionError {
    // Try re-connecting with the opcua server if the connection closed.
    kTryReconnect,
    // Return the error and don't attempt to re-connect with the opcua server.
    kReturnError,
  };

  struct Authentication {
    std::string username;
    std::string password;

    // If any of these are non-empty, then encryption will be used for the
    // communication with the server.
    std::string cert_file;
    std::string private_key_file;
    std::string application_uri;

    // Note: this field is temporarily ignored until a segfault is fixed in the
    // third_party code.
    std::vector<std::string> trusted_certificates_filepaths;
  };

  // Creates an instance of opcua client but does not initiate connection with
  // the opcua server. `response_timeout` is the timeout for each opcua request.
  // `connectivity_check_interval` is the interval at which the client checks
  // connectivivty with the server. 0 disables the connectivity
  // check.
  explicit OpcuaClient(
      absl::string_view opcua_server_address, const Authentication& auth,
      absl::Duration response_timeout = kResponseTimeout,
      absl::Duration connectivity_check_interval = kConnectivityCheckInterval);

  OpcuaClient(OpcuaClient&& other) noexcept;

  // Closes connection with the opcua server if active.
  ~OpcuaClient();

  // Initiates connection with the opcua server.
  absl::Status Connect();
  // Closes connection with the opcua server. Does not delete the client.
  // Connection can be re-established by calling `Connect()`.
  absl::Status Disconnect();

  // Returns whether the opcua client has a healthy connection to the server.
  bool IsConnected() const;

  // Tries to keep the connection to the opcua server in a healthy state and
  // reconnects if needed. If re-connection is attempted, this method may block
  // for many seconds.
  absl::Status KeepConnectionHealthyOrReconnect();

  // Returns the response from the opcua server for the given write request.
  // Supports re-connecting with the opcua server.
  ::intrinsic::opcua::WriteResponse Write(
      const ::intrinsic::opcua::WriteRequest& request,
      OnConnectionError on_connection_error);

  // Returns the response from the opcua server for the given read request.
  // Supports re-connecting with the opcua server.
  ::intrinsic::opcua::ReadResponse Read(
      const ::intrinsic::opcua::ReadRequest& request,
      OnConnectionError on_connection_error);
  // Returns the response from the opcua server for the given read request.
  // Supports re-connecting with the opcua server.
  ABSL_DEPRECATED("Use Read(const ::intrinsic::opcua::ReadRequest&) instead.")
  ::intrinsic::opcua::ReadResponse Read(const UA_ReadRequest& request,
                                        OnConnectionError on_connection_error);

  // Returns the pointer to the underlying UA_Client.
  UA_Client* Handle() { return client_; }

  // Returns the address of the opcua server.
  std::string OpcuaServerAddress() const { return opcua_server_address_; }

  // Sets the state callback to be called when the state of the connection
  // changes. Thread-safe.
  void SetStateCallback(StateCallbackType state_callback);

 private:
  // Callback function given to the UA_ClientConfig for the state of the
  // connection.
  static void OpcuaStateCallback(UA_Client* client,
                                 UA_SecureChannelState channelState,
                                 UA_SessionState sessionState,
                                 UA_StatusCode connectStatus);
  // Performs the one time initialization for the client_. If successful it will
  // set initialized_client_config_ to ensure it will only be called once.
  //
  // TODO(b/290409083): Add unittests to cover the encryption case.
  absl::Status MaybeInitializeClientConfig();
  bool initialized_client_config_ = false;

  std::string opcua_server_address_;

  UA_Client* client_;

  // TODO(dhirajgoel): Allow the user to configure the client (e.g. set
  // timeouts).
  UA_ClientConfig* config_;

  const Authentication auth_;
  // Timeout for each opcua request.
  const absl::Duration response_timeout_;
  const absl::Duration connectivity_check_interval_;
  absl::Mutex state_callback_mutex_;
  // Callback to be called when the state of the connection changes.
  StateCallbackType state_callback_ ABSL_GUARDED_BY(state_callback_mutex_);
};

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_CLIENT_H_
