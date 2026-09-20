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

#include "intrinsic/hardware/opcua/opcua_client.h"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/hardware/opcua/opcua_read_request.h"
#include "intrinsic/hardware/opcua/opcua_read_response.h"
#include "intrinsic/hardware/opcua/opcua_write_request.h"
#include "intrinsic/hardware/opcua/opcua_write_response.h"
#include "intrinsic/util/status/status_macros.h"
#include "open62541/client.h"
#include "open62541/client_config_default.h"
#include "open62541/common.h"
#include "open62541/types.h"
#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"

namespace intrinsic::opcua {

namespace {

// Pulled from examples code in open62541 repo.
// TODO(keegang): clean this up by returning a std::vector<uint_8> and using
// fstream rather than fopen.
absl::StatusOr<UA_ByteString> LoadFile(const char* const path) {
  UA_ByteString fileContents = UA_STRING_NULL;

  // Open the file
  FILE* fp = fopen(path, "rb");
  if (!fp) {
    LOG(ERROR) << "Failed to read file: " << path;
    errno = 0;
    return fileContents;
  }

  absl::Cleanup fp_closer = [fp] { fclose(fp); };

  // Get the file length, allocate the data and read.
  fseek(fp, 0, SEEK_END);
  fileContents.length = (size_t)ftell(fp);
  fileContents.data =
      (UA_Byte*)UA_malloc(fileContents.length * sizeof(UA_Byte));
  if (fileContents.data) {
    fseek(fp, 0, SEEK_SET);
    size_t read =
        fread(fileContents.data, sizeof(UA_Byte), fileContents.length, fp);
    if (read != fileContents.length) {
      UA_ByteString_clear(&fileContents);
    }
  } else {
    fileContents.length = 0;
  }

  return fileContents;
}

}  // namespace

OpcuaClient::OpcuaClient(const absl::string_view opcua_server_address,
                         const Authentication& auth,
                         absl::Duration response_timeout,
                         absl::Duration connectivity_check_interval)
    : opcua_server_address_(opcua_server_address),
      auth_(auth),
      response_timeout_(response_timeout),
      connectivity_check_interval_(connectivity_check_interval) {
  client_ = UA_Client_new();
  config_ = UA_Client_getConfig(client_);
}

OpcuaClient::~OpcuaClient() {
  if (client_) {
    UA_Client_disconnect(client_);
    // Disconnects the client from the server.
    UA_Client_delete(client_);
  }
}

OpcuaClient::OpcuaClient(OpcuaClient&& other) noexcept
    : initialized_client_config_(
          std::exchange(other.initialized_client_config_, false)),
      opcua_server_address_(std::exchange(other.opcua_server_address_, "")),
      client_(std::exchange(other.client_, nullptr)),
      config_(std::exchange(other.config_, nullptr)),
      auth_(std::move(other.auth_)),
      response_timeout_(std::move(other.response_timeout_)),
      state_callback_(std::move(other.state_callback_)) {
  config_->clientContext = this;
}

absl::Status OpcuaClient::Connect() {
  UA_StatusCode status = UA_STATUSCODE_BAD;

  INTR_RETURN_IF_ERROR(MaybeInitializeClientConfig());

  // Checks if authentication information is valid.
  auto auth_is_valid = [](const absl::string_view username,
                          const absl::string_view password) -> absl::Status {
    if (!username.empty() && !password.empty()) {
      return absl::OkStatus();
    }

    if (username.empty() && password.empty()) {
      return absl::InvalidArgumentError(
          "Expected non-empty username and password to authenticate with the "
          "opcua server.");
    }

    if (username.empty()) {
      return absl::InvalidArgumentError(
          "Expected non-empty username to authenticate with the opcua "
          "server. ");
    }
    if (password.empty()) {
      return absl::InvalidArgumentError(
          "Expected non-empty password to authenticate with the opcua server.");
    }
    return absl::OkStatus();
  };

  if (!auth_.username.empty() || !auth_.password.empty()) {
    INTR_RETURN_IF_ERROR(auth_is_valid(auth_.username, auth_.password));
    LOG(INFO) << "Connecting with username and password.";
    status = UA_Client_connectUsername(client_, opcua_server_address_.c_str(),
                                       auth_.username.c_str(),
                                       auth_.password.c_str());
  } else {
    LOG(INFO) << "Connecting anonymously to " << opcua_server_address_;
    status = UA_Client_connect(client_, opcua_server_address_.c_str());
  }

  if (status != UA_STATUSCODE_GOOD) {
    return absl::InternalError(
        absl::StrCat("Connection error: ", UA_StatusCode_name(status)));
  }

  return absl::OkStatus();
}

absl::Status OpcuaClient::Disconnect() {
  auto status = UA_Client_disconnect(client_);
  if (status != UA_STATUSCODE_GOOD) {
    return absl::InternalError(
        absl::StrCat("Error while disconnecting from opcua server: ",
                     UA_StatusCode_name(status)));
  }
  return absl::OkStatus();
}

// Returns if the connection to the opcua server is healthy as of the last call
// made to the server. To get the most up-to-date status, a read, write or
// iterate call should be made prior to calling this function.
static bool IsConnectionToServerHealthy(UA_Client* client) {
  // For a healthy connection, we need channel to be OPEN, session to be
  // ACTIVATED and connection status to be GOOD.
  UA_SecureChannelState channel_state;
  UA_SessionState session_state;
  UA_StatusCode connect_status;
  UA_Client_getState(client, &channel_state, &session_state, &connect_status);
  if (channel_state == UA_SECURECHANNELSTATE_OPEN &&
      session_state == UA_SESSIONSTATE_ACTIVATED &&
      connect_status == UA_STATUSCODE_GOOD) {
    return true;
  }

  return false;
}

bool OpcuaClient::IsConnected() const {
  return IsConnectionToServerHealthy(client_);
}

absl::Status OpcuaClient::KeepConnectionHealthyOrReconnect() {
  // Timeout to wait for a new message to arrive on the network socket. We keep
  // this very low to avoid blocking other requests.
  constexpr uint32_t kTimeoutMs = 1;

  // Runs a single iteration so that internal state is updated. The return value
  // is ignored because it is not sufficient to catch connection errors.
  UA_Client_run_iterate(client_, kTimeoutMs);

  if (IsConnectionToServerHealthy(client_)) {
    return absl::OkStatus();
  }

  // Connection is unhealthy. Try to establish the connection again, which
  // internally calls `run_iterate` till the session has been activated. This
  // can block till `kConnectionTimeout`.
  // https://github.com/open62541/open62541/blob/v1.3.5/examples/client_connect_loop.c#L44-L47
  return Connect();
}

::intrinsic::opcua::WriteResponse OpcuaClient::Write(
    const ::intrinsic::opcua::WriteRequest& request,
    const OnConnectionError on_connection_error) {
  ::intrinsic::opcua::WriteResponse response;
  response.response = UA_Client_Service_write(client_, request.request);
  if (response.response.responseHeader.serviceResult == UA_STATUSCODE_GOOD) {
    return response;
  }

  if ((on_connection_error != OnConnectionError::kTryReconnect) ||
      IsConnectionToServerHealthy(client_)) {
    return response;
  }

  if (const auto status = Connect(); !status.ok()) {
    return response;
  }

  response.response = UA_Client_Service_write(client_, request.request);
  return response;
}

::intrinsic::opcua::ReadResponse OpcuaClient::Read(
    const ReadRequest& request, const OnConnectionError on_connection_error) {
  ::intrinsic::opcua::ReadResponse response;
  response.response = UA_Client_Service_read(client_, request.request);
  if (response.response.responseHeader.serviceResult == UA_STATUSCODE_GOOD) {
    return response;
  }

  if ((on_connection_error != OnConnectionError::kTryReconnect) ||
      IsConnectionToServerHealthy(client_)) {
    return response;
  }

  if (const auto status = Connect(); !status.ok()) {
    return response;
  }

  response.response = UA_Client_Service_read(client_, request.request);
  return response;
}

::intrinsic::opcua::ReadResponse OpcuaClient::Read(
    const UA_ReadRequest& request,
    const OnConnectionError on_connection_error) {
  ::intrinsic::opcua::ReadRequest req;
  UA_ReadRequest_copy(&request, &req.request);
  return Read(req, on_connection_error);
}

void OpcuaClient::OpcuaStateCallback(UA_Client* client,
                                     UA_SecureChannelState channelState,
                                     UA_SessionState sessionState,
                                     UA_StatusCode connectStatus) {
  UA_ClientConfig* config = UA_Client_getConfig(client);
  if (config->clientContext == nullptr) {
    return;
  }
  OpcuaClient* opcua_client = static_cast<OpcuaClient*>(config->clientContext);
  absl::MutexLock lock(opcua_client->state_callback_mutex_);
  if (opcua_client->state_callback_) {
    opcua_client->state_callback_(client, channelState, sessionState,
                                  connectStatus);
  }
}

absl::Status OpcuaClient::MaybeInitializeClientConfig() {
  if (initialized_client_config_) {
    return absl::OkStatus();
  }

  UA_StatusCode status = UA_STATUSCODE_BAD;

  if (!auth_.cert_file.empty() || !auth_.private_key_file.empty() ||
      !auth_.application_uri.empty() ||
      !auth_.trusted_certificates_filepaths.empty()) {
    // Load certificate and private key.
    INTR_ASSIGN_OR_RETURN(UA_ByteString certificate,
                          LoadFile(auth_.cert_file.c_str()));
    INTR_ASSIGN_OR_RETURN(UA_ByteString private_key,
                          LoadFile(auth_.private_key_file.c_str()));

    // Setup the trust list and revocation list. Both are temporarily disabled
    // for now.
    constexpr size_t trust_list_size = 0;
    UA_ByteString* trust_list = nullptr;
    // TODO(keegang): Figure out how to set up a variable size array.
    // constexpr size_t trust_list_size = 1;
    // UA_STACKARRAY(UA_ByteString, trust_list, trust_list_size + 1);
    // INTR_ASSIGN_OR_RETURN(
    //     trust_list[0],
    //     LoadFile(auth_.trusted_certificates_filepaths.front().c_str()));

    UA_ByteString* revocation_list = nullptr;
    size_t revocation_list_size = 0;

    config_->securityMode = UA_MESSAGESECURITYMODE_SIGNANDENCRYPT;
    UA_StatusCode retval = UA_ClientConfig_setDefaultEncryption(
        config_, certificate, private_key, trust_list, trust_list_size,
        revocation_list, revocation_list_size);
    if (retval != UA_STATUSCODE_GOOD) {
      UA_ByteString_clear(&certificate);
      UA_ByteString_clear(&private_key);
      return absl::InternalError(absl::StrCat(
          "Setting UA client config (with encryption) failed with error: ",
          UA_StatusCode_name(status)));
    }

    // ApplicationUri MUST match with the "urn:..." name in the
    // "X509v3 Subject Alternative Name" section of the client cert (including
    // the "urn:" prefix, but not the "URI:" prefix in front of that).
    //
    // You can see the names in the cert with something like:
    //   openssl x509 -in server_cert.der -inform der -text
    //
    // This MUST go after UA_ClientConfig_setDefaultEncryption since that call
    // overrides clientDescription.applicationUri with a default value.
    //
    // The other names (dns, ip address) do not seem to matter. You can transfer
    // the cert and private key to a different machine and it appears to work
    // fine.
    config_->clientDescription.applicationUri =
        UA_STRING_ALLOC(auth_.application_uri.c_str());

    UA_ByteString_clear(&certificate);
    UA_ByteString_clear(&private_key);
    for (size_t deleteCount = 0; deleteCount < trust_list_size; deleteCount++) {
      UA_ByteString_clear(&trust_list[deleteCount]);
    }
  } else {
    if (const auto status = UA_ClientConfig_setDefault(config_);
        status != UA_STATUSCODE_GOOD) {
      return absl::InternalError(absl::StrCat(
          "Setting UA client config (no encryption) failed with error: ",
          UA_StatusCode_name(status)));
    }
  }

  // The following parameters are needed to ensure stability of long running
  // connections. We do retry the connection on failures, but without these
  // settings the number of retries goes up quite a bit.

  // Sets connection timeout in milliseconds.
  config_->timeout = absl::ToInt64Milliseconds(response_timeout_);

  // Sets session and channel timeouts (in milliseconds) to the largest
  // possible values.
  constexpr auto kSessionTimeout = std::numeric_limits<uint32_t>::max();
  constexpr auto kSecureChannelLifetime = std::numeric_limits<uint32_t>::max();
  config_->requestedSessionTimeout = kSessionTimeout;
  config_->secureChannelLifeTime = kSecureChannelLifetime;

  // Sets interval (in milliseconds) to check connectivity with the server. The
  // default value is zero (which disables the connectivity check).
  config_->connectivityCheckInterval =
      absl::ToInt64Milliseconds(connectivity_check_interval_);
  // Sets the client context to this object.
  // This is used to access this class in the state callback.
  config_->clientContext = this;
  // Called when the state of the connection changes.
  config_->stateCallback = &OpcuaClient::OpcuaStateCallback;

  initialized_client_config_ = true;
  return absl::OkStatus();
}

void OpcuaClient::SetStateCallback(
    OpcuaClient::StateCallbackType state_callback) {
  absl::MutexLock lock(state_callback_mutex_);
  state_callback_ = state_callback;
}

}  // namespace intrinsic::opcua
