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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_client.h"

#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"
#include "open62541/client.h"
#include "open62541/client_config_default.h"
#include "open62541/client_highlevel.h"
#include "open62541/types.h"
#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"

inline constexpr absl::string_view kOpcNodePrefix = "::AsGlobalPV:";
// Interval at which the OPC-UA keep-alive function will be called.
inline constexpr absl::Duration kOpcUaKeepAliveInterval = absl::Seconds(30);

namespace intrinsic::kuka {

absl::string_view KukaPlcStateToString(KukaPlcState state) {
  switch (state) {
    case KukaPlcState::kReady:
      return kKukaPlcStateReady;
    case KukaPlcState::kStopMessagesActive:
      return kKukaPlcStateStopMessagesActive;
    case KukaPlcState::kWaitForPrgNoReq:
      return kKukaPlcStateWaitForPrgNoReq;
    case KukaPlcState::kRunning:
      return kKukaPlcStateRunning;
    case KukaPlcState::kWaitForApplRun:
      return kKukaPlcStateWaitForApplRun;
    case KukaPlcState::kEStop:
      return kKukaPlcStateEStop;
    default:
      return kKukaPlcStateUnknown;
  }
}
std::ostream& operator<<(std::ostream& os, const KukaPlcState& state) {
  return os << KukaPlcStateToString(state);
}

absl::StatusOr<std::unique_ptr<KukaPlcClient>> KukaPlcClient::Create(
    std::string_view plc_opcua_address, std::string_view node_name,
    absl::Duration plc_opcua_timeout) {
  auto client = std::unique_ptr<KukaPlcClient>(
      new KukaPlcClient{plc_opcua_address, node_name, plc_opcua_timeout});
  INTR_RETURN_IF_ERROR(client->Init());
  return client;
}

KukaPlcClient::KukaPlcClient(std::string_view plc_opcua_address,
                             std::string_view node_name,
                             absl::Duration plc_opcua_timeout)
    : client_(UA_Client_new(), UA_Client_delete),
      base_node_name_(absl::StrCat(kOpcNodePrefix, node_name, ".Kuka.")),
      plc_opcua_address_(plc_opcua_address),
      plc_opcua_timeout_(plc_opcua_timeout) {
  UA_ClientConfig* config = UA_Client_getConfig(client_.get());
  QCHECK(config != nullptr);
  UA_StatusCode ua_status = UA_ClientConfig_setDefault(config);
  if (ua_status != UA_STATUSCODE_GOOD) {
    LOG(INFO) << (absl::StrFormat(
        "KukaPlcClient was not able to set the default UA config. "
        "Error: %s",
        UA_StatusCode_name(ua_status)));
  }

  // TODO(b/376690605): Reduce log verbosity.

  config->timeout = absl::ToInt64Milliseconds(plc_opcua_timeout_);
  UA_Variant_init(&temp_variant_);

  LOG(INFO) << "KukaPlcClient initialized with base node name: "
            << base_node_name_;

  // Initialize the Variant value for "true".
  UA_Boolean b = true;
  UA_Variant_setScalarCopy(&true_value_, &b, &UA_TYPES[UA_TYPES_BOOLEAN]);
}

KukaPlcClient::~KukaPlcClient() {
  shutdown_requested_.Notify();
  if (opcua_keep_alive_thread_.joinable()) {
    opcua_keep_alive_thread_.join();
  }
  UA_Variant_clear(&temp_variant_);
  UA_Variant_clear(&true_value_);
}

absl::Status KukaPlcClient::Init() {
  // This is not technically a real-time thread and when intrinsic::Thread
  // offers an API to specify a thread name, that API is recommended instead.
  INTR_ASSIGN_OR_RETURN(
      opcua_keep_alive_thread_,
      CreateRealtimeCapableThread(ThreadOptions().SetName("opcua_keepalive"),
                                  [this] { OpcuaKeepAliveThreadJob(); }));

  return Connect();
}

void KukaPlcClient::OpcuaKeepAliveThreadJob() {
  while (!shutdown_requested_.WaitForNotificationWithTimeout(
      kOpcUaKeepAliveInterval)) {
    absl::MutexLock lock(opcua_mutex_);
    // This OPC-UA function needs to be called regularly for housekeeping.
    // Otherwise the connection will be closed after some time.
    auto ua_status = UA_Client_run_iterate(client_.get(), 1000);
    if (ua_status != UA_STATUSCODE_GOOD) {
      LOG(WARNING) << absl::StrFormat(
          "KukaPlcClient keep-alive error:"
          "%s",
          UA_StatusCode_name(ua_status));
    }
  }
  LOG(INFO) << "KukaPlcClient keep alive thread was shut down.";
}

absl::Status KukaPlcClient::Connect() {
  absl::MutexLock lock(opcua_mutex_);
  return ConnectWithoutMutex(*client_);
}

absl::Status KukaPlcClient::ConnectWithoutMutex(UA_Client& client) {
  UA_StatusCode status = UA_Client_connect(&client, plc_opcua_address_.data());

  if (status != UA_STATUSCODE_GOOD) {
    return absl::UnavailableError(absl::StrFormat(
        "KukaPlcClient was not able to connect to the OPCUA server (%s). "
        "Error: %s",
        plc_opcua_address_.data(), UA_StatusCode_name(status)));
  }
  LOG(INFO) << "KukaPlcClient is connected to: " << plc_opcua_address_;

  return absl::OkStatus();
}

absl::Status KukaPlcClient::Reconnect() {
  absl::MutexLock lock(opcua_mutex_);

  auto ua_status = UA_Client_disconnect(client_.get());
  if (ua_status != UA_STATUSCODE_GOOD) {
    LOG(INFO) << absl::StrFormat(
        "KukaPlcClient was not able to disconnect properly from the OPCUA "
        "server (%s). Error: %s. Trying to reconnect anyway.",
        plc_opcua_address_.data(), UA_StatusCode_name(ua_status));
  }

  return ConnectWithoutMutex(*client_);
}

absl::StatusOr<KukaPlcState> KukaPlcClient::getState() const {
  absl::MutexLock lock(opcua_mutex_);
  std::string node_name = absl::StrCat(base_node_name_, "Output.usiStatus");
  UA_NodeId node_id = UA_NODEID_STRING(6, (char*)node_name.c_str());
  UA_StatusCode status =
      UA_Client_readValueAttribute(client_.get(), node_id, &temp_variant_);

  if (status == UA_STATUSCODE_GOOD &&
      UA_Variant_hasScalarType(&temp_variant_, &UA_TYPES[UA_TYPES_BYTE])) {
    return static_cast<KukaPlcState>(*(uint8_t*)temp_variant_.data);
  }

  return absl::UnavailableError(absl::StrCat("Error reading the PLC's state: ",
                                             UA_StatusCode_name(status)));
}

absl::StatusOr<std::string> KukaPlcClient::getStateAsString() const {
  absl::MutexLock lock(opcua_mutex_);
  std::string node_name = absl::StrCat(base_node_name_, "Output.strStatus");
  UA_NodeId node_id = UA_NODEID_STRING(6, (char*)node_name.data());
  UA_StatusCode status =
      UA_Client_readValueAttribute(client_.get(), node_id, &temp_variant_);

  if (status == UA_STATUSCODE_GOOD &&
      UA_Variant_hasScalarType(&temp_variant_, &UA_TYPES[UA_TYPES_STRING])) {
    UA_String ua_str = *(UA_String*)temp_variant_.data;
    return std::string((char*)ua_str.data, ua_str.length);
  }

  return absl::UnavailableError(
      absl::StrCat("Error reading the PLC's state as a string: ",
                   UA_StatusCode_name(status)));
}

absl::Status KukaPlcClient::RequestStartRSI() const {
  absl::MutexLock lock(opcua_mutex_);
  std::string node_name = absl::StrCat(base_node_name_, "Input.blnStartRSIRq");
  UA_NodeId node_id = UA_NODEID_STRING(6, (char*)node_name.data());
  UA_StatusCode status =
      UA_Client_writeValueAttribute(client_.get(), node_id, &true_value_);

  if (status != UA_STATUSCODE_GOOD)
    return absl::UnavailableError(absl::StrCat(
        "Error requesting to start Rsi: ", UA_StatusCode_name(status)));

  return absl::OkStatus();
}

absl::Status KukaPlcClient::RequestStopRSI() const {
  absl::MutexLock lock(opcua_mutex_);
  std::string node_name = absl::StrCat(base_node_name_, "Input.blnStopRSIRq");
  UA_NodeId node_id = UA_NODEID_STRING(6, (char*)node_name.data());
  UA_StatusCode status =
      UA_Client_writeValueAttribute(client_.get(), node_id, &true_value_);

  if (status != UA_STATUSCODE_GOOD)
    return absl::UnavailableError(absl::StrCat("Error requesting to stop Rsi: ",
                                               UA_StatusCode_name(status)));

  return absl::OkStatus();
}

absl::Status KukaPlcClient::RequestAcknowledgeErrors() const {
  absl::MutexLock lock(opcua_mutex_);
  std::string node_name = absl::StrCat(base_node_name_, "Input.blnAckErrorRq");
  UA_NodeId node_id = UA_NODEID_STRING(6, (char*)node_name.data());
  UA_StatusCode status =
      UA_Client_writeValueAttribute(client_.get(), node_id, &true_value_);

  if (status != UA_STATUSCODE_GOOD)
    return absl::UnavailableError(
        absl::StrCat("Error requesting to acknowledge errors: ",
                     UA_StatusCode_name(status)));

  return absl::OkStatus();
}

absl::Status KukaPlcClient::WaitForState(
    KukaPlcState expected_state, absl::Duration timeout,
    absl::Duration polling_interval) const {
  absl::Time deadline = absl::Now() + timeout;

  INTR_ASSIGN_OR_RETURN(auto current_state, getState());
  while (absl::Now() < deadline) {
    if (current_state == expected_state) return absl::OkStatus();
    absl::SleepFor(polling_interval);
    INTR_ASSIGN_OR_RETURN(current_state, getState());
  }
  return absl::UnavailableError(absl::StrFormat(
      "The state did not become %s (%i) after the given "
      "timeout (%s). The state is %s (%i).",
      KukaPlcStateToString(expected_state), static_cast<int>(expected_state),
      absl::FormatDuration(timeout), KukaPlcStateToString(current_state),
      static_cast<int>(current_state)));

  return absl::OkStatus();
}

}  // namespace intrinsic::kuka
