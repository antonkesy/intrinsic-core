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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_PLC_CLIENT_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_PLC_CLIENT_H_

#include <memory>
#include <ostream>
#include <string>
#include <string_view>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/util/thread/thread.h"
#include "open62541/types.h"
#include "open62541/util.h"

namespace intrinsic::kuka {

enum class KukaPlcState {
  // The RSI cell program is ready to be started.
  kReady = 3,
  // Stop messages are active. An acknowledge request is needed to resume.
  kStopMessagesActive = 4,
  // (Transient state) WAIT_FOR_PG_ACTIVE is a transient state occurring before
  // READY.
  kWaitForPrgActive = 7,
  // (Transient state) WAIT_FOR_PGNO_REQ is a transient state occurring between
  // RUNNING and READY, also in the startup and error recovery paths.
  kWaitForPrgNoReq = 8,
  // The RSI cell program is running : the robot is controllable.
  // Note: RSI can be running without "move_corr" being active, which
  // effectively means that we can read the state of the robot but not control
  // it. In that case, the state would not be `RUNNING`.
  kRunning = 9,
  // (Transient state) WAIT_FOR_APPL_RUN is a transient state occurring between
  // READY and RUNNING
  kWaitForApplRun = 10,
  // The ESTOP state can mean that there is a safety violation that must be
  // reset using the dedicated button in the cell, or that the robot is not
  // in External mode (eg. # T1/T2). In any case, an operator intervention is
  // required.
  kEStop = 50,
};

constexpr absl::string_view kKukaPlcStateReady("Ready");
constexpr absl::string_view kKukaPlcStateStopMessagesActive(
    "StopMessagesActive");
constexpr absl::string_view kKukaPlcStateWaitForPrgNoReq("WaitForPrgNoReq");
constexpr absl::string_view kKukaPlcStateRunning("Running");
constexpr absl::string_view kKukaPlcStateWaitForApplRun("WaitForApplRun");
constexpr absl::string_view kKukaPlcStateEStop("Estop");
constexpr absl::string_view kKukaPlcStateUnknown("Unknown");

absl::string_view KukaPlcStateToString(KukaPlcState state);

std::ostream& operator<<(std::ostream& os, const KukaPlcState& state);

// This class allows interacting with a PLC in a KUKA cell, via OPC-UA.
// It can be used to start/stop RSI and clear errors.
class KukaPlcClient {
 protected:
  KukaPlcClient() : client_(nullptr, nullptr) {}  // For mocking.

 public:
  virtual ~KukaPlcClient();
  // Create an instance and connect to the given  OPC-UA address of the PLC
  // and the node name of the robot we want to interact with. For instance, if
  // the full path to one of the OPC-UA nodes is
  // "::AsGlobalPV:r5012.Kuka.Input.blnStartRSIRq", the "robot_node_name" would
  // be "r5012".
  static absl::StatusOr<std::unique_ptr<KukaPlcClient>> Create(
      std::string_view plc_opcua_address, std::string_view robot_node_name,
      absl::Duration plc_opcua_timeout);
  static absl::StatusOr<std::unique_ptr<KukaPlcClient>> Create(
      const KukaPlcClientParams& params) {
    return Create(params.opcua_address, params.opcua_node_name,
                  params.opcua_timeout);
  }

  // Disconnect and connect (again) to the OPC-UA server. This function will
  // attempt once to open a connection. On slow connections this can takes
  // several seconds. If the connect attempt fails, the function returns
  // UnavailableError.
  virtual absl::Status Reconnect();

  // Get the state of the PLC as an integer.
  virtual absl::StatusOr<KukaPlcState> getState() const;
  // Get the state of the PLC as a string, which is convenient for debugging.
  virtual absl::StatusOr<std::string> getStateAsString() const;
  // Request to start the RSI_MC() program on the KRC.
  virtual absl::Status RequestStartRSI() const;
  // Request to stop the RSI_MC() program on the KRC.
  virtual absl::Status RequestStopRSI() const;
  // Request to acknowledge errors on the KRC.
  virtual absl::Status RequestAcknowledgeErrors() const;

  // Wait until the state of the PLC becomes 'expected_state', up to 'timeout',
  // checking at 'polling_interval'.
  virtual absl::Status WaitForState(KukaPlcState expected_state,
                                    absl::Duration timeout,
                                    absl::Duration polling_interval) const;

 private:
  KukaPlcClient(std::string_view plc_opcua_address,
                std::string_view robot_node_name,
                absl::Duration plc_opcua_timeout);

  // Initialize the PLC client by starting the keep-alive thread and connecting
  // to the OPC-UA server.
  absl::Status Init();
  void OpcuaKeepAliveThreadJob();
  absl::Status Connect();
  absl::Status ConnectWithoutMutex(UA_Client& client);

  mutable absl::Mutex opcua_mutex_;
  std::unique_ptr<UA_Client, void (*)(UA_Client*)> client_
      ABSL_GUARDED_BY(opcua_mutex_);
  mutable UA_Variant temp_variant_;
  UA_Variant true_value_;

  const std::string base_node_name_;
  const std::string plc_opcua_address_;
  const absl::Duration plc_opcua_timeout_;
  absl::Notification shutdown_requested_;
  Thread opcua_keep_alive_thread_;
};

}  // namespace intrinsic::kuka

#endif  //  INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_PLC_CLIENT_H_
