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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_EKI_SYSTEM_CONTROL_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_EKI_SYSTEM_CONTROL_H_

#include <stdbool.h>

#include <atomic>
#include <list>
#include <memory>
#include <queue>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_system_control_interface.h"
#include "intrinsic/icon/utils/tcp_server_client.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic::kuka {

// Implements controlling the system state of a KUKA robot via EKI (Ethernet KRL
// interface) using a TCP connection and a running KRL program on the KRC side
// (e.g. as a KUKA submit interpreter, a periodic additional task on the KRC).
class KukaEkiSystemControl : public KukaSystemControlInterface {
 public:
  struct EkiTelemetry {
    // The current operation mode.
    OpMode op_mode = OpMode::kNone;
    // True if RSI is currently active, i.e. the specific output variable is set
    // by the KUKA internal RSI routine. Could potentially be outdated if RSI
    // was killed abnormally.
    bool rsi_active = false;
    // True when any messages are present on the controller. Corresponds to KUKA
    // $STOP_MESS. Will only be reset, when the robot is enabled again.
    bool messages_present = false;
    // True when a program is actively running.
    bool program_active = false;
    // True when a program is selected.
    bool program_selected = false;
    // Path of the currently selected program.
    std::string selected_program_path;
  };

  // The different types of operations possible on the KRC via EKI.
  enum class EkiOperation {
    kNone,
    // Start the program on the KUKA controller. Which program is started, is
    // defined in the submit interpreter on the KUKA controller side.
    // TODO: Make program configurable again.
    kStartProgram,
    // Stop the robot and the program, so that the program could be reset or
    // resumed.
    kStopProgram,
    // Clear faults on the robot. Note that some faults can only be cleared on
    // the teach pendant itself.
    kClearFaults,
    // Deselects the active program so that another program could be loaded.
    // Stop program needs to be called beforehand.
    kCancelProgram
  };

  static absl::string_view EkiOperationToString(EkiOperation operation) {
    switch (operation) {
      case EkiOperation::kNone:
        return "None";
      case EkiOperation::kStartProgram:
        return "StartProgram";
      case EkiOperation::kStopProgram:
        return "StopProgram";
      case EkiOperation::kClearFaults:
        return "ClearFaults";
      case EkiOperation::kCancelProgram:
        return "CancelProgram";
      default:
        break;
    }
  }
  struct EkiCommand {
    EkiOperation operation = EkiOperation::kNone;
  };

 private:
  explicit KukaEkiSystemControl(
      const KukaEkiClientParams& params,
      std::unique_ptr<icon::TcpClient> tcp_client = nullptr);

 public:
  static absl::StatusOr<std::unique_ptr<KukaEkiSystemControl>> Create(
      const KukaEkiClientParams& params,
      std::unique_ptr<icon::TcpClient> tcp_client = nullptr);
  ~KukaEkiSystemControl() override;

  FixedVector<OpMode, OpModeCount> CompatibleOperationModes() const override {
    return {OpMode::kExternal};
  }
  absl::Status StartRSI(absl::Duration timeout) override;
  absl::Status StopRSI() override;
  absl::Status ClearFaults() override;
  absl::Status CancelProgram();

  absl::StatusOr<KukaErrorFlagsMask> ActiveErrorFlags() const override;

  // Returns the most recent EKI telemetry data.
  EkiTelemetry Telemetry() const {
    absl::MutexLock lock(mutex_);
    return eki_telemetry_;
  }

  // Computes and returns the average duration between sending and receiving a
  // request from KUKA EKI. The majority of the duration is usually spent
  // on waiting for the reply.
  absl::Duration ComputeAverageReplyDuration() const;

 private:
  absl::Status Init();
  void EkiCommunicationThreadJob();
  // Checks if we have a connection. If not tries to reconnect. Returns false if
  // reconnect also fails.
  bool HandleEKIConnection(absl::Duration& reconnect_interval);
  //  Sends the most recent command in the queue, or a None command when the
  //  queue is empty.
  bool SendEkiCommand();
  // Waits for the next EKI telemetry package on the TCP socket and stores it in
  // `eki_telemetry_`. On any error, e.g. timeout, the old telemetry is kept.
  void ReceiveEkiTelemetry(absl::Duration timeout);
  absl::StatusOr<std::string> AssembleEkiXml(
      const EkiCommand& eki_command) const;
  absl::StatusOr<EkiTelemetry> ParseEkiXml(absl::string_view xml) const;

  // TCP client that communicates with EKI on the KRC.
  std::unique_ptr<icon::TcpClient> tcp_client_;
  // Frequency at which commands are send to the KRC.
  double update_frequency_;
  // Thread that communicates with the KRC via TCP.
  intrinsic::Thread communication_thread_;
  // If true, the communication_thread function will return.
  std::atomic_bool shutdown_requested_ = false;

  mutable absl::Mutex mutex_;
  // Latest telemetry data.
  EkiTelemetry eki_telemetry_ ABSL_GUARDED_BY(mutex_);
  // Commands queue that will be send to the KRC in the next cycles. If empty, a
  // NoOp command will be send instead.
  std::queue<EkiCommand> eki_command_queue_ ABSL_GUARDED_BY(mutex_);
  // Durations in seconds from cycle start until the EKI replied and
  // the message was parsed. The sleep time to keep the cycle is *not* included.
  // Thus, the majority of the duration is the time the KRC needed to reply.
  std::list<double> reply_durations_;
};
}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_EKI_SYSTEM_CONTROL_H_
