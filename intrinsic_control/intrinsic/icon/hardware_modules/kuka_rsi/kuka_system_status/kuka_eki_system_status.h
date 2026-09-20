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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_EKI_SYSTEM_STATUS_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_EKI_SYSTEM_STATUS_H_
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_system_status_interface.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/tcp_server_client.h"
#include "intrinsic/util/invalid_until_set.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::kuka {

// The software version of the RSI HWM. Used to compare against
// the version on the KRC side. The patch version should always be 0 since
// patches of the same version should be compatible.
const Version kMinimumRsiHwmSoftwareVersion({1, 0, 0});
// The first version of the RSI HWM that is incompatible with the current
// version of the Intrinsic option package. Must be greater than
// `minimum_rsi_hwm_software_version`.
const Version kFirstIncompatibleRsiHwmSoftwareVersion({1, 2, 0});
// The protocol version of the RSI HWM. Used to compare against
// the version on the KRC side. The patch version should always be 0 since
// patches of the same version should be compatible.
const Version kMinimumRsiHwmProtocolVersion({1, 0, 0});
// The first version of the RSI HWM protocol that is incompatible with the
// current version of the Intrinsic option package. Must be greater than
// `minimum_rsi_hwm_protocol_version`.
const Version kFirstIncompatibleRsiHwmProtocolVersion({1, 2, 0});

// Implements reading the system state of a KUKA robot via EKI (Ethernet KRL
// interface) using a TCP connection and a running KRL program on the KRC side
// (e.g. as a submit interpreter).
class KukaEkiSystemStatus : public KukaSystemStatusInterface {
 public:
  // LINT.IfChange
  struct EkiTelemetry {
    size_t cycle = 0;
    // The software version of the RSI HWM and of the Intrinsic option package
    // on KRC side.
    std::string software_version;
    // The protocol version of EKI and RSI of the Intrinsic option package on
    // KRC side.
    std::string protocol_version;
    // String representation of the robot model.
    std::string robot_model;
    // Serial number of the robot as given by KUKA.
    uint32_t serial_no;
    // The position of each joint, in order, in radians.
    eigenmath::Vectord<kKukaNumJoints> position =
        eigenmath::VectorXd::Zero(kKukaNumJoints);
    // True if the any emergency stop is pressed.
    bool emergency_stop_active = false;
    // True if the internal emergency stop is pressed.
    bool internal_emergency_stop_active = false;
    // True if the operator safety is closed, e.g. safety PLC reports OK.
    bool operator_safety_closed = false;
    // The current operation mode.
    OpMode op_mode = OpMode::kNone;
    // True if RSI is currently active, i.e. the specific output variable is set
    // by the KUKA internal RSI routine. Could potentially be outdated if RSI
    // was killed abnormally.
    bool rsi_active = false;
    // True when any messages are present on the controller. Corresponds to KUKA
    // $STOP_MESS. Will only be reset, when the robot is enabled again.
    bool messages_present = false;
    // A set of all active KUKA KSS messages represented as their message
    // number.
    std::set<int32_t> message_numbers;
    // True when a program is actively running, i.e. green R on the teach
    // pendant.
    bool program_active = false;
    // True when a program is selected, i.e. shown on the teach pendant.
    bool program_selected = false;
    // The maximum payload mass that the robot can carry, in kg.
    std::optional<double> max_payload_mass;
    // The current payload of the robot as configured currently on the KUKA KRC.
    std::optional<KukaPayload> payload;
  };
  // LINT.ThenChange(//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_eki_system_status.cc:ResetFaultStorage)

  // Command to send to the EKI.
  struct EkiCommand {
    std::optional<RobotPayloadBase> payload;
  };

 private:
  explicit KukaEkiSystemStatus(
      const KukaEkiClientParams& params,
      std::unique_ptr<icon::TcpClient> tcp_client = nullptr);

 public:
  // Creates an instance of the KukaEkiSystemStatus with the configuration
  // parameters given in `params`. `tcp_client` can be injected or left as null.
  // If null, the tcp client will be created based on the values in `params`.
  static absl::StatusOr<std::unique_ptr<KukaEkiSystemStatus>> Create(
      const KukaEkiClientParams& params,
      std::unique_ptr<icon::TcpClient> tcp_client = nullptr);
  ~KukaEkiSystemStatus() override;

  InvalidUntilSet<EkiTelemetry> Telemetry() const ABSL_LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock lock(mutex_);
    return eki_telemetry_;
  }

  std::string Name() const override { return "EKI"; }

  absl::StatusOr<OpMode> CurrentOpMode() const override
      ABSL_LOCKS_EXCLUDED(mutex_);

  absl::StatusOr<KukaErrorFlagsMask> ActiveErrorFlags() const override;

  std::vector<std::string> ErrorMessages() const override
      ABSL_LOCKS_EXCLUDED(mutex_);

  icon::RealtimeStatusOr<eigenmath::Vectord<kKukaNumJoints>> CurrentPosition()
      const override INTRINSIC_CHECK_REALTIME_SAFE;

  absl::StatusOr<uint32_t> RobotSerialNumber() const override
      ABSL_LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock lock(mutex_);
    if (!eki_telemetry_) {
      return absl::UnavailableError("");
    }
    return eki_telemetry_->serial_no;
  }

  absl::StatusOr<std::string> RobotModel() const override
      ABSL_LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock lock(mutex_);
    if (!eki_telemetry_) {
      return absl::UnavailableError("");
    }
    return eki_telemetry_->robot_model;
  }

  bool WaitForNewData(absl::Duration timeout) const override
      ABSL_LOCKS_EXCLUDED(mutex_);

  void ResetFaultStorage() override ABSL_LOCKS_EXCLUDED(mutex_);

  // Set the payload in the EKI command which is sent in the next EKI
  // communication cycle and will be applied on the next RSI start. Returns when
  // the next EKI command was sent and a reply was received.
  //
  // Note: Does not wait until or check if the payload was actually applied,
  // since this happens later during the next RSI program start (i.e. during
  // EnableMotion).
  absl::Status SetPayload(const RobotPayloadBase& payload) override;

 private:
  // Initializes the connection to the EKI via TCP and starts the communication
  // thread.
  absl::Status Init();
  // Communicates with the KRC via TCP, manages the connection and parses the
  // incoming data.
  void EkiCommunicationThreadJob();
  // Checks if we have a connection. If not tries to reconnect. Returns false if
  // reconnect also fails.
  bool HandleEKIConnection(absl::Duration& reconnect_interval);
  // Sends a new request to the EKI.
  bool SendEkiRequest();
  // Waits for the next EKI telemetry package on the TCP socket and stores it in
  // `eki_telemetry_`.
  absl::Status ReceiveEkiTelemetry(absl::Duration timeout);
  absl::StatusOr<std::string> AssembleEkiXml(
      const EkiCommand& eki_command) const;
  absl::StatusOr<EkiTelemetry> ParseEkiXml(absl::string_view xml) const;

  // TCP client that communicates with EKI on the KRC.
  std::unique_ptr<icon::TcpClient> tcp_client_;
  // True if connect to. Unset before first iteration of the
  // communication thread.
  InvalidUntilSet<std::atomic_bool> is_connected_;
  // Frequency at which requests are send to the KRC.
  double update_frequency_;
  // Thread that communicates with the KRC via TCP.
  intrinsic::Thread communication_thread_;
  // If true, the communication_thread function will return.
  std::atomic_bool shutdown_requested_ = false;

  mutable absl::Mutex mutex_;
  // Latest telemetry data.
  InvalidUntilSet<EkiTelemetry> eki_telemetry_ ABSL_GUARDED_BY(mutex_);
  // Command to be sent to the EKI in the next cycle of `communication_thread_`.
  EkiCommand eki_command_ ABSL_GUARDED_BY(mutex_);
  // Condition variable used to wait for new data from the
  // EKI interface.
  mutable absl::CondVar new_data_arrived_;

  // Latest joint positions in extra member since this value is accessed by the
  // realtime thread.
  mutable AsyncBuffer<InvalidUntilSet<eigenmath::Vectord<kKukaNumJoints>>>
      position_;

  enum class KssMessageSeverity {
    kUnknown,
    kInfo,
    kState,
    kAcknowledgment,
    kError,
    kInfoOrAcknowledgment
  };
  static KssMessageSeverity StringToKssMessageSeverity(absl::string_view str);

  // Data of a KUKA KSS message.
  struct KssMessage {
    // String representation of the KSS message.
    std::string message;
    // Message number of the KSS message.
    int32_t message_number;
    // Severity of the KSS message.
    KssMessageSeverity severity;
  };
  // Creates a map of KSS messages from the generated c-array.
  static absl::flat_hash_map<std::string,
                             absl::flat_hash_map<int32_t, KssMessage>>
  CreateKssMessageMap();
  // Container for all KSS Messages that is filled in the constructor.
  const absl::flat_hash_map<std::string,
                            absl::flat_hash_map<int32_t, KssMessage>>
      kss_messages_map_;
};
}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_EKI_SYSTEM_STATUS_H_
