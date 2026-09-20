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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_COMMUNICATOR_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_COMMUNICATOR_H_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <set>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/realtime_clock_interface.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/udp_server_client.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/invalid_until_set.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::kuka {

// The KukaRsiCommunicator takes care of all communication with the KUKA RSI
// interface. A realtime communication thread starts when `Init()` is called
// that listens for new RSI packages from the KUKA system. It will respond in a
// timely manner regardless of the activation or enabled state to keep the RSI
// object on the robot side alive. If disabled, it sends the last command
// continuously.
class KukaRsiCommunicator {
 public:
  // All possible module states.
  enum class ModuleState : uint8_t {
    // After construction.
    kNone,
    // After Init() was called.
    kInitialized,
    // Motion disabled, but the clock will be ticked.
    kTickClockMotionDisabled,
    // Motion enabled and the clock will be ticked.
    kTickClockMotionEnabled,
    // The module was deactivated by ICON. The clock will not be ticked.
    kDeactivated,
    // A critical, non-recoverable error occurred. The clock will be ticked and
    // the error will be reported until shutdown.
    kCriticalError,
    // The module was shutdown. No communication via RSI will happen after
    // shutdown.
    kShutdown,
  };
  explicit KukaRsiCommunicator(const ThreadOptions& thread_options = {},
                               icon::RealtimeClockInterface* realtime_clock =
                                   nullptr) INTRINSIC_NON_REALTIME_ONLY;

  virtual ~KukaRsiCommunicator() INTRINSIC_NON_REALTIME_ONLY;

  // Initializes this class by starting a realtime communication thread and
  // opening the UDP connection RSI communication. This function returns only
  // after the reader thread started working.
  absl::Status Init(const KukaConfig& config) INTRINSIC_NON_REALTIME_ONLY;

  // Stops the RSI communication thread and closes the UDP connection, if
  // running. Then starts the RSI communication thread and opens the UDP
  // connection (again).
  absl::Status Prepare();

  // Closes the UDP connection used for RSI communication and stops the reader
  // thread.
  virtual absl::Status Shutdown() INTRINSIC_NON_REALTIME_ONLY;

  // Read the last telemetry (robot state) received from the robot via RSI.
  virtual RsiTelemetry GetTelemetry() const INTRINSIC_CHECK_REALTIME_SAFE;

  // Set a command to be send to the robot via RSI.
  virtual icon::RealtimeStatus SetCommand(
      absl::Span<const double> joint_positions,
      absl::Span<const bool> digital_outputs) INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns whether RSI is currently active, i.e. ticks the real-time clock.
  virtual bool IsActive() const INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns whether sending new commands is currently enabled.
  virtual bool IsEnabled() const INTRINSIC_CHECK_REALTIME_SAFE {
    return module_state_ == ModuleState::kTickClockMotionEnabled;
  }

  // Enables sending of new commands to the robot. Commands must be provided in
  // time, otherwise an error message will be printed and the old position will
  // be sent again. This can lead to faults due to instant stop request due to
  // sending the same position command twice.
  // Must be called as part of the ICON/HWM lockstep, i.e. in
  // `HardwareModuleInterface::Enabled()`.
  void Enable() INTRINSIC_CHECK_REALTIME_SAFE;

  // Disables sending of new commands to the robot. The latest position command,
  // when the robot was still enabled or the start up position, will be sent
  // continuously. If called during motion, the robot will likely fault.
  // Must be called as part of the ICON/HWM lockstep, i.e. in
  // `HardwareModuleInterface::Disabled()`.
  void Disable() INTRINSIC_CHECK_REALTIME_SAFE;

  // Activates ticking of the realtime clock.
  icon::RealtimeStatus Activate() INTRINSIC_CHECK_REALTIME_SAFE {
    return SwitchModuleState(ModuleState::kTickClockMotionDisabled);
  }

  // Deactivates ticking of the realtime clock. The RSI reader thread
  // will just read the telemetry and send the latest command to the robot to
  // keep the RSI healthy.
  icon::RealtimeStatus Deactivate() INTRINSIC_CHECK_REALTIME_SAFE;

 private:
  static absl::string_view ToString(ModuleState module_state)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Listens to RSI packets and responds with the newest command. This is the
  // central function of this class.
  void RsiCommunicationThreadJob(absl::Notification& ready);
  // Runs the RSI communication in a loop until cancel_rsi_communication_thread_
  // is set.
  icon::RealtimeStatusOr<size_t> RsiCommunicationThreadLoop()
      INTRINSIC_CHECK_REALTIME_SAFE;

  absl::Status StopRsiCommunicationThread() INTRINSIC_NON_REALTIME_ONLY;

  // Performs some validity checks on the telemetry data and set the
  // `report_fault_` variable in case of issues.
  void CheckTelemetryData(const RsiTelemetry& telemetry,
                          const RsiTelemetry& new_telemetry)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Handles the disconnected RSI state, e.g. tick the ICON clock.
  icon::RealtimeStatus HandleDisconnectedRSIState(
      const icon::RealtimeStatus& status) INTRINSIC_CHECK_REALTIME_SAFE;

  // Prepares and sends the next RSI command to the KUKA robot controller via
  // UDP.
  void PrepareAndSendRSICommand(uint64_t interpolator_counter_ms)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Ticks the realtime clock of the ICON server.
  icon::RealtimeStatus TickClock() INTRINSIC_CHECK_REALTIME_SAFE;

  // Read a telemetry value from RSI. Returns UnavailableError if no data is
  // received or InternalError if there is an error reading the data.
  icon::RealtimeStatusOr<RsiTelemetry> ReadRsiTelemetry()
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Switches the module state if the transition given the current
  // `module_state_` is allowed.
  icon::RealtimeStatus SwitchModuleState(ModuleState new_state)
      INTRINSIC_CHECK_REALTIME_SAFE;

  KukaConfig config_;
  // Clock that drives the ICON clock. Will be ticked from the reader thread
  // if `Activate()` was called.
  icon::RealtimeClockInterface* realtime_clock_;
  // A UDP server for socket communication
  std::unique_ptr<icon::UdpServer> udp_server_;
  RSIXML rsi_xml_;

  ThreadOptions rsi_communication_thread_options_;
  Thread rsi_communication_thread_;
  std::atomic_bool cancel_rsi_communication_thread_{false};

  // Tracks whether a telemetry package has been received yet. Will not ever be
  // reset.
  std::atomic_bool has_received_telemetry_;
  // Whether RSI is currently active, i.e. data has been received from the RSI
  // recently.
  std::atomic_bool rsi_active_{false};
  // Number of cycles left before the RSI client is considered enabled. This
  // gives filters/controllers time to adjust to a position jump when enabling
  // RSI.
  std::atomic_int64_t remaining_warm_up_cycles_ = 0;
  static_assert(decltype(remaining_warm_up_cycles_)::is_always_lock_free);
  // Counts the consecutive cycles in which the RSI correction was reported to
  // be inactive.
  std::atomic_uint64_t consecuctive_inactive_rsi_correction_cycles_ = 0;
  static_assert(decltype(rsi_active_)::is_always_lock_free);

  std::atomic<ModuleState> module_state_{ModuleState::kNone};
  static_assert(decltype(module_state_)::is_always_lock_free);
  // Contains all allowed transitions between module states. Transitions to
  // kCriticalError and to kShutdown are always possible.
  const absl::flat_hash_map<ModuleState, std::set<ModuleState>>
      allowed_module_state_transitions_ = {
          {ModuleState::kNone, {ModuleState::kInitialized}},
          {ModuleState::kInitialized,
           {ModuleState::kTickClockMotionDisabled, ModuleState::kDeactivated}},
          {ModuleState::kTickClockMotionDisabled,
           {ModuleState::kTickClockMotionEnabled, ModuleState::kDeactivated}},
          {ModuleState::kTickClockMotionEnabled,
           {ModuleState::kTickClockMotionDisabled, ModuleState::kDeactivated}},
          {ModuleState::kDeactivated, {ModuleState::kTickClockMotionDisabled}},
          {ModuleState::kShutdown, {ModuleState::kInitialized}}};

  // The time when the most recent telemetry was received.
  InvalidUntilSet<absl::Time> time_last_telemetry_;

  // A buffer to receive RSI telemetry.
  FixedVector<char, kTelemetryBufferSize> in_buffer_;
  // Additional buffer to receive additional RSI telemetry packets in case the
  // communication thread on our side is too slow.
  FixedVector<char, kTelemetryBufferSize> in_buffer_additional_packets_;
  // A buffer to send RSI commands as XML string.
  FixedVector<char, kCommandBufferSize> out_buffer_;

  // The most recent telemetry value. Only to be written by the RSI
  // communication thread and read by the ICON RT thread.
  mutable AsyncBuffer<RsiTelemetry> telemetry_;
  // The next command to send via RSI. Only to be used from RSI
  // communication thread.
  RSICommand rsi_command_;

  // The offset for RSI position commands. This is the position of the joints
  // when RSI was last started, all position commands send to the KUKA robot
  // controller are relative to it.
  std::array<double, kuka::kKukaNumJoints> rsi_position_offset_{};

  struct CyclicCommands {
    std::array<double, kuka::kKukaNumJoints> position;
    FixedVector<bool, kuka::kKukaMaxDigitalSignals> digital_output;
  };

  // The RSI position command, relative to the RSI position offset. Only to be
  // written by the ICON RT thread and to be read by the RSI communication
  // thread.
  AsyncBuffer<CyclicCommands> received_cyclic_commands_;

  // Variable to report faults from the reader thread to the ICON realtime
  // thread.
  AsyncBuffer<icon::RealtimeStatus> report_fault_;
  // Whether the next RSI command should be dropped. This is set to true if the
  // clock ticking took too long.
  std::atomic<bool> drop_next_command_{false};
};
}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_COMMUNICATOR_H_
