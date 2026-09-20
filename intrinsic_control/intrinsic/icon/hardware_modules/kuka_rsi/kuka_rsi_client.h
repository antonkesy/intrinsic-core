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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_CLIENT_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_CLIENT_H_

#include <stdbool.h>

#include <atomic>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/realtime_clock_interface.h"
#include "intrinsic/icon/hal/proto/hardware_module_inspection.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_communicator.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_system_control_interface.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_system_status_interface.h"
#include "intrinsic/icon/interprocess/lockable_binary_futex.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::kuka {

// KUKA RSI is the real-time control interface for KUKA robots. The protocol is
// XML based.
//
// This class is the one-stop-shop for controlling KUKA robots using RSI and
// controlling the system state using various options, such as teach pendant
// mode or PLC mode. For more fine-granular control see `KukaRsiCommunicator` in
// intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_communicator.cc
// and `KukaSystemControlInterface` implementations in
// intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/,
// which are used in this class.
class RealtimeKukaRsiClient {
 public:
  // Specify `realtime_thread_options` to configure the thread used for RSI
  // communication (should be a real-time thread).
  //
  // Use `realtime_clock` to specify the real-time clock to tick with
  // each arrived RSI telemetry package. If not set, no external clock will be
  // ticked by this hardware module. The RSI communication thread will send and
  // receive data at the RSI frequency regardless how often the HWM interface
  // functions are called. If there is no new command available for RSI in a
  // cycle, this can lead to jerky motion or faults since the last command will
  // be send until there is a new command.
  //
  // Use `kuka_system_control` or `kuka_system_status` to inject the desired
  // system control/status instance. If `kuka_system_control` or
  // `kuka_system_status` is NULL, an instance will be created based on the
  // content of `config` during `Init()`. Use `std::move` to pass in the
  // unique pointers.
  explicit RealtimeKukaRsiClient(
      const ThreadOptions& realtime_thread_options = {},
      intrinsic::icon::RealtimeClockInterface* realtime_clock = nullptr,
      std::unique_ptr<KukaSystemControlInterface> kuka_system_control = nullptr,
      std::unique_ptr<KukaSystemStatusInterface> kuka_system_status = nullptr)
      INTRINSIC_NON_REALTIME_ONLY
      : kuka_system_control_(std::move(kuka_system_control)),
        kuka_system_status_(std::move(kuka_system_status)),
        rsi_communicator_(realtime_thread_options, realtime_clock) {}

  // Cannot do actual clean-up in destructor due to virtual functions. Call
  // Disconnect() manually.
  virtual ~RealtimeKukaRsiClient();

  // Initializes this instance by
  // - populating `kuka_system_control_` depending on the contents of `config`
  // - initializing the RSICommunicator instance
  // - starting the fault handling thread.
  //
  // Returns various errors from the `KukaSystemControlInterface` factory or the
  // RSICommunicator::Init() function.
  virtual absl::Status Init(const KukaConfig& config)
      INTRINSIC_NON_REALTIME_ONLY;

  // Calls `Prepare()` on the underlying RSICommunicator. This will reset the
  // internal state machine and the RT communication thread.
  virtual absl::Status Prepare() INTRINSIC_NON_REALTIME_ONLY;

  // Read the last telemetry (robot state) received from the robot via RSI.
  virtual RsiTelemetry GetTelemetry() const INTRINSIC_CHECK_REALTIME_SAFE;

  // Send a command to the robot via RSI.
  virtual intrinsic::icon::RealtimeStatus SendCommand(
      absl::Span<const double> joint_positions,
      absl::Span<const bool> digital_outputs) INTRINSIC_CHECK_REALTIME_SAFE;

  // Shutdown the RSI client. No new data can be received or sent after this
  // method has been called.
  virtual absl::Status Shutdown() INTRINSIC_NON_REALTIME_ONLY;

  // Clear faults if necessary. Needs a KukaSystemControlInterface
  // implementation that supports this feature.
  virtual absl::Status ClearFaults() INTRINSIC_NON_REALTIME_ONLY;

  // Returns whether RSI is currently active, i.e. new telemetry data has been
  // received recently from the KUKA robot controller.
  virtual bool IsActive() const INTRINSIC_CHECK_REALTIME_SAFE {
    return rsi_communicator_.IsActive();
  }

  // Attempts to start RSI (if the chosen KukaSystemControlInterface supports
  // this) and waits until it is active.
  absl::Status Enable() INTRINSIC_NON_REALTIME_ONLY;

  // Enables accepting new commands for the robot. Commands must be provided in
  // time, otherwise an error message will be printed and the old position will
  // be sent again. This can lead to faults due to a implicit instant stop
  // request (due to sending the same position command twice).
  icon::RealtimeStatus Enabled() INTRINSIC_CHECK_REALTIME_SAFE;
  // Disables accepting new commands for the robot. The latest position command
  // will be sent continuously via RSI to the KRC.
  icon::RealtimeStatus Disabled() INTRINSIC_CHECK_REALTIME_SAFE;

  // Stops RSI on the robot and disables sending of new commands to the robot.
  // The latest position command, when the robot was still enabled or the start
  // up position, will be sent continuously in case the used
  // KukaSystemControlInterface does not support stopping RSI.
  absl::Status Disable() INTRINSIC_NON_REALTIME_ONLY;

  // Returns whether the RSI is enabled, i.e. whether new commands will be
  // forwarded via the RSI to the robot.
  virtual bool IsEnabled() const INTRINSIC_CHECK_REALTIME_SAFE {
    return rsi_communicator_.IsEnabled();
  }

  // Activates ticking of the realtime clock.
  virtual icon::RealtimeStatus Activate() INTRINSIC_CHECK_REALTIME_SAFE;

  // Deactivates ticking of the realtime clock. The RSI reader thread
  // will just read the telemetry and send the latest command to the robot to
  // keep the RSI healthy.
  virtual icon::RealtimeStatus Deactivate() INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns a string representation of active KUKA fault(s) (also some other
  // faults). Depending on the used KukaSystemControlInterface this might never
  // contain any faults or only a subset of actually active faults.
  virtual icon::RealtimeStatus GetFault() const INTRINSIC_CHECK_REALTIME_SAFE;

  // Sets the payload of the robot. This will be sent to the robot via the
  // chosen KukaSystemStatusInterface implementation, e.g. via EKI.
  virtual absl::Status SetPayload(const RobotPayloadBase& payload) {
    return kuka_system_status_->SetPayload(payload);
  }

  // Provides inspection data to the hardware module runtime.
  absl::Status ProvideInspectionData(
      intrinsic_proto::icon::v1::HardwareModuleInspectionData& data);

 private:
  // Intermediate data structure to store inspection data over multiple cycles.
  struct InspectionData {
    std::vector<std::string> error_messages;
    bool emergency_stop_active;
    std::optional<OpMode> op_mode;
    std::list<intrinsic_proto::icon::v1::Event> event_history;
  };

  // Start RSI. This acknowledges errors if necessary. RSI must be started in
  // order to receive telemetry and send commands.
  virtual absl::Status StartRSI(absl::Time deadline);

  // Polls the RSI state from the communicator until it reports `active` or the
  // deadline is reached.
  //
  // Returns OK if RSI becomes active before the deadline.
  // Returns DeadlineExceededError otherwise.
  absl::Status WaitForRSIStart(absl::Time deadline);

  // Triggers stopping of the RSI on the robot side. Does *not* wait until it is
  // actually stopped. The used KukaSystemControlInterface must support stopping
  // the RSI, otherwise this function will have no effect.
  virtual absl::Status StopRSI();

  // Thread to periodically check for faults reported by
  // `kuka_system_control_` and forward them to the realtime thread.
  void NonRealtimeThreadJob();

  // Converts and updates the internal inspection data storage with the given
  // values.
  void UpdateInspectionData(const std::vector<std::string>& error_messages,
                            bool emergency_stop_active,
                            std::optional<OpMode> op_mode);

  KukaConfig config_;
  std::unique_ptr<KukaSystemControlInterface> kuka_system_control_;
  std::unique_ptr<KukaSystemStatusInterface> kuka_system_status_;
  KukaRsiCommunicator rsi_communicator_;
  // Thread to monitor the kuka system state using the `kuka_system_status_`
  // instance.
  intrinsic::Thread kuka_system_status_monitor_thread_;
  std::atomic_bool shutdown_requested_ = false;

  mutable icon::LockableBinaryFutex fault_to_report_mutex_;
  // Variable to report faults from various sources to the ICON rt thread.
  icon::RealtimeStatus fault_to_report_
      ABSL_GUARDED_BY(fault_to_report_mutex_){};
  std::atomic_bool reset_fault_to_report_ = false;
  static_assert(decltype(reset_fault_to_report_)::is_always_lock_free);

  absl::Mutex inspection_data_mutex_;
  InspectionData inspection_data_ ABSL_GUARDED_BY(inspection_data_mutex_);
};

}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_CLIENT_H_
