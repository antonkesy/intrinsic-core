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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_LOOPBACK_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_LOOPBACK_H_

#include <atomic>
#include <cstdint>
#include <optional>

#include "absl/base/thread_annotations.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/udp_server_client.h"
#include "intrinsic/icon/utils/xml/realtime_xml_generator.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::kuka {
class RsiLoopbackXml {
 public:
  RsiLoopbackXml() {
    RSIXML::CreateXMLAttributeStrings("DO", digital_outputs_);
    RSIXML::CreateXMLAttributeStrings("DI", digital_inputs_);
    RSIXML::CreateXMLAttributeStrings("A", joint_axis_names_);
  }
  icon::RealtimeStatus ParseCommand(absl::Span<const uint8_t> buffer,
                                    RSICommand& command);
  icon::RealtimeStatus AssembleTelemetry(
      const RsiTelemetry& telemetry,
      FixedVector<uint8_t, kTelemetryBufferSize>& buffer);

 private:
  RSIXML rsi_xml_;
  RealtimeXmlGenerator xml_generator_ = RealtimeXmlGenerator(
      /*max_element_count=*/10, /*max_attribute_count=*/200);
  FixedVector<icon::FixedString<5>, kKukaMaxDigitalSignals> digital_outputs_;
  FixedVector<icon::FixedString<5>, kKukaMaxDigitalSignals> digital_inputs_;
  FixedVector<icon::FixedString<3>, kKukaNumJoints> joint_axis_names_;
};

// The RSI loopback fakes the robot side of RSI and just returns the command
// position as actual position and a sinusoidal curve for all joint torques.
class KukaRsiLoopback {
 public:
  KukaRsiLoopback(absl::string_view host, uint16_t port,
                  double frequency = 250.0)
      : cycle_time_(absl::Seconds(1.0 / frequency)), udp_client_(host, port) {
    LOG(INFO) << "Starting RSI loopback on " << host << ":" << port;
  }

  // Sets up the UDP connection.
  absl::Status Prepare();

  // Calls `RunCycle()` at the desired frequency.
  icon::RealtimeStatus Run();

  // Sends the current telemetry and receives the next command via UDP.
  // `cycle_start_timestamp_ms` needs to contain a perfect timestamp of the
  // current cycle, i.e. not measured from a real clock but incremented by the
  // cycle frequency. `cycle_deadline` must contain a deadline using the system
  // clock (i.e. absl::Now()), specifying when the function should return (only
  // best effort).
  icon::RealtimeStatus RunCycle(uint64_t cycle_start_timestamp_ms,
                                absl::Time cycle_deadline);

  // Aborts the loop in `Run()`.
  void Shutdown() { shutdown_requested_ = true; }

  // Returns whether or not the loop of `Run()` has ended (due to error or to
  // call to `Shutdown()`).
  bool IsShutdown() const { return is_shutdown_; }

  // Sets the operation mode that will be reported.
  void SetOpMode(OpMode op_mode) { op_mode_ = op_mode; }

  // Sets the digital inputs that will be reported on the next cycle.
  void SetDigitalInputs(absl::Span<const bool> digital_inputs);

  // Returns the latest command received from the RSI client.
  std::optional<RSICommand> LatestCommand() const;

 private:
  RsiLoopbackXml rsi_xml_;
  absl::Duration cycle_time_;
  icon::UdpClient udp_client_;
  std::optional<RSICommand> last_command_ ABSL_GUARDED_BY(mutex_);
  std::atomic_bool shutdown_requested_ = false;
  std::atomic_bool is_shutdown_ = false;

  std::atomic<OpMode> op_mode_ = OpMode::kAutomatic;
  static_assert(decltype(op_mode_)::is_always_lock_free);

  mutable absl::Mutex mutex_;
  FixedVector<bool, kKukaMaxDigitalSignals> digital_input_command_
      ABSL_GUARDED_BY(mutex_);
};
}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_LOOPBACK_H_
