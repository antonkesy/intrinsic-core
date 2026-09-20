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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_communicator.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/cleanup/cleanup.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/notification.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/realtime_clock_interface.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/strerror.h"
#include "intrinsic/icon/utils/udp_server_client.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::kuka {
namespace {
// Duration to wait for the RSI communication thread to start.
constexpr absl::Duration kThreadStartupTimeout = absl::Seconds(2);
// Duration to wait for the RSI telemetry to be received from the KUKA robot
// controller.
constexpr absl::Duration kWaitForTelemetryTimeout = absl::Seconds(2);

// Duration of one RSI control cycle (250 Hz).
constexpr const absl::Duration kCycleDuration = absl::Milliseconds(4);
// Conservative time buffer reserved for the duration it takes to send the RSI
// package back to the KUKA robot controller and for the controller to process
// it so that it does not complain about a late package.
constexpr const absl::Duration kCycleReplyTimeBuffer = absl::Milliseconds(1);
// Tolerated lateness of ICON to the KUKA robot controller. If ICON is more than
// this late, this HWM will fault the ICON server by resetting the clock.
constexpr const absl::Duration kToleratedIconLateness = absl::Milliseconds(1);
// Module states in which the clock should be ticked.
constexpr std::array<KukaRsiCommunicator::ModuleState, 2> kClockTickingStates =
    {KukaRsiCommunicator::ModuleState::kTickClockMotionEnabled,
     KukaRsiCommunicator::ModuleState::kTickClockMotionDisabled};
// Specifies the number of consecutive cycles in which the rsi_correction_active
// flag can have a different value than the rsi_active flag before RSI is
// considered to be not active.
constexpr size_t kMaxInactiveRsiCorrectionCyclesUntilFault = 10;
// Number of cycles after enabling the RSI interface until the RSI client is
// considered enabled/active. This gives filters and controllers time to adjust
// to a position jump, which happens when EKI is not used.
constexpr size_t kWarmUpCycleCount = 30;

}  // namespace

KukaRsiCommunicator::KukaRsiCommunicator(
    const ThreadOptions& thread_options,
    icon::RealtimeClockInterface* realtime_clock)
    : realtime_clock_(realtime_clock),
      rsi_communication_thread_options_(thread_options) {}

KukaRsiCommunicator::~KukaRsiCommunicator() {
  if (auto status = Shutdown(); !status.ok()) {
    LOG(ERROR) << "Failed to shutdown: " << status;
  }
}

absl::Status KukaRsiCommunicator::Init(const KukaConfig& config) {
  config_ = config;

  LOG(INFO) << "Connecting to RSI socket via UDP at: " << config.local_rsi_host
            << ":" << config.local_rsi_port;
  udp_server_ = std::make_unique<icon::UdpServer>(config.local_rsi_host,
                                                  config.local_rsi_port);
  INTR_RETURN_IF_ERROR(udp_server_->Connect());

  RSICommand command;
  command.digital_output.resize(config.num_digital_outputs);
  INTR_RETURN_IF_ERROR(rsi_xml_.AssembleCommandRT(command, 1234, out_buffer_));
  LOG(INFO) << "A typical RSI Command XML that will be send to the robot:\n"
            << absl::string_view(out_buffer_.data(), out_buffer_.size());

  return absl::OkStatus();
}

void KukaRsiCommunicator::RsiCommunicationThreadJob(absl::Notification& ready) {
  INTRINSIC_RT_LOG(INFO) << "Kuka RSI communication thread running on CPU #"
                         << sched_getcpu();
  ready.Notify();

  auto iterations = RsiCommunicationThreadLoop();
  if (!iterations.ok()) {
    INTRINSIC_RT_LOG(WARNING)
        << "The Kuka RSI reader thread ended with an error: "
        << iterations.status().message();
  }
  // Report error until the module gets shutdown.
  while (!this->cancel_rsi_communication_thread_) {
    auto start_time = absl::Now();
    if (module_state_ == ModuleState::kCriticalError && !iterations.ok()) {
      *report_fault_.GetFreeBuffer() =
          icon::InternalError(icon::RealtimeStatus::StrCat(
              "RSI comm thread failed: ", iterations.status().message()));
      report_fault_.CommitFreeBuffer();
    }
    if (realtime_clock_) {
      (void)TickClock();
    }
    absl::SleepFor(kCycleDuration - (absl::Now() - start_time));
  }

  INTRINSIC_RT_LOG(INFO)
      << "The Kuka RSI reader thread ended its main loop. Ran for "
      << (iterations.ok() ? iterations.value() : 0) << " iterations.";
  module_state_ = ModuleState::kShutdown;
}

icon::RealtimeStatusOr<size_t>
KukaRsiCommunicator::RsiCommunicationThreadLoop() {
  size_t iterations = 0;
  RsiTelemetry telemetry;
  icon::RealtimeStatus status;
  while (!this->cancel_rsi_communication_thread_) {
    iterations++;
    icon::RealtimeStatusOr<RsiTelemetry> new_telemetry = ReadRsiTelemetry();
    if (!new_telemetry.ok()) {
      if (status = HandleDisconnectedRSIState(new_telemetry.status());
          !status.ok()) {
        module_state_ = ModuleState::kCriticalError;
        break;
      }
      continue;
    }
    time_last_telemetry_ = absl::Now();
    CheckTelemetryData(telemetry, *new_telemetry);
    telemetry = new_telemetry.value();

    *telemetry_.GetFreeBuffer() = telemetry;
    if (!telemetry_.CommitFreeBuffer()) {
      INTRINSIC_RT_LOG_THROTTLED(WARNING)
          << "Failed to commit RSI telemetry buffer.";
    }

    if (!rsi_active_) {
      remaining_warm_up_cycles_ = kWarmUpCycleCount;
      rsi_active_ = true;
      // RSI became active -> Store the current position as offset for future
      // commands.
      rsi_position_offset_ = telemetry.position;
      absl::c_fill(rsi_command_.position, 0.0);
      absl::string_view buffer_view(in_buffer_.data(), in_buffer_.size());
      // Print the XML buffer for easier debugging of RSI configuration issues.
      if (!has_received_telemetry_) {
        INTRINSIC_RT_LOG(INFO)
            << "Received first RSI Telemetry data: " << buffer_view;
        has_received_telemetry_ = true;
      } else {
        INTRINSIC_RT_LOG(INFO) << "Received first RSI Telemetry data since the "
                                  "stream was interrupted: "
                               << buffer_view;
      }
    } else if (remaining_warm_up_cycles_ > 0) {
      remaining_warm_up_cycles_--;
    }
    if (realtime_clock_ && absl::c_find(kClockTickingStates, module_state_) !=
                               kClockTickingStates.end()) {
      if (status = TickClock(); !status.ok()) {
        module_state_ = ModuleState::kCriticalError;
        break;
      }
    }

    PrepareAndSendRSICommand(telemetry.interpolator_counter_ms);
  }
  if (!status.ok()) {
    return status;
  }
  return iterations;
}

void KukaRsiCommunicator::CheckTelemetryData(
    const RsiTelemetry& telemetry, const RsiTelemetry& new_telemetry) {
  if (new_telemetry.digital_input.size() != config_.num_digital_inputs) {
    *report_fault_.GetFreeBuffer() =
        icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
            "The number of received inputs (",
            new_telemetry.digital_input.size(),
            ") does not match the number of configured digital inputs (",
            config_.num_digital_inputs, ")."));
    report_fault_.CommitFreeBuffer();
  }
  if ((telemetry.delayed_packets != new_telemetry.delayed_packets)) {
    INTRINSIC_RT_LOG(WARNING)
        << "The number of delayed packets reported by the RSI changed from "
        << telemetry.delayed_packets << " to " << new_telemetry.delayed_packets;
  }
  if (telemetry.rsi_correction_active != new_telemetry.rsi_correction_active) {
    INTRINSIC_RT_LOG(WARNING)
        << "The `RSI correction active` state changed from "
        << telemetry.rsi_correction_active << " to "
        << new_telemetry.rsi_correction_active;
  }
  if (new_telemetry.rsi_correction_active) {
    consecuctive_inactive_rsi_correction_cycles_ = 0;
  } else {
    consecuctive_inactive_rsi_correction_cycles_++;
  }
}

icon::RealtimeStatus KukaRsiCommunicator::HandleDisconnectedRSIState(
    const icon::RealtimeStatus& status) {
  if (time_last_telemetry_.has_value()) {
    absl::Duration t_elapsed = absl::Now() - *time_last_telemetry_;

    if (rsi_active_ && t_elapsed > config_.rsi_timeout) {
      INTRINSIC_RT_LOG(WARNING) << "No telemetry was received for " << t_elapsed
                                << ". RSI is not active anymore.";
      rsi_active_ = false;
      INTRINSIC_RT_RETURN_IF_ERROR(SwitchModuleState(
          KukaRsiCommunicator::ModuleState::kTickClockMotionDisabled));
    } else {
      INTRINSIC_RT_LOG_THROTTLED(INFO)
          << "Time since we last received RSI telemetry data: "
          << absl::ToInt64Seconds(t_elapsed) << "s";
    }
  }
  if (!icon::IsUnavailable(status)) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Error reading state via RSI: " << status.message();
  }
  // We need to tick ICON even if we do not receive data currently so that
  // we can recover later.
  if (realtime_clock_ && absl::c_find(kClockTickingStates, module_state_) !=
                             kClockTickingStates.end()) {
    INTRINSIC_RT_RETURN_IF_ERROR(TickClock());
  }
  return icon::OkStatus();
}

void KukaRsiCommunicator::PrepareAndSendRSICommand(
    uint64_t interpolator_counter_ms) {
  if (module_state_ ==
      KukaRsiCommunicator::ModuleState::kTickClockMotionEnabled) {
    CyclicCommands* received_cyclic_commands;
    if (received_cyclic_commands_.GetActiveBuffer(&received_cyclic_commands)) {
      for (int i = 0; i < received_cyclic_commands->position.size(); ++i) {
        rsi_command_.position[i] =
            (received_cyclic_commands->position)[i] - rsi_position_offset_[i];
      }
      rsi_command_.digital_output = received_cyclic_commands->digital_output;
    } else {
      INTRINSIC_RT_LOG_THROTTLED(WARNING)
          << "No cyclic commands were received from ICON since the last cycle. "
             "This means ApplyCommands()/SendCommands() was not called since "
             "the last cycle.";
    }
  }
  {
    icon::RealtimeStatus status = rsi_xml_.AssembleCommandRT(
        rsi_command_, interpolator_counter_ms, out_buffer_);

    if (!status.ok()) {
      INTRINSIC_RT_LOG_THROTTLED(INFO)
          << "Failed to generate XML for RSI command: " << status.message();
    }
  }
  if (drop_next_command_) {
    INTRINSIC_RT_LOG_THROTTLED(WARNING)
        << "Dropping the next RSI command because the previous one was sent "
           "too late. This could lead to a jerky motion or to a torque limit "
           "violation.";
    drop_next_command_ = false;
    return;
  }
  if (auto res = udp_server_->Send(absl::Span<uint8_t>(
          reinterpret_cast<uint8_t*>(out_buffer_.data()), out_buffer_.size()));
      !res.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "There was an issue sending an RSI command: "
        << icon::StrError(errno);
  }
}

icon::RealtimeStatus KukaRsiCommunicator::TickClock() {
  if (!realtime_clock_) {
    return icon::InternalError(
        "Realtime clock pointer is null; cannot tick the clock. This is a "
        "bug.");
  }
  auto start = absl::Now();
  auto deadline = start + kCycleDuration + kToleratedIconLateness;
  // TODO(b/309933583): Use KUKA's reported timestamp for ticking the realtime
  // clock
  if (icon::RealtimeStatus status = realtime_clock_->TickBlockingWithDeadline(
          intrinsic::Clock::now(), deadline);
      !status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(WARNING)
        << "Clock ticking returned a fault: " << status.message();
    // This is only interesting for users when the robot is actually enabled.
    if (module_state_ ==
        KukaRsiCommunicator::ModuleState::kTickClockMotionEnabled) {
      *report_fault_.GetFreeBuffer() = status;
      report_fault_.CommitFreeBuffer();
      if (icon::RealtimeStatus status = SwitchModuleState(
              KukaRsiCommunicator::ModuleState::kTickClockMotionDisabled);
          !status.ok()) {
        return status;
      }
    }
    if (icon::RealtimeStatus status =
            realtime_clock_->Reset(config_.rsi_timeout);
        !status.ok()) {
      return icon::InternalError(icon::RealtimeStatus::StrCat(
          "Resetting the realtime clock failed: ", status.message()));
    }
  }
  const auto kSoftTimeout = kCycleDuration - kCycleReplyTimeBuffer;
  const auto kSoftDeadline = start + kSoftTimeout;
  const auto after_tick_clock_time = absl::Now();
  if (after_tick_clock_time > kSoftDeadline) {
    INTRINSIC_RT_LOG_THROTTLED(WARNING)
        << "Clock ticking took " << after_tick_clock_time - start
        << ". Soft timeout is " << kSoftTimeout
        << ". Dropping the next RSI command. ";
    drop_next_command_ = true;
  }
  return icon::OkStatus();
}

icon::RealtimeStatusOr<RsiTelemetry> KukaRsiCommunicator::ReadRsiTelemetry() {
  auto start = absl::Now();
  // Some container=types fill the container with zeros when increasing the
  // size. So, increase it first to maximum (should be O(1)), then the shrink
  // the size afterwards again.
  in_buffer_.resize(in_buffer_.capacity());
  auto num_bytes_received = udp_server_->Receive(
      absl::Span<uint8_t>(reinterpret_cast<uint8_t*>(in_buffer_.data()),
                          in_buffer_.capacity()),
      /*timeout=*/config_.rsi_timeout);
  if (!num_bytes_received.ok()) {
    in_buffer_.resize(0);
    if (icon::IsDeadlineExceeded(num_bytes_received.status())) {
      return icon::UnavailableError("No data received via RSI.");
    }
    return icon::InternalError(
        icon::RealtimeStatus::StrCat("Error reading data from the RSI socket: ",
                                     num_bytes_received.status().message()));
  }
  in_buffer_.resize(*num_bytes_received);
  // Try to read additional packets that might have arrived, but do not wait if
  // there are none. If there are additional packets, only use the newest.
  size_t additional_packets = 0;
  do {
    in_buffer_additional_packets_.resize(
        in_buffer_additional_packets_.capacity());
    num_bytes_received = udp_server_->Receive(
        absl::Span<uint8_t>(
            reinterpret_cast<uint8_t*>(in_buffer_additional_packets_.data()),
            in_buffer_additional_packets_.capacity()),
        /*timeout=*/absl::ZeroDuration());
    if (!num_bytes_received.ok()) {
      in_buffer_additional_packets_.resize(0);
      break;
    }
    if (*num_bytes_received > in_buffer_additional_packets_.capacity()) {
      return icon::InternalError(icon::RealtimeStatus::StrCat(
          "Received larger RSI packet than memory available. Received: ",
          *num_bytes_received,
          ",  Capacity: ", in_buffer_additional_packets_.capacity()));
    }
    in_buffer_additional_packets_.resize(
        std::max<size_t>(0, *num_bytes_received));
    additional_packets++;
    in_buffer_ = in_buffer_additional_packets_;

    if (absl::Now() - start > kWaitForTelemetryTimeout) {
      return icon::DeadlineExceededError(icon::RealtimeStatus::StrCat(
          "Read ", additional_packets + 1, " new packages in ",
          kWaitForTelemetryTimeout,
          ". The sender sends too fast or this reader reads too slow."));
    }
  } while (true);
  absl::string_view buffer_view(in_buffer_.data(), in_buffer_.size());
  if (additional_packets > 0) {
    INTRINSIC_RT_LOG_THROTTLED(INFO)
        << "Received additional " << additional_packets
        << " packets. Using newest (size: " << in_buffer_.size()
        << "): " << buffer_view;
  }

  RsiTelemetry telemetry;
  icon::RealtimeStatus status =
      rsi_xml_.ParseTelemetryRT(buffer_view, telemetry);
  if (!status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Error parsing RSI telemetry: " << status.message()
        << "\nFrom the following XML with size " << buffer_view.size() << ": '"
        << buffer_view << "'";
    return status;
  }

  return telemetry;
}

RsiTelemetry KukaRsiCommunicator::GetTelemetry() const {
  RsiTelemetry* telemetry;
  (void)telemetry_.GetActiveBuffer(
      &telemetry);  // Always return the most recent. We do not care if it is
                    // new or old.
  return *telemetry;
}

bool KukaRsiCommunicator::IsActive() const {
  const uint64_t consecuctive_inactive_rsi_correction_cycles =
      consecuctive_inactive_rsi_correction_cycles_;
  const bool rsi_active = rsi_active_;
  // The reported `rsi_correction_active` telemetry flag can be flaky during RSI
  // startup. We relax the check to tolerate brief drops rather than faulting
  // immediately upon a single inactive correction cycle.
  const bool within_fault_tolerance =
      consecuctive_inactive_rsi_correction_cycles <
      kMaxInactiveRsiCorrectionCyclesUntilFault;
  // Stop logging the warning once `IsActive` returns false to prevent log spam
  // when the robot remains idle with an active UDP connection (rsi_active_).
  if (rsi_active && within_fault_tolerance &&
      consecuctive_inactive_rsi_correction_cycles > 0) {
    INTRINSIC_RT_LOG_THROTTLED(WARNING)
        << "`inactive RSI correction` though RSI "
           "is active: consecutive cycles "
        << consecuctive_inactive_rsi_correction_cycles;
  }
  return rsi_active && within_fault_tolerance && remaining_warm_up_cycles_ <= 0;
}

icon::RealtimeStatus KukaRsiCommunicator::SetCommand(
    absl::Span<const double> joint_positions,
    absl::Span<const bool> digital_outputs) {
  auto failure_mode_cleanup = absl::MakeCleanup([this] {
    // Things to do in every failure case.
    if (auto status = SwitchModuleState(ModuleState::kTickClockMotionDisabled);
        !status.ok()) {
      INTRINSIC_RT_LOG_THROTTLED(ERROR)
          << "Failed to switch to kTickClockMotionDisabled: "
          << status.message();
    }
  });

  icon::RealtimeStatus* fault;
  if (report_fault_.GetActiveBuffer(&fault) && !fault->ok()) {
    // Report any fault that was encountered in the RSI communication thread.
    return *fault;
  }

  std::array<double, kuka::kKukaNumJoints> rsi_position_cmd;
  if (joint_positions.size() != rsi_position_cmd.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Wrong number of joint position commands provided: ",
        joint_positions.size(), " Expected: ", rsi_position_cmd.size()));
  }

  if (digital_outputs.size() != config_.num_digital_outputs) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Wrong number of digital output commands provided: ",
        digital_outputs.size(), " Expected: ", config_.num_digital_outputs));
  }

  for (int i = 0; i < joint_positions.size(); ++i) {
    rsi_position_cmd[i] = joint_positions[i];
  }

  FixedVector<bool, kuka::kKukaMaxDigitalSignals> rsi_digital_output_cmd;
  for (int i = 0; i < digital_outputs.size(); ++i) {
    rsi_digital_output_cmd.push_back(digital_outputs[i]);
  }

  // Provide the new RSI command non-blockingly to the RSI communication thread.
  *received_cyclic_commands_.GetFreeBuffer() = CyclicCommands{
      .position = rsi_position_cmd, .digital_output = rsi_digital_output_cmd};
  received_cyclic_commands_.CommitFreeBuffer();

  // All good. Cancel failure mode cleanup.
  std::move(failure_mode_cleanup).Cancel();
  return intrinsic::icon::OkStatus();
}

absl::Status KukaRsiCommunicator::Shutdown() {
  LOG(INFO) << "Shutting down KukaRsiCommunicator";
  cancel_rsi_communication_thread_ = true;
  if (rsi_communication_thread_.joinable()) {
    rsi_communication_thread_.join();
  }
  udp_server_.reset();
  return absl::OkStatus();
}

icon::RealtimeStatus KukaRsiCommunicator::Deactivate() {
  cancel_rsi_communication_thread_ = true;

  return SwitchModuleState(ModuleState::kDeactivated);
}

icon::RealtimeStatus KukaRsiCommunicator::SwitchModuleState(
    ModuleState new_state) {
  if (new_state == module_state_) return icon::OkStatus();

  auto it = allowed_module_state_transitions_.find(module_state_);
  if (it == allowed_module_state_transitions_.end() ||
      it->second.find(new_state) == it->second.end()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "RSI Comm. state: Cannot switch from ", ToString(module_state_), " to ",
        ToString(new_state)));
  }

  module_state_ = new_state;
  return icon::OkStatus();
}

absl::string_view KukaRsiCommunicator::ToString(ModuleState module_state) {
  switch (module_state) {
    case ModuleState::kNone:
      return "kNone";
    case ModuleState::kInitialized:
      return "kInitialized";
    case ModuleState::kTickClockMotionDisabled:
      return "kTickClockMotionDisabled";
    case ModuleState::kTickClockMotionEnabled:
      return "kTickClockMotionEnabled";
    case ModuleState::kDeactivated:
      return "kDeactivated";
    case ModuleState::kShutdown:
      return "kShutdown";
    case ModuleState::kCriticalError:
      return "kCriticalError";
    default:
      return "<Unknown>";
  }
}

absl::Status KukaRsiCommunicator::StopRsiCommunicationThread() {
  cancel_rsi_communication_thread_ = true;
  if (rsi_communication_thread_.joinable()) {
    rsi_communication_thread_.join();
    if (realtime_clock_) {
      INTR_RETURN_IF_ERROR(realtime_clock_->Reset(absl::Seconds(10)));
    }
  }
  return absl::OkStatus();
}

absl::Status KukaRsiCommunicator::Prepare() {
  INTR_RETURN_IF_ERROR(StopRsiCommunicationThread());
  LOG(INFO) << "Starting the Kuka RSI communication thread.";
  absl::Notification reader_thread_running;
  rsi_communication_thread_options_.SetName("rsi_comm");
  rsi_communication_thread_ = Thread();
  cancel_rsi_communication_thread_ = false;
  // Clear lingering faults.
  *report_fault_.GetFreeBuffer() = icon::OkStatus();
  report_fault_.CommitFreeBuffer();
  INTRINSIC_RT_RETURN_IF_ERROR(
      SwitchModuleState(KukaRsiCommunicator::ModuleState::kInitialized));
  INTR_ASSIGN_OR_RETURN(
      rsi_communication_thread_,
      CreateRealtimeCapableThread(
          rsi_communication_thread_options_, [this, &reader_thread_running]() {
            RsiCommunicationThreadJob(reader_thread_running);
          }));

  // Wait until the thread is running.
  if (!reader_thread_running.WaitForNotificationWithTimeout(
          kThreadStartupTimeout)) {
    return absl::DeadlineExceededError(
        absl::StrCat("Starting RSI communication thread timed out after ",
                     absl::FormatDuration(kThreadStartupTimeout)));
  }

  return absl::OkStatus();
}

void KukaRsiCommunicator::Enable() {
  if (auto status = SwitchModuleState(
          KukaRsiCommunicator::ModuleState::kTickClockMotionEnabled);
      !status.ok()) {
    module_state_ = ModuleState::kCriticalError;
    return;
  }
  INTRINSIC_RT_LOG(INFO) << "RSI HWM enabled - will send new commands to robot";
}

void KukaRsiCommunicator::Disable() {
  if (auto status = SwitchModuleState(
          KukaRsiCommunicator::ModuleState::kTickClockMotionDisabled);
      !status.ok()) {
    module_state_ = ModuleState::kCriticalError;
    return;
  }
  INTRINSIC_RT_LOG(INFO)
      << "RSI HWM disabled - will continuously send last command to robot";
}

}  // namespace intrinsic::kuka
