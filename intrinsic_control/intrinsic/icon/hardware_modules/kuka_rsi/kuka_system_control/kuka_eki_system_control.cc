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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_eki_system_control.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/base/thread_annotations.h"
#include "absl/cleanup/cleanup.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/tcp_server_client.h"
#include "intrinsic/icon/utils/xml/realtime_xml_generator.h"
#include "intrinsic/icon/utils/xml/realtime_xml_parser.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/invalid_until_set.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::kuka {

// Max allowed length of the EKI xml.
constexpr size_t kMaxEkiXmlLength = 4096;
// Connect timeout for the EKI TCP connection.
constexpr absl::Duration kConnectTimeout = absl::Seconds(2);
// Timeout for the EKI command to be processed by the KRC.
constexpr absl::Duration kEkiCommandTimeout = absl::Seconds(10);
// Factor used to multiply the target cycle time with to get the timeout for EKI
// calls.
constexpr double kEkiTimeoutFactor = 10.0;
// Length of the filter used to determine the average EKI cycle round trip time.
constexpr size_t kCycleTimeFilterLength = 50;
// Interval at which local variables are polled when waiting for a KRC response.
constexpr absl::Duration kPollingInterval = absl::Milliseconds(10);

KukaEkiSystemControl::KukaEkiSystemControl(
    const KukaEkiClientParams& params,
    std::unique_ptr<icon::TcpClient> tcp_client)
    : update_frequency_(params.update_frequency) {
  if (tcp_client != nullptr) {
    tcp_client_ = std::move(tcp_client);
  } else {
    tcp_client_ = std::make_unique<icon::TcpClient>(
        params.eki_address, params.eki_port, "EKI_Control");
  }
}

absl::StatusOr<std::unique_ptr<KukaEkiSystemControl>>
KukaEkiSystemControl::Create(const KukaEkiClientParams& params,
                             std::unique_ptr<icon::TcpClient> tcp_client) {
  std::unique_ptr<KukaEkiSystemControl> result(
      new KukaEkiSystemControl(params, std::move(tcp_client)));

  INTR_RETURN_IF_ERROR(result->Init());
  return result;
}

absl::Status KukaEkiSystemControl::Init() {
  INTR_RETURN_IF_ERROR(tcp_client_->Connect(kConnectTimeout));
  LOG(INFO) << "Connected to EKI, using update frequency of "
            << update_frequency_ << " Hz";
  intrinsic::ThreadOptions thread_options;
  thread_options.SetName("kuka_eki_ctrl");

  INTR_ASSIGN_OR_RETURN(communication_thread_,
                        CreateRealtimeCapableThread(thread_options, [this] {
                          EkiCommunicationThreadJob();
                        }));
  return absl::OkStatus();
}

KukaEkiSystemControl::~KukaEkiSystemControl() {
  shutdown_requested_ = true;
  if (communication_thread_.joinable()) {
    communication_thread_.join();
  }
}

void KukaEkiSystemControl::EkiCommunicationThreadJob() {
  absl::Duration target_cycle_duration = absl::Seconds(1.0 / update_frequency_);
  absl::Duration reconnect_interval = absl::Seconds(0.5);
  while (!this->shutdown_requested_) {
    auto start = absl::Now();
    // This cleanup lambda makes sures that we keep the desired cycle
    // time (also for all `continue` calls), if possible.
    auto cleanup_sleep =
        absl::MakeCleanup([this, target_cycle_duration, start] {
          absl::Duration elapsed_time = absl::Now() - start;
          absl::Duration time_remaining = target_cycle_duration - elapsed_time;
          reply_durations_.push_back(absl::ToDoubleSeconds(elapsed_time));
          // Keep the size of reply_durations at the maximum of
          // kCycleTimeFilterLength to realize a sliding window filter.
          if (reply_durations_.size() > kCycleTimeFilterLength) {
            reply_durations_.pop_front();
          }
          absl::SleepFor(time_remaining);
        });
    if (!HandleEKIConnection(reconnect_interval)) {
      continue;
    }
    if (!SendEkiCommand()) {
      continue;
    }
    ReceiveEkiTelemetry(target_cycle_duration * kEkiTimeoutFactor);
  }
}

absl::Status KukaEkiSystemControl::StartRSI(absl::Duration timeout)
    ABSL_LOCKS_EXCLUDED(mutex_) {
  LOG(INFO) << "Starting RSI";
  bool rsi_stop_required = false;
  {
    absl::MutexLock lock(mutex_);
    if (eki_telemetry_.op_mode != OpMode::kExternal) {
      return absl::UnavailableError(
          absl::StrCat("KUKA is not in 'External' mode. Current Mode: '",
                       OpModeToString(eki_telemetry_.op_mode), "'"));
    }
    rsi_stop_required =
        eki_telemetry_.program_active || eki_telemetry_.program_selected;
  }
  if (rsi_stop_required) {
    LOG(INFO) << "Program found - Stopping RSI";
    INTR_RETURN_IF_ERROR(StopRSI());
    // Give the KUKA KRC some time...
    absl::SleepFor(absl::Seconds(0.5));
    LOG(INFO) << "Program stopped - Starting RSI";
  }
  {
    EkiCommand eki_command;
    eki_command.operation = EkiOperation::kStartProgram;
    absl::MutexLock lock(mutex_);
    eki_command_queue_.push(eki_command);
  }
  absl::Time start = absl::Now();
  while ((absl::Now() - start) < timeout) {
    {
      absl::MutexLock lock(mutex_);
      if (eki_telemetry_.rsi_active && eki_telemetry_.program_active) {
        return absl::OkStatus();
      }
    }
    absl::SleepFor(kPollingInterval);
  }
  {
    absl::MutexLock lock(mutex_);
    if (eki_telemetry_.messages_present) {
      return absl::UnavailableError(
          "Cannot start RSI since some KUKA messages are still present. Some "
          "messages can only be cleared in T1 mode by the operator on the "
          "teach pendant.");
    }
  }
  absl::Status error =
      absl::DeadlineExceededError("Timed out waiting for RSI to start");
  LOG(WARNING) << error.message();
  return error;
}

absl::Status KukaEkiSystemControl::StopRSI() ABSL_LOCKS_EXCLUDED(mutex_) {
  // Stop the RSI EKI program first. And then cancel the program to get a clean
  // slate. Stopping first is required, since an active program cannot be
  // directly cancelled.
  LOG(INFO) << "Stopping RSI";
  {
    EkiCommand eki_command;
    eki_command.operation = EkiOperation::kStopProgram;
    absl::MutexLock lock(mutex_);
    eki_command_queue_.push(eki_command);
  }
  auto start = absl::Now();
  while ((absl::Now() - start) < kEkiCommandTimeout) {
    {
      bool program_active = false;
      {
        absl::MutexLock lock(mutex_);
        program_active = eki_telemetry_.program_active;
      }
      if (!program_active) {
        return CancelProgram();
      }
    }
    absl::SleepFor(kPollingInterval);
  }
  return absl::DeadlineExceededError("Timed out waiting for RSI to stop");
}

absl::Status KukaEkiSystemControl::CancelProgram() ABSL_LOCKS_EXCLUDED(mutex_) {
  {
    EkiCommand eki_command;
    eki_command.operation = EkiOperation::kCancelProgram;
    absl::MutexLock lock(mutex_);
    eki_command_queue_.push(eki_command);
  }
  auto start = absl::Now();
  while ((absl::Now() - start) < kEkiCommandTimeout) {
    {
      absl::MutexLock lock(mutex_);
      if (!eki_telemetry_
               .program_selected) {  // Canceling was successful when there is
                                     // no program selected anymore on the KRC.
        return absl::OkStatus();
      }
    }
    absl::SleepFor(kPollingInterval);
  }
  return absl::OkStatus();
}

absl::Status KukaEkiSystemControl::ClearFaults() {
  LOG(INFO) << "Clearing faults via EKI";
  {
    EkiCommand eki_command;
    eki_command.operation = EkiOperation::kClearFaults;
    absl::MutexLock lock(mutex_);
    eki_command_queue_.push(eki_command);
  }
  auto start = absl::Now();
  while ((absl::Now() - start) < kEkiCommandTimeout) {
    {
      absl::MutexLock lock(mutex_);
      if (!eki_telemetry_.messages_present) {
        return absl::OkStatus();
      }
    }
    absl::SleepFor(kPollingInterval);
  }
  {
    absl::MutexLock lock(mutex_);
    if (eki_telemetry_.op_mode == OpMode::kExternal &&
        eki_telemetry_.messages_present)
      return absl::DeadlineExceededError(
          "Could not clear faults since some KUKA messages are still present. "
          "Some messages can only be cleared in T1 mode by the operator on the "
          "teach pendant.");
  }
  return absl::OkStatus();
}

bool KukaEkiSystemControl::HandleEKIConnection(
    absl::Duration& reconnect_interval) {
  if (!tcp_client_->IsConnected()) {
    absl::SleepFor(reconnect_interval);
    if (auto status = tcp_client_->Connect(kConnectTimeout); !status.ok()) {
      LOG(ERROR) << status;
      if (reconnect_interval < absl::Seconds(10)) {
        reconnect_interval *= 2;
      }
    } else {
      reconnect_interval = absl::Seconds(0.5);
    }
    return false;
  }

  return true;
}

bool KukaEkiSystemControl::SendEkiCommand() {
  InvalidUntilSet<std::string> xml_command;
  // We have to send always an eki command, even if the queue is empty, to
  // receive the most current telemetry from the robot controller.
  EkiCommand eki_command;
  {
    absl::MutexLock lock(mutex_);
    if (!eki_command_queue_.empty()) {
      eki_command = eki_command_queue_.front();
      eki_command_queue_.pop();
    }
  }
  absl::StatusOr<std::string> xml = AssembleEkiXml(eki_command);
  if (!xml.ok()) {
    LOG_EVERY_N_SEC(ERROR, 5) << "Failed to assemble EKI XML: " << xml.status();
    return false;
  } else {
    xml_command = xml.value();
  }

  if (xml_command.has_value()) {
    if (icon::RealtimeStatus status = tcp_client_->Send(
            absl::Span<const uint8_t>(
                reinterpret_cast<const uint8_t*>(xml_command->data()),
                xml_command->size()),
            MSG_NOSIGNAL  // Avoid getting sig_nopipe
        );
        !status.ok()) {
      LOG_EVERY_N_SEC(ERROR, 5) << "Failed to send to EKI: " << status;
      tcp_client_->Disconnect();
      return false;
    }
  }

  return true;
}

void KukaEkiSystemControl::ReceiveEkiTelemetry(absl::Duration timeout) {
  FixedVector<uint8_t, kMaxEkiXmlLength> buffer;

  auto received = tcp_client_->ReceiveContainer(timeout, false, 0, buffer);
  if (!received.ok()) {
    tcp_client_->Disconnect();
    LOG_EVERY_N_SEC(ERROR, 5) << "Failed to receive EKI data: " << received;
  } else {
    auto telemetry = ParseEkiXml(absl::string_view(
        reinterpret_cast<const char*>(buffer.data()), buffer.size()));
    if (telemetry.ok()) {
      absl::MutexLock lock(mutex_);
      eki_telemetry_ = *telemetry;
    } else {
      LOG_EVERY_N_SEC(ERROR, 5)
          << "Failed to parse telemetry: " << telemetry.status();
    }
  }
}

absl::StatusOr<std::string> KukaEkiSystemControl::AssembleEkiXml(
    const EkiCommand& eki_command) const {
  RealtimeXmlGenerator generator(10, 10);
  INTRINSIC_RT_ASSIGN_OR_RETURN(RtXmlElement * root,
                                generator.AddRootElement("Intrinsic"));
  INTRINSIC_RT_RETURN_IF_ERROR(root->AddElement("Update").status());

  INTRINSIC_RT_RETURN_IF_ERROR(
      root->AddElementWithText("Operation",
                               EkiOperationToString(eki_command.operation))
          .status());

  std::string output;
  output.resize(kMaxEkiXmlLength);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      size_t size, generator.GenerateXMLString(
                       absl::Span<char>(output.data(), output.size())));
  output.resize(size);
  return output;
}

absl::StatusOr<KukaEkiSystemControl::EkiTelemetry>
KukaEkiSystemControl::ParseEkiXml(absl::string_view xml) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto root, RealtimeXmlParserElement::ParseDoc(xml, "Robot"));
  EkiTelemetry result;

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto op_mode, root.FirstElement("OpMode"));
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto op_mode_value,
                                op_mode.TextAsInteger<int32_t>());
  result.op_mode = static_cast<OpMode>(op_mode_value);

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto rsi_running,
                                root.FirstElement("RSIActive"));
  result.rsi_active = rsi_running.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto messages_present,
                                root.FirstElement("MessagesPresent"));
  result.messages_present = messages_present.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto program_active,
                                root.FirstElement("ProgramActive"));
  result.program_active = program_active.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto program_selected,
                                root.FirstElement("ProgramSelected"));
  result.program_selected = program_selected.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto selected_program_path,
                                root.FirstElement("SelectedProgramPath"));
  result.selected_program_path = absl::StripAsciiWhitespace(
      selected_program_path.Text());  // When no program is selected, KUKA will
                                      // report a whitespace.

  return result;
}

absl::Duration KukaEkiSystemControl::ComputeAverageReplyDuration() const {
  std::vector<double> durations(reply_durations_.begin(),
                                reply_durations_.end());
  return absl::Seconds(
      Eigen::Map<Eigen::VectorXd>(durations.data(), durations.size()).mean());
}

absl::StatusOr<KukaErrorFlagsMask> KukaEkiSystemControl::ActiveErrorFlags()
    const {
  return KukaErrorFlagsMask::kNone;
}

}  // namespace intrinsic::kuka
