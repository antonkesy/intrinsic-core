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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_loopback.h"

#include <cmath>
#include <cstdint>
#include <optional>

#include "absl/cleanup/cleanup.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/xml/realtime_xml_generator.h"
#include "intrinsic/icon/utils/xml/realtime_xml_parser.h"
#include "intrinsic/math/units.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kuka {
using icon::OkStatus;
using icon::RealtimeStatus;

RealtimeStatus RsiLoopbackXml::ParseCommand(absl::Span<const uint8_t> buffer,
                                            RSICommand& command) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto robot_node,
      RealtimeXmlParserElement::ParseDoc(
          absl::string_view(reinterpret_cast<const char*>(buffer.data()),
                            buffer.size()),
          "Sen"));

  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto position_node,
                                  robot_node.FirstElement("AK"));
    for (int i = 0; i < command.position.size(); i++) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          command.position[i],
          position_node.AttributeAsDouble(RealtimeStatus::StrCat("A", i + 1)));
      command.position[i] = DegToRad(command.position[i]);
    }
  }
  {
    {
      INTRINSIC_RT_ASSIGN_OR_RETURN(auto digital_output_node,
                                    robot_node.FirstElement("DigOut"));
      command.digital_output.clear();
      int i = 0;
      // We do not know how many digital outputs there are, so we just read
      // attributes until we do not find the next attribute.
      while (true) {
        auto value = digital_output_node.AttributeAsInteger<int>(
            RealtimeStatus::StrCat("DO", i + 1), false);

        if (icon::IsNotFound(value.status())) {
          break;
        } else if (!value.ok()) {
          return icon::InternalError(
              RealtimeStatus::StrCat("Error when trying to read digital "
                                     "output XML field  %s",
                                     value.status().message()));
        }
        if (command.digital_output.size() ==
            command.digital_output.capacity()) {
          return icon::FailedPreconditionError(absl::StrFormat(
              "Received more digital outputs than allocated capacity (%i).",
              command.digital_output.capacity()));
        }
        command.digital_output.push_back(*value);
        ++i;
      }
    }
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto stop_node,
                                robot_node.FirstElement("STOP"));
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto stop, stop_node.TextAsInteger<uint64_t>());
  command.exit_rsi = stop != 0;

  return OkStatus();
}

RealtimeStatus RsiLoopbackXml::AssembleTelemetry(
    const RsiTelemetry& telemetry,
    FixedVector<uint8_t, kTelemetryBufferSize>& buffer) {
  xml_generator_.Clear();
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto* root_element,
                                xml_generator_.AddRootElement("Rob"));
  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto* position_element,
                                  root_element->AddElement("Pos"));
    for (int i = 0; i < telemetry.position.size(); i++) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          auto attribute, position_element->AddAttribute(joint_axis_names_[i]));
      INTRINSIC_RT_RETURN_IF_ERROR(
          attribute->SetValue(RadToDeg(telemetry.position[i])));
    }
  }
  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto* torque_element,
                                  root_element->AddElement("Trq"));
    for (int i = 0; i < telemetry.torque.size(); i++) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          auto* attribute,
          torque_element->AddAttribute(joint_axis_names_.at(i)));
      INTRINSIC_RT_RETURN_IF_ERROR(attribute->SetValue(telemetry.torque[i]));
    }
  }
  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto* digital_in_element,
                                  root_element->AddElement("DigIn"));
    for (int i = 0; i < telemetry.digital_input.size(); i++) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          auto* attribute,
          digital_in_element->AddAttribute(digital_inputs_.at(i)));
      INTRINSIC_RT_RETURN_IF_ERROR(
          attribute->SetValue(telemetry.digital_input[i] ? "1" : "0"));
    }
  }
  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto* digital_out_element,
                                  root_element->AddElement("DigOut"));
    for (int i = 0; i < telemetry.digital_output.size(); i++) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          auto* attribute,
          digital_out_element->AddAttribute(digital_outputs_.at(i)));
      INTRINSIC_RT_RETURN_IF_ERROR(
          attribute->SetValue(telemetry.digital_output[i] ? "1" : "0"));
    }
  }
  INTRINSIC_RT_RETURN_IF_ERROR(
      root_element
          ->AddElementWithText("ModeOp", static_cast<int>(telemetry.op_mode))
          .status());
  INTRINSIC_RT_RETURN_IF_ERROR(
      root_element
          ->AddElementWithText("IPOC", telemetry.interpolator_counter_ms)
          .status());
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto* delay_element,
                                root_element->AddElement("Delay"));
  INTRINSIC_RT_RETURN_IF_ERROR(
      delay_element->AddAttributeWithValue("D", telemetry.delayed_packets));

  buffer.resize(buffer.capacity());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto size,
      xml_generator_.GenerateXMLString(absl::MakeSpan(
          reinterpret_cast<char*>(buffer.data()), buffer.capacity())));
  buffer.resize(size);
  return OkStatus();
}

absl::Status KukaRsiLoopback::Prepare() {
  INTR_RETURN_IF_ERROR(udp_client_.Connect());
  return absl::OkStatus();
}
icon::RealtimeStatus KukaRsiLoopback::Run() {
  auto cleanup = absl::MakeCleanup([this] { is_shutdown_ = true; });
  // Do not start at zero. Zero means invalid data for the other side.
  uint64_t synthetic_timestamp_ms = 1;
  if (cycle_time_ <= absl::ZeroDuration() ||
      cycle_time_ == absl::InfiniteDuration()) {
    return icon::FailedPreconditionError(RealtimeStatus::StrCat(
        "Invalid frequency given: ", 1.0 / absl::ToDoubleSeconds(cycle_time_),
        ". Must be > 0."));
  }
  while (!shutdown_requested_) {
    auto cycle_start = absl::Now();
    auto cycle_end = cycle_start + cycle_time_;
    INTRINSIC_RT_RETURN_IF_ERROR(RunCycle(synthetic_timestamp_ms, cycle_end));
    synthetic_timestamp_ms += absl::ToInt64Milliseconds(cycle_time_);
    absl::SleepFor(cycle_end - absl::Now());
  }
  return OkStatus();
}

icon::RealtimeStatus KukaRsiLoopback::RunCycle(
    uint64_t cycle_start_timestamp_ms, absl::Time cycle_deadline) {
  FixedVector<uint8_t, kTelemetryBufferSize> buf;

  RsiTelemetry telemetry;
  {
    absl::MutexLock lock(mutex_);
    if (last_command_.has_value()) {
      telemetry.position = last_command_->position;
    } else {
      telemetry.position.fill(0);
    }
  }
  // Sending some sinusoidal torque data so that it is not flat zero.
  for (int i = 0; i < telemetry.torque.size(); i++) {
    telemetry.torque[i] =
        sin(static_cast<double>(cycle_start_timestamp_ms) * 1e-3);
  }
  telemetry.interpolator_counter_ms = cycle_start_timestamp_ms;
  telemetry.op_mode = op_mode_;
  telemetry.delayed_packets = 0;

  {
    absl::MutexLock lock(mutex_);
    telemetry.digital_input = digital_input_command_;
    if (last_command_.has_value()) {
      telemetry.digital_output = last_command_->digital_output;
    }
  }
  INTRINSIC_RT_RETURN_IF_ERROR(rsi_xml_.AssembleTelemetry(telemetry, buf));
  INTRINSIC_RT_RETURN_IF_ERROR(udp_client_.Send(buf).status());
  const auto received_or_status =
      udp_client_.Receive(absl::MakeSpan(buf), cycle_deadline - absl::Now());
  if (!received_or_status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(INFO)
        << "Did not receive a new command in time.";
    return icon::OkStatus();
  }
  RSICommand command;
  INTRINSIC_RT_RETURN_IF_ERROR(rsi_xml_.ParseCommand(buf, command));
  {
    absl::MutexLock lock(mutex_);
    if (last_command_.has_value() &&
        last_command_->digital_output != command.digital_output) {
      INTRINSIC_RT_LOG_THROTTLED(INFO)
          << "Digital output changed to: "
          << absl::StrJoin(command.digital_output, ", ");
    }
    last_command_ = command;
  }
  return OkStatus();
}

void KukaRsiLoopback::SetDigitalInputs(absl::Span<const bool> digital_inputs) {
  absl::MutexLock lock(mutex_);
  digital_input_command_ = FixedVector<bool, kKukaMaxDigitalSignals>(
      digital_inputs.begin(), digital_inputs.end());
}

std::optional<RSICommand> KukaRsiLoopback::LatestCommand() const {
  absl::MutexLock lock(mutex_);
  if (last_command_) {
    return last_command_;
  }
  return std::nullopt;
}
}  // namespace intrinsic::kuka
