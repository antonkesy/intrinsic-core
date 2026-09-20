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

#include "intrinsic/icon/control/rtcl_safety_message_handler.h"

#include <algorithm>
#include <string_view>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages_utils.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/adio.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/lib/fieldbus/v1/value_parsing.pb.h"
#include "intrinsic/icon/hal/lib/fieldbus/value_pattern_utils.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/bitset.h"

namespace intrinsic::icon {
namespace {

using ::intrinsic_fbs::ButtonStatus;
using ::intrinsic_fbs::ModeOfSafeOperation;
using ::intrinsic_fbs::RequestedBehavior;

// The most severe requested behavior is selected.
// The order is:
// SAFE_STOP_0 > SAFE_STOP_1_TIME_MONITORED > SAFE_STOP_2_TIME_MONITORED > PAUSE
// > NORMAL_OPERATION > UNKNOWN
RequestedBehavior CombineRequestedBehavior(const RequestedBehavior& a,
                                           const RequestedBehavior& b) {
  if (a == RequestedBehavior::SAFE_STOP_0 ||
      b == RequestedBehavior::SAFE_STOP_0) {
    return RequestedBehavior::SAFE_STOP_0;
  }
  if (a == RequestedBehavior::SAFE_STOP_1_TIME_MONITORED ||
      b == RequestedBehavior::SAFE_STOP_1_TIME_MONITORED) {
    return RequestedBehavior::SAFE_STOP_1_TIME_MONITORED;
  }
  if (a == RequestedBehavior::SAFE_STOP_2_TIME_MONITORED ||
      b == RequestedBehavior::SAFE_STOP_2_TIME_MONITORED) {
    return RequestedBehavior::SAFE_STOP_2_TIME_MONITORED;
  }
  if (a == RequestedBehavior::PAUSE || b == RequestedBehavior::PAUSE) {
    return RequestedBehavior::PAUSE;
  }
  return a > b ? a : b;
}

// The most restrictive mode of safe operation is selected.
// The order is:
// UPDATING > CONFIGURATION > TEACH_PENDANT_2 > TEACH_PENDANT_1 > AUTOMATIC >
// UNKNOWN
ModeOfSafeOperation CombineModeOfSafeOperation(const ModeOfSafeOperation& a,
                                               const ModeOfSafeOperation& b) {
  if (a == ModeOfSafeOperation::UPDATING ||
      b == ModeOfSafeOperation::UPDATING) {
    return ModeOfSafeOperation::UPDATING;
  }
  if (a == ModeOfSafeOperation::CONFIGURATION ||
      b == ModeOfSafeOperation::CONFIGURATION) {
    return ModeOfSafeOperation::CONFIGURATION;
  }
  if (a == ModeOfSafeOperation::TEACH_PENDANT_2 ||
      b == ModeOfSafeOperation::TEACH_PENDANT_2) {
    return ModeOfSafeOperation::TEACH_PENDANT_2;
  }
  if (a == ModeOfSafeOperation::TEACH_PENDANT_1 ||
      b == ModeOfSafeOperation::TEACH_PENDANT_1) {
    return ModeOfSafeOperation::TEACH_PENDANT_1;
  }
  if (a == ModeOfSafeOperation::AUTOMATIC ||
      b == ModeOfSafeOperation::AUTOMATIC) {
    return ModeOfSafeOperation::AUTOMATIC;
  }
  return ModeOfSafeOperation::UNKNOWN;
}

// The order is:
// ENGAGED > DISENGAGED > NOT_AVAILABLE > UNKNOWN
ButtonStatus CombineButtonStatus(const ButtonStatus& a, const ButtonStatus& b) {
  if (a == ButtonStatus::ENGAGED || b == ButtonStatus::ENGAGED) {
    return ButtonStatus::ENGAGED;
  }
  if (a == ButtonStatus::DISENGAGED || b == ButtonStatus::DISENGAGED) {
    return ButtonStatus::DISENGAGED;
  }
  if (a == ButtonStatus::NOT_AVAILABLE || b == ButtonStatus::NOT_AVAILABLE) {
    return ButtonStatus::NOT_AVAILABLE;
  }
  return ButtonStatus::UNKNOWN;
}

// The most severe e-stop button status is selected.
// The goal is no motion if any button is engaged.
ButtonStatus CombineEStopButtonStatus(const ButtonStatus& a,
                                      const ButtonStatus& b) {
  return CombineButtonStatus(a, b);
}

// Halt motion if any enable button is disengaged; enable motion only when
// engaged.
// Rationale: Multiple enabling devices in a workspace must follow a logical AND
// construction to permit motion. If any operator releases their device to the
// DISENGAGED position, it must dominate the state and halt motion.
// The order is:
// DISENGAGED > ENGAGED > NOT_AVAILABLE > UNKNOWN
ButtonStatus CombineEnableButtonStatus(const ButtonStatus& a,
                                       const ButtonStatus& b) {
  if (a == ButtonStatus::DISENGAGED || b == ButtonStatus::DISENGAGED) {
    return ButtonStatus::DISENGAGED;
  }
  if (a == ButtonStatus::ENGAGED || b == ButtonStatus::ENGAGED) {
    return ButtonStatus::ENGAGED;
  }
  if (a == ButtonStatus::NOT_AVAILABLE || b == ButtonStatus::NOT_AVAILABLE) {
    return ButtonStatus::NOT_AVAILABLE;
  }
  return ButtonStatus::UNKNOWN;
}

// Returns true if any pattern matches.
bool SignalIsActive(const SafetyMessageHandler::BehaviorOverrideRequestSignal&
                        override_signal) {
  if (*override_signal.digital_input_hardware_interface == nullptr) {
    return false;
  }
  auto* signals = override_signal.digital_input_hardware_interface->signals();
  if (signals == nullptr) {
    return false;
  }

  using SignalMaskType =
      decltype(std::declval<intrinsic_proto::fieldbus::v1::ValuePattern>()
                   .signal_mask());
  intrinsic::bitset<SignalMaskType> input_value = 0;
  // Ensures we stay within the bounds of the signals and bitset.
  const size_t safe_size =
      std::min(static_cast<size_t>(signals->size()), input_value.size());

  for (size_t i = 0; i < safe_size; ++i) {
    auto* signal = signals->Get(i);
    if (signal == nullptr) [[unlikely]] {
      continue;
    }
    input_value.set(i, signal->value());
  }
  return std::any_of(
      override_signal.match_patterns.begin(),
      override_signal.match_patterns.end(),
      [&input_value](
          const intrinsic_proto::fieldbus::v1::ValuePattern& pattern) {
        return pattern.expected_value() ==
               (intrinsic::GetValue<SignalMaskType>(input_value) &
                pattern.signal_mask());
      });
}

// Looks for the safety status message among hardware modules and bus devices.
// `safety_module_name` is the name of a hardware module or a bus device.
// `safety_interface_name` is the corresponding interface name (typically
// "safety_status_message" for bus devices).
// The function will first check for a hardware module with the desired
// interface and will return that if found. Otherwise, it will look for a bus
// device with the corresponding name and interface.
//
// Returns a `SafetyStatusMessage*` if the device or module provide the safety
// status.
// Returns InvalidArgumentError if `safety_module_name` is empty.
// Returns InvalidArgumentError if neither a hardware module nor a bus device
// could be found that provide the desired interface.
absl::StatusOr<HardwareInterfaceHandle<intrinsic_fbs::SafetyStatusMessage>>
FindSafetyStatusInterface(absl::string_view safety_module_name,
                          absl::string_view safety_interface_name,
                          const Context& context) {
  if (safety_module_name.empty()) {
    return absl::InvalidArgumentError("safety_module_name is empty");
  }

  // Check for available hardware module and interface, if that's not there,
  // look for a subordinate bus device.
  auto status_or_safety_status_interface =
      context.GetHardwareInterfaceHandle<intrinsic_fbs::SafetyStatusMessage>(
          safety_module_name, safety_interface_name);
  if (status_or_safety_status_interface.ok() &&
      **status_or_safety_status_interface != nullptr) {
    LOG(INFO) << "Subscribed to 'SafetyStatusMessage' of '"
              << safety_module_name << "' with interface '"
              << safety_interface_name << "'.";
    return status_or_safety_status_interface;
  }
  return absl::InvalidArgumentError(absl::StrCat(
      "Didn't find the hardware interface id '", safety_module_name,
      "' with interface '", safety_interface_name, "'."));
}

absl::StatusOr<SafetyMessageHandler::BehaviorOverrideRequestSignal>
ParseExpectedSignal(const intrinsic_proto::icon::ExpectedSignal& signal,
                    const Context& context) {
  if (const auto num_patterns = signal.match_patterns().size();
      num_patterns > SafetyMessageHandler::kMaxPatterns) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Too many expected_signals. Max: ", SafetyMessageHandler::kMaxPatterns,
        ", got: ", num_patterns));
  }

  std::string_view module_name =
      signal.digital_input_hardware_interface().module_name();
  std::string_view interface_name =
      signal.digital_input_hardware_interface().interface_name();

  INTR_ASSIGN_OR_RETURN(
      auto digital_input_interface,
      context.GetHardwareInterfaceHandle<intrinsic_fbs::DIOStatus>(
          module_name, interface_name));

  if (*digital_input_interface == nullptr) {
    return absl::InvalidArgumentError(
        absl::StrCat("Could not find digital input interface '", interface_name,
                     "' on module '", module_name, "'."));
  }

  auto* signals = digital_input_interface->signals();
  if (signals == nullptr) {
    return absl::InvalidArgumentError(
        absl::StrCat("Digital input interface '", interface_name,
                     "' on module '", module_name, "' has no signals."));
  }
  const size_t input_size_bits = signals->size();

  FixedVector<intrinsic_proto::fieldbus::v1::ValuePattern,
              SafetyMessageHandler::kMaxPatterns>
      match_patterns;
  for (const auto& pattern : signal.match_patterns()) {
    // Assumes bit indices are consecutive.
    INTR_RETURN_IF_ERROR(fieldbus::ValidateValuePattern(
        /*signal_mask=*/pattern.signal_mask(),
        /*expected_value=*/pattern.expected_value(),
        /*input_size_bits=*/input_size_bits));

    match_patterns.push_back(pattern);
  }

  return SafetyMessageHandler::BehaviorOverrideRequestSignal{
      .digital_input_hardware_interface = std::move(digital_input_interface),
      .match_patterns = std::move(match_patterns)};
}

}  // namespace

SafetyMessageHandler::SafetyMessageHandler(Config config)
    : config_(std::move(config)) {
  // Empty until Update is called.
  output_safety_status_buffer_ = intrinsic_fbs::BuildSafetyStatusMessage();
  output_safety_status_ =
      flatbuffers::GetMutableRoot<intrinsic_fbs::SafetyStatusMessage>(
          output_safety_status_buffer_.data());
  QCHECK(output_safety_status_ != nullptr)
      << "SafetyMessageHandler failed to create output_safety_status_. Please "
         "report a bug.";
}
// static
absl::StatusOr<std::unique_ptr<SafetyMessageHandler>>
SafetyMessageHandler::Create(
    const intrinsic_proto::icon::SafetyMessageHandlerConfig& config,
    const Context& context) {
  std::vector<HardwareInterfaceHandle<intrinsic_fbs::SafetyStatusMessage>>
      input_hardware_interfaces;

  for (const auto& conf : config.safety_hardware_interfaces()) {
    INTR_ASSIGN_OR_RETURN(
        auto interface,
        FindSafetyStatusInterface(
            /*safety_module_name=*/conf.module_name(),
            /*safety_interface_name=*/conf.interface_name(), context));

    input_hardware_interfaces.push_back(std::move(interface));
  }

  if (input_hardware_interfaces.empty()) {
    LOG(INFO) << "No safety module is configured. Not subscribing to a "
                 "'SafetyStatusMessage'.";
  } else {
    LOG(INFO) << "Subscribing to " << input_hardware_interfaces.size()
              << " 'SafetyStatusMessage's";
  }

  std::optional<SafetyMessageHandler::BehaviorOverrideRequestSignal>
      behavior_override_request_pause_signal;

  if (config.has_behavior_override_request_pause_signal()) {
    INTR_ASSIGN_OR_RETURN(
        behavior_override_request_pause_signal,
        ParseExpectedSignal(config.behavior_override_request_pause_signal(),
                            context));
    LOG(INFO) << "Configured with BehaviorOverrideRequestPauseSignal.";

  } else {
    LOG(INFO) << "No BehaviorOverrideRequestPauseSignal is defined.";
  }

  absl::FixedArray<HardwareInterfaceHandle<intrinsic_fbs::SafetyStatusMessage>>
      input_hardware_interfaces_array(
          std::make_move_iterator(input_hardware_interfaces.begin()),
          std::make_move_iterator(input_hardware_interfaces.end()));

  SafetyMessageHandler::Config safety_message_handler_config{
      .input_hardware_interfaces = std::move(input_hardware_interfaces_array),
      .behavior_override_request_pause_signal =
          std::move(behavior_override_request_pause_signal)};

  return absl::WrapUnique(
      new SafetyMessageHandler(std::move(safety_message_handler_config)));
}

RealtimeStatus SafetyMessageHandler::Update() {
  ModeOfSafeOperation mode_of_safe_operation = ModeOfSafeOperation::UNKNOWN;
  ButtonStatus estop_button_status = ButtonStatus::UNKNOWN;
  ButtonStatus enable_button_status = ButtonStatus::UNKNOWN;
  RequestedBehavior requested_behavior = RequestedBehavior::UNKNOWN;

  for (const auto& interface : config_.input_hardware_interfaces) {
    if (*interface == nullptr) [[unlikely]] {
      // This should be checked during Init()
      continue;
    }
    mode_of_safe_operation = CombineModeOfSafeOperation(
        mode_of_safe_operation, interface->mode_of_safe_operation());
    estop_button_status = CombineEStopButtonStatus(
        estop_button_status, interface->estop_button_status());
    enable_button_status = CombineEnableButtonStatus(
        enable_button_status, interface->enable_button_status());
    requested_behavior = CombineRequestedBehavior(
        requested_behavior, interface->requested_behavior());
  }

  // Inject PAUSE request from digital_input.
  // Once the hwm receives a safety signal, the requested_behavior most likely
  // is `SAFE_STOP_2_TIME_MONITORED` and will not be override by PAUSE.
  if (config_.behavior_override_request_pause_signal &&
      SignalIsActive(*config_.behavior_override_request_pause_signal)) {
    requested_behavior =
        CombineRequestedBehavior(requested_behavior, RequestedBehavior::PAUSE);
  }

  intrinsic_fbs::SetSafetyStatusMessage(
      mode_of_safe_operation, estop_button_status, enable_button_status,
      requested_behavior, *output_safety_status_);
  return OkStatus();
}

std::vector<intrinsic_proto::icon::v1::BehaviorOverrideRequest>
SafetyMessageHandler::GetConfiguredBehaviorOverrides() const {
  std::vector<intrinsic_proto::icon::v1::BehaviorOverrideRequest> overrides;
  // Currently only BEHAVIOR_OVERRIDE_REQUEST_PAUSE is supported and always
  // returned as it's the only one that could be potentially requested by
  // the SafetyMessageHandler.
  overrides.push_back(intrinsic_proto::icon::v1::BehaviorOverrideRequest::
                          BEHAVIOR_OVERRIDE_REQUEST_PAUSE);
  return overrides;
}

}  // namespace intrinsic::icon
