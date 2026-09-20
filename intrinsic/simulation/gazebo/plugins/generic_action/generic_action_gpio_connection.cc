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

#include "intrinsic/simulation/gazebo/plugins/generic_action/generic_action_gpio_connection.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "gloop/util/gtl/iterator_adaptors.h"
#include "intrinsic/hardware/gpio/v1/signal.pb.h"

namespace intrinsic {
namespace simulation {

GPIOGenericActionPluginConnection::GPIOGenericActionPluginConnection(
    absl::string_view plugin_handle)
    : plugin_handle_(plugin_handle) {}

absl::flat_hash_map<std::string, intrinsic_proto::gpio::v1::SignalDescription>
GPIOGenericActionPluginConnection::GetSignalDescriptions() const {
  return signal_descriptions_;
}

absl::Status GPIOGenericActionPluginConnection::CheckSignalsWritable(
    const absl::flat_hash_set<std::string>& signal_names) {
  if (set_command_cbs_.contains(signal_names)) {
    return absl::OkStatus();
  }
  for (const auto& signals : gtl::key_view(values_to_command_cb_)) {
    if (std::all_of(signal_names.cbegin(), signal_names.cend(),
                    [&signals](const std::string& n) {
                      return signals.values().contains(n);
                    })) {
      return absl::OkStatus();
    }
  }
  return absl::InvalidArgumentError(absl::Substitute(
      "Command signals $0 not found", absl::StrJoin(signal_names, ",")));
}

intrinsic_proto::gpio::v1::SignalValueSet
GPIOGenericActionPluginConnection::GetSignalValues() const {
  intrinsic_proto::gpio::v1::SignalValueSet signal_values;
  for (const auto& [signal_names, callback] : get_status_cbs_) {
    if (callback == nullptr) {
      LOG(INFO) << "Callback is null for " << absl::StrJoin(signal_names, ",");
      continue;
    }
    intrinsic_proto::gpio::v1::SignalValueSet values = callback();
    signal_values.MergeFrom(values);
  }
  return signal_values;
}

absl::StatusOr<intrinsic_proto::gpio::v1::SignalValueSet>
GPIOGenericActionPluginConnection::GetStatusValues(
    const absl::flat_hash_set<std::string>& signal_names) {
  if (auto cb = get_status_cbs_.find(signal_names);
      cb != get_status_cbs_.end()) {
    return cb->second();
  }
  return absl::InvalidArgumentError(absl::Substitute(
      "Status signals $0 not found", absl::StrJoin(signal_names, ",")));
}

absl::Status GPIOGenericActionPluginConnection::SetCommandValues(
    intrinsic_proto::gpio::v1::SignalValueSet command_values) {
  bool command_executed = false;
  if (values_to_command_cb_.contains(command_values)) {
    values_to_command_cb_.at(command_values)(command_values);
    command_executed = true;
  }

  absl::flat_hash_set<std::string> signal_names;
  for (const auto& [name, value] : command_values.values()) {
    signal_names.insert(name);
  }
  if (auto cb = set_command_cbs_.find(signal_names);
      cb != set_command_cbs_.end()) {
    cb->second(command_values);
    command_executed = true;
  }

  return command_executed ? absl::OkStatus()
                          : absl::NotFoundError(absl::Substitute(
                                "Command signals $0 not found",
                                absl::StrJoin(signal_names, ",")));
}

absl::Status GPIOGenericActionPluginConnection::RegisterSetCommandCallback(
    const std::vector<intrinsic_proto::gpio::v1::SignalDescription>&
        command_signal_descriptions,
    SetCommandCallback set_command) {
  if (set_command == nullptr) {
    return absl::InvalidArgumentError(absl::Substitute(
        "set_command cannot be null for command signal descriptions: $0",
        absl::StrJoin(command_signal_descriptions, ",")));
  }
  absl::flat_hash_set<std::string> signal_names;
  for (const auto& signal : command_signal_descriptions) {
    signal_names.insert(signal.signal_name());
    signal_descriptions_[signal.signal_name()] = signal;
  }
  if (set_command_cbs_.contains(signal_names)) {
    return absl::InvalidArgumentError(absl::Substitute(
        "Command signals $0 already exists", absl::StrJoin(signal_names, ",")));
  }
  set_command_cbs_[signal_names] = std::move(set_command);
  return absl::OkStatus();
}

absl::Status GPIOGenericActionPluginConnection::RegisterSetCommandCallback(
    intrinsic_proto::gpio::v1::SignalValueSet command_signal_values,
    SetCommandCallback set_command) {
  if (set_command == nullptr) {
    return absl::InvalidArgumentError(absl::Substitute(
        "set_command cannot be null for command signal values: $0",
        command_signal_values));
  }

  if (values_to_command_cb_.contains(command_signal_values)) {
    return absl::InvalidArgumentError(absl::Substitute(
        "Duplicate command signals: $0", command_signal_values));
  }
  values_to_command_cb_[command_signal_values] = std::move(set_command);
  return absl::OkStatus();
}

absl::Status GPIOGenericActionPluginConnection::RegisterGetStatusCallback(
    const absl::flat_hash_set<std::string>& status_signal_names,
    GetStatusCallback get_status) {
  if (get_status == nullptr) {
    return absl::InvalidArgumentError("get_status cannot be null");
  }
  if (get_status_cbs_.contains(status_signal_names)) {
    return absl::InvalidArgumentError(
        absl::Substitute("Status signals $0 already exists",
                         absl::StrJoin(status_signal_names, ",")));
  }

  get_status_cbs_[status_signal_names] = get_status;
  return absl::OkStatus();
}

}  // namespace simulation
}  // namespace intrinsic
