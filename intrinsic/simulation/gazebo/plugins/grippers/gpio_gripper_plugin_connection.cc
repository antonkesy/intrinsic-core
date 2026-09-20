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

#include "intrinsic/simulation/gazebo/plugins/grippers/gpio_gripper_plugin_connection.h"

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "intrinsic/hardware/gpio/gpio_service_proto_utils.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gpio/v1/signal.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_utils.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/gripper.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace simulation {

using ::intrinsic_proto::simulation::gazebo::GripperCommand;
using ::intrinsic_proto::simulation::gazebo::GripperStatus;

namespace {
GripperCommand GraspCommand() {
  GripperCommand command;
  command.set_command(GripperCommand::GRASP);
  return command;
}
GripperCommand ReleaseCommand() {
  GripperCommand command;
  command.set_command(GripperCommand::RELEASE);
  return command;
}
GripperCommand BlowoffOnCommand() {
  GripperCommand command;
  command.set_command(GripperCommand::BLOWOFF_ON);
  return command;
}
GripperCommand BlowoffOffCommand() {
  GripperCommand command;
  command.set_command(GripperCommand::BLOWOFF_OFF);
  return command;
}
}  // namespace

namespace gpio_gripper_plugin_connection_internal {
using intrinsic_proto::gpio::v1::SignalValue;

intrinsic_proto::gpio::v1::SignalValueSet GetNegatedValueSet(
    const intrinsic_proto::gpio::v1::SignalValueSet& original) {
  intrinsic_proto::gpio::v1::SignalValueSet negated = original;
  for (auto& [name, signal] : *negated.mutable_values()) {
    switch (signal.value_case()) {
      case intrinsic_proto::gpio::v1::SignalValue::kBoolValue: {
        signal.set_bool_value(!signal.bool_value());
        break;
      }
      case SignalValue::kUnsignedIntValue: {
        signal.set_unsigned_int_value(!signal.unsigned_int_value());
        break;
      }
      case SignalValue::kIntValue: {
        signal.set_int_value(!signal.int_value());
        break;
      }
      case SignalValue::kFloatValue: {
        signal.set_float_value(!signal.float_value());
        break;
      }
      case SignalValue::kDoubleValue: {
        signal.set_double_value(!signal.double_value());
        break;
      }
      case SignalValue::kInt8Value: {
        signal.mutable_int8_value()->set_value(!signal.int8_value().value());
        break;
      }
      case SignalValue::kUnsignedInt8Value: {
        signal.mutable_unsigned_int8_value()->set_value(
            !signal.unsigned_int8_value().value());
        break;
      }
      case SignalValue::VALUE_NOT_SET: {
        LOG(ERROR) << "Negating of unknown type of "
                      "intrinsic_proto::gpio::v1::SignalValue";
        break;
      }
    }
  }
  return negated;
}
}  // namespace gpio_gripper_plugin_connection_internal

using gpio_gripper_plugin_connection_internal::GetNegatedValueSet;

void GPIOGripperPluginConnection::PopulateSignalDescrptions(
    const std::vector<const intrinsic_proto::eoat::SignalConfig*>&
        signal_values,
    const absl::flat_hash_set<std::string>& writable_signals) {
  for (const intrinsic_proto::eoat::SignalConfig* config : signal_values) {
    for (const auto& [name, signal] : config->value_set().values()) {
      intrinsic_proto::gpio::v1::SignalDescription signal_description;
      signal_description.set_signal_name(name);
      signal_description.set_can_read(true);
      signal_description.set_can_write(false);
      signal_description.set_can_force(false);
      signal_description.set_type(gpio::SignalTypeFromValue(signal));
      signal_descriptions_.insert({name, signal_description});
    }
  }
  for (const auto& signal_name : writable_signals) {
    signal_descriptions_[signal_name].set_can_write(true);
  }
}

GPIOGripperPluginConnection::GPIOGripperPluginConnection(
    const intrinsic_proto::eoat::SuctionGripperConfig& suction_config,
    absl::string_view gripper_handle, SetCommandCallback set_command_cb,
    GetStatusCallback get_status_cb)
    : gripper_handle_(gripper_handle),
      set_command_(std::move(set_command_cb)),
      get_status_(std::move(get_status_cb)) {
  std::vector<const intrinsic_proto::eoat::SignalConfig*> all_values{
      &suction_config.grasp(), &suction_config.release(),
      &suction_config.blowoff_on(), &suction_config.blowoff_off(),
      &suction_config.gripping_indicated()};

  absl::flat_hash_set<std::string> signals_to_claim =
      gripper::SignalsToClaim(suction_config);
  PopulateSignalDescrptions(all_values, signals_to_claim);

  values_to_command_[suction_config.grasp().value_set()] = GraspCommand();
  values_to_command_[suction_config.release().value_set()] = ReleaseCommand();
  values_to_command_[suction_config.blowoff_on().value_set()] =
      BlowoffOnCommand();
  values_to_command_[suction_config.blowoff_off().value_set()] =
      BlowoffOffCommand();

  gpio_status_attached_ = suction_config.gripping_indicated().value_set();
  gpio_status_dettached_ = GetNegatedValueSet(gpio_status_attached_);

  writable_signal_values_.MergeFrom(suction_config.release().value_set());
  writable_signal_values_.MergeFrom(suction_config.blowoff_off().value_set());
}

GPIOGripperPluginConnection::GPIOGripperPluginConnection(
    const intrinsic_proto::eoat::PinchGripperConfig& pinch_config,
    absl::string_view gripper_handle, SetCommandCallback set_command_cb,
    GetStatusCallback get_status_cb)
    : gripper_handle_(gripper_handle),
      set_command_(std::move(set_command_cb)),
      get_status_(std::move(get_status_cb)) {
  std::vector<const intrinsic_proto::eoat::SignalConfig*> all_values{
      &pinch_config.grasp(), &pinch_config.release(),
      &pinch_config.gripping_indicated()};

  absl::flat_hash_set<std::string> signals_to_claim =
      gripper::SignalsToClaim(pinch_config);
  PopulateSignalDescrptions(all_values, signals_to_claim);

  values_to_command_[pinch_config.grasp().value_set()] = GraspCommand();
  values_to_command_[pinch_config.release().value_set()] = ReleaseCommand();

  writable_signal_values_.MergeFrom(pinch_config.release().value_set());

  gpio_status_attached_ = pinch_config.gripping_indicated().value_set();
  gpio_status_dettached_ = GetNegatedValueSet(gpio_status_attached_);
}

absl::flat_hash_map<std::string, intrinsic_proto::gpio::v1::SignalDescription>
GPIOGripperPluginConnection::GetSignalDescriptions() const {
  return signal_descriptions_;
}

absl::Status GPIOGripperPluginConnection::CheckSignalsWritable(
    const absl::flat_hash_set<std::string>& signal_names) {
  for (const auto& signal_name : signal_names) {
    if (!signal_descriptions_.contains(signal_name)) {
      return absl::NotFoundError(
          absl::Substitute("Signal $0 for fixed joint gripper $1 is not found",
                           signal_name, gripper_handle_));
    }
    if (!signal_descriptions_.at(signal_name).can_write()) {
      return absl::PermissionDeniedError(absl::Substitute(
          "Signal $0 for finxed joint gripper $1 is not writable", signal_name,
          gripper_handle_));
    }
  }
  return absl::OkStatus();
}

intrinsic_proto::gpio::v1::SignalValueSet
GPIOGripperPluginConnection::GetSignalValues() const {
  intrinsic_proto::gpio::v1::SignalValueSet result;
  if (get_status_) {
    const GripperStatus status = get_status_();
    if (status.grasp_status() == GripperStatus::ATTACHED) {
      result.MergeFrom(gpio_status_attached_);
    } else if (status.grasp_status() == GripperStatus::DETACHED) {
      result.MergeFrom(gpio_status_dettached_);
    }
  }
  result.MergeFrom(writable_signal_values_);
  return result;
}

absl::Status GPIOGripperPluginConnection::SetCommandValues(
    intrinsic_proto::gpio::v1::SignalValueSet command_values) {
  GripperCommand command;
  if (values_to_command_.contains(command_values)) {
    command = values_to_command_.at(command_values);
  } else {
    return absl::InvalidArgumentError(absl::Substitute(
        "Command values $0 are not compatible with fixed joint gripper $1",
        absl::StrCat(command_values), gripper_handle_));
  }
  writable_signal_values_.MergeFrom(command_values);
  if (set_command_) {
    INTR_RETURN_IF_ERROR(set_command_(command));
  }
  return absl::OkStatus();
}

}  // namespace simulation
}  // namespace intrinsic
