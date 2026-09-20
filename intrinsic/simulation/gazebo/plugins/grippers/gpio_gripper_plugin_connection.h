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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_GPIO_GRIPPER_PLUGIN_CONNECTION_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_GPIO_GRIPPER_PLUGIN_CONNECTION_H_

#include <functional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/hardware/gpio/v1/signal.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"
#include "intrinsic/simulation/gazebo/plugins/gpio/gpio_connection_interface.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/gripper.pb.h"
#include "intrinsic/util/proto/pb_hash.h"

namespace intrinsic {
namespace simulation {

// Defines the connection between a simulation::GPIOService and a
// fixed joint gripper plugin. Responsible for translating GPIO command signals
// written to gripper commands to the corresponding
// FixedJointGripperPlugin and report grasping status as gpio
// signals.
class GPIOGripperPluginConnection : public GPIOConnectionInterface {
 public:
  // Constructs and sets up this connection to support signals provided in
  // suction_config. set_command_cb and get_status_cb are used to communicate
  // with owning FixedJointGripperPlugin without depending on it to avoid
  // circular dependency.
  using SetCommandCallback = std::function<absl::Status(
      const intrinsic_proto::simulation::gazebo::GripperCommand&)>;
  using GetStatusCallback =
      std::function<intrinsic_proto::simulation::gazebo::GripperStatus()>;

  explicit GPIOGripperPluginConnection(
      const intrinsic_proto::eoat::SuctionGripperConfig& suction_config,
      absl::string_view gripper_handle, SetCommandCallback set_command_cb,
      GetStatusCallback get_status_cb);

  GPIOGripperPluginConnection(
      const intrinsic_proto::eoat::PinchGripperConfig& pinch_config,
      absl::string_view gripper_handle, SetCommandCallback set_command_cb,
      GetStatusCallback get_status_cb);

  ~GPIOGripperPluginConnection() override = default;

  // Gets the signal description for all signal that this grippe connection
  // represents.
  absl::flat_hash_map<std::string, intrinsic_proto::gpio::v1::SignalDescription>
  GetSignalDescriptions() const override;

  // Checks if signal_names are all writable in this gripper connection.
  // Returns OK status if the signals are writable. Returns an error if not all
  // signals are writable.
  absl::Status CheckSignalsWritable(
      const absl::flat_hash_set<std::string>& signal_names) override;

  // Gets all signal values that this connection represents.
  intrinsic_proto::gpio::v1::SignalValueSet GetSignalValues() const override;

  // Sets command_values to the underlying gripper implementation. Only
  // command_values matching one of the commands configured are accepted.
  absl::Status SetCommandValues(
      intrinsic_proto::gpio::v1::SignalValueSet command_values) override;

  const std::string& plugin_handle() const override { return gripper_handle_; }

 private:
  void PopulateSignalDescrptions(
      const std::vector<const intrinsic_proto::eoat::SignalConfig*>&
          signal_values,
      const absl::flat_hash_set<std::string>& writable_signals);

  std::string gripper_handle_;

  intrinsic_proto::gpio::v1::SignalValueSet gpio_status_attached_;
  intrinsic_proto::gpio::v1::SignalValueSet gpio_status_dettached_;

  SetCommandCallback set_command_;
  GetStatusCallback get_status_;

  absl::flat_hash_map<std::string, intrinsic_proto::gpio::v1::SignalDescription>
      signal_descriptions_;
  intrinsic_proto::gpio::v1::SignalValueSet writable_signal_values_;

  absl::flat_hash_map<intrinsic_proto::gpio::v1::SignalValueSet,
                      intrinsic_proto::simulation::gazebo::GripperCommand,
                      pb_hash, pb_equals>
      values_to_command_;
};

namespace gpio_gripper_plugin_connection_internal {
// Returns a negated version of original SignalValueSet provided. The negation
// treats each value as a bool and return the negated boolean value's
// representation in its original type. As a result, only 0 and 1 values are
// returned for numeric values.
intrinsic_proto::gpio::v1::SignalValueSet GetNegatedValueSet(
    const intrinsic_proto::gpio::v1::SignalValueSet& original);
}  // namespace gpio_gripper_plugin_connection_internal

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_GPIO_GRIPPER_PLUGIN_CONNECTION_H_
