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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GENERIC_ACTION_GENERIC_ACTION_GPIO_CONNECTION_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GENERIC_ACTION_GENERIC_ACTION_GPIO_CONNECTION_H_

#include <functional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gpio/v1/signal.pb.h"
#include "intrinsic/simulation/gazebo/plugins/gpio/gpio_connection_interface.h"
#include "intrinsic/util/proto/pb_hash.h"

namespace intrinsic {
namespace simulation {

// Defines the connection between a simulation::GPIOService and a generic action
// plugin. Responsible for translating GPIO read and write requests to the
// underlying gazebo commands.
class GPIOGenericActionPluginConnection : public GPIOConnectionInterface {
 public:
  using SetCommandCallback =
      std::function<void(const intrinsic_proto::gpio::v1::SignalValueSet&)>;
  using GetStatusCallback =
      std::function<intrinsic_proto::gpio::v1::SignalValueSet()>;

  explicit GPIOGenericActionPluginConnection(absl::string_view plugin_handle);

  ~GPIOGenericActionPluginConnection() override = default;

  // Gets the signal description for all signal that this GPIO connection
  // represents.
  absl::flat_hash_map<std::string, intrinsic_proto::gpio::v1::SignalDescription>
  GetSignalDescriptions() const override;

  // Checks if signal_names are all writable in this GPIO connection.
  // Returns OK status if the signals are writable. Returns an error if not all
  // signals are writable.
  absl::Status CheckSignalsWritable(
      const absl::flat_hash_set<std::string>& signal_names) override;

  // TODO(b/275102358): Figure out a better way than returning all signal values
  // from a connection.
  // Gets all signal values that this connection represents.
  intrinsic_proto::gpio::v1::SignalValueSet GetSignalValues() const override;

  // TODO(b/275102358): Consider making this a function in
  // GPIOConnectionInterface.
  // Returns signal values based on signal_names.
  // If any of the names are not supported by this connection, a
  // absl::NotFoundError is returned.
  absl::StatusOr<intrinsic_proto::gpio::v1::SignalValueSet> GetStatusValues(
      const absl::flat_hash_set<std::string>& signal_names);

  // Sets command_values to the underlying GPIO implementation. Only
  // command_values matching one of the commands configured are accepted.
  absl::Status SetCommandValues(
      intrinsic_proto::gpio::v1::SignalValueSet command_values) override;

  const std::string& plugin_handle() const override { return plugin_handle_; }

  // Registers a set_command callback that expects a signal value set described
  // by command_signal_descriptions. set_command will be invoked when a matching
  // SignalValueSet is received from its registered GPIOService.
  absl::Status RegisterSetCommandCallback(
      const std::vector<intrinsic_proto::gpio::v1::SignalDescription>&
          command_signal_descriptions,
      SetCommandCallback set_command);

  // Registers a set_command callback that expects a specific signal value set
  // specified by command_signal_values. set_command will be called when this
  // connection receives exactly command_signal_values
  absl::Status RegisterSetCommandCallback(
      intrinsic_proto::gpio::v1::SignalValueSet command_signal_values,
      SetCommandCallback set_command);

  // Registers a get_status callback that expects signal read request on a
  // signal value set described by status_signal_descriptions. get_status will
  // be invoked that returns a matching SignalValueSet.
  //
  absl::Status RegisterGetStatusCallback(
      const absl::flat_hash_set<std::string>& status_signal_names,
      GetStatusCallback get_status);

 private:
  std::string plugin_handle_;

  absl::flat_hash_map<std::string, intrinsic_proto::gpio::v1::SignalDescription>
      signal_descriptions_;

  // TODO(qingyou): A set of signal names is not unique enough to identify a
  // callback for this connection., Should we hash command value set?
  absl::flat_hash_map<absl::flat_hash_set<std::string>, SetCommandCallback>
      set_command_cbs_;

  absl::flat_hash_map<intrinsic_proto::gpio::v1::SignalValueSet,
                      SetCommandCallback, pb_hash, pb_equals>
      values_to_command_cb_;

  absl::flat_hash_map<absl::flat_hash_set<std::string>, GetStatusCallback>
      get_status_cbs_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GENERIC_ACTION_GENERIC_ACTION_GPIO_CONNECTION_H_
