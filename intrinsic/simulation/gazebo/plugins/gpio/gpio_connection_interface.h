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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GPIO_GPIO_CONNECTION_INTERFACE_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GPIO_GPIO_CONNECTION_INTERFACE_H_

#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"

namespace intrinsic {
namespace simulation {
// Abstract class that defines a connection between a gazebo plugin and
// simulation::GPIOService.
class GPIOConnectionInterface {
 public:
  virtual ~GPIOConnectionInterface() = default;

  virtual absl::flat_hash_map<std::string,
                              intrinsic_proto::gpio::v1::SignalDescription>
  GetSignalDescriptions() const = 0;

  // Checks if signal_names are all writable in this gripper connection.
  // Returns OK status if the signals are writable. Returns an error if not all
  // signals are writable.
  virtual absl::Status CheckSignalsWritable(
      const absl::flat_hash_set<std::string>& signal_names) = 0;

  // Gets all signal values that this connection represents.
  virtual intrinsic_proto::gpio::v1::SignalValueSet GetSignalValues() const = 0;

  // Sets command_values to the underlying gripper implementation. Only
  // command_values matching one of the commands configured are accepted.
  virtual absl::Status SetCommandValues(
      intrinsic_proto::gpio::v1::SignalValueSet command_values) = 0;

  virtual const std::string& plugin_handle() const = 0;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GPIO_GPIO_CONNECTION_INTERFACE_H_
