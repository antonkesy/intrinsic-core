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

#ifndef INTRINSIC_HARDWARE_MODBUS_TCP_MODBUS_TCP_H_
#define INTRINSIC_HARDWARE_MODBUS_TCP_MODBUS_TCP_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "libmodbus/src/modbus-tcp.h"
#include "libmodbus/src/modbus.h"

namespace intrinsic {
namespace hardware {

// Custom Deleter to handle safe unloading of modbus connection.
struct ModbusDeleter {
  void operator()(modbus_t* client) {
    if (client != nullptr) {
      modbus_close(client);
      modbus_free(client);
      client = nullptr;
    }
  }
};

using ModbusPtr = std::unique_ptr<modbus_t, ModbusDeleter>;

// The ModbusTcp object initializes and maintains a connection
// to a Modbus device over TCP and provides an interface for read/write of
// device registers.
class ModbusTcp {
 public:
  static absl::StatusOr<std::unique_ptr<ModbusTcp>> Create(
      absl::string_view ip_address, int port = MODBUS_TCP_DEFAULT_PORT);
  virtual ~ModbusTcp();

  // Discards data received (but does not read) from device.
  absl::Status flush();

  // Writes data to the device
  virtual absl::Status write(absl::Span<uint16_t> data,
                             int start_register_address);
  absl::Status write(absl::Span<uint16_t> data) { return write(data, 0); }

  // Reads data from the device.
  virtual absl::StatusOr<std::vector<uint16_t>> read(
      size_t length, int start_register_address);
  absl::StatusOr<std::vector<uint16_t>> read(size_t length) {
    return read(length, 0);
  }

 protected:
  explicit ModbusTcp(absl::string_view ip_address,
                     int port = MODBUS_TCP_DEFAULT_PORT);
  absl::Status connect();
  absl::Status disconnect();

  enum Status { kUninitialized = 0, kNotConnected, kConnected };
  ModbusPtr modbus_client_;

  absl::string_view ip_address_;
  const int port_;
  Status status_;
};

}  // namespace hardware
}  // namespace intrinsic

#endif  // INTRINSIC_HARDWARE_MODBUS_TCP_MODBUS_TCP_H_
