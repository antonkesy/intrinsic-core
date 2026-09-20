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

#include "intrinsic/hardware/modbus_tcp/modbus_tcp.h"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/util/status/status_macros.h"
#include "libmodbus/src/modbus-tcp.h"
#include "libmodbus/src/modbus.h"

ABSL_FLAG(bool, modbus_debug_output, false, "Enable modbus debug messages.");

namespace intrinsic {
namespace hardware {

ModbusTcp::ModbusTcp(absl::string_view ip_address, const int port)
    : modbus_client_(modbus_new_tcp(std::string(ip_address).c_str(), port)),
      ip_address_(ip_address),
      port_(port),
      status_(kUninitialized) {
  // TODO(williambaker): DNS lookup
}

absl::StatusOr<std::unique_ptr<ModbusTcp>> ModbusTcp::Create(
    absl::string_view ip_address, int port) {
  auto modbus = absl::WrapUnique(new ModbusTcp(ip_address, port));
  INTR_RETURN_IF_ERROR(modbus->connect());
  return modbus;
}

ModbusTcp::~ModbusTcp() {
  if (!disconnect().ok()) {
    LOG(ERROR) << "Failed to disconnect.";
  }
}

absl::Status ModbusTcp::connect() {
  if (status_ == kConnected) {
    return absl::OkStatus();
  }
  LOG(INFO) << "ModbusTcp::connect(): " << ip_address_ << ":" << port_;

  if (modbus_client_ == nullptr) {
    modbus_client_ =
        ModbusPtr(modbus_new_tcp(std::string(ip_address_).c_str(), port_));
  }
  // Connecting to device.
  if (modbus_connect(modbus_client_.get()) == -1) {
    auto error = absl::InternalError(modbus_strerror(errno));
    modbus_client_ = nullptr;
    return error;
  }
  status_ = kConnected;

  // Enabling debug output if request (bytes sent and received).
  if (absl::GetFlag(FLAGS_modbus_debug_output)) {
    modbus_set_debug(modbus_client_.get(), 1);
  }
  // Setting unit id to 0/broadcast assuming only one device per IP address.
  modbus_set_slave(modbus_client_.get(), 0);

  return absl::OkStatus();
}

absl::Status ModbusTcp::flush() {
  if (modbus_client_ == nullptr || status_ != kConnected) {
    return absl::UnavailableError("Invalid connection.");
  }
  if (modbus_flush(modbus_client_.get()) == -1) {
    return absl::InternalError("Failed to flush connection.");
  }
  return absl::OkStatus();
}

absl::Status ModbusTcp::disconnect() {
  if (status_ == kNotConnected) {
    return absl::OkStatus();
  }
  modbus_client_ = nullptr;
  status_ = kNotConnected;
  return absl::OkStatus();
}

absl::Status ModbusTcp::write(const absl::Span<uint16_t> data,
                              int start_register_address) {
  VLOG(2) << "Write: " << data.length() << " " << start_register_address;
  if (modbus_client_ == nullptr || status_ != kConnected) {
    return absl::UnavailableError("Invalid connection.");
  }
  int num_written_registers = modbus_write_registers(
      modbus_client_.get(), start_register_address, data.length(), data.data());
  if (num_written_registers != data.length()) {  // -1 fail
    return absl::InternalError(
        absl::StrFormat("Failed to write registers(%d/%d).",
                        num_written_registers, data.length()));
  }
  return absl::OkStatus();
}

absl::StatusOr<std::vector<uint16_t>> ModbusTcp::read(
    const size_t length, int start_register_address) {
  VLOG(2) << "ModbusTcp::read " << length << " bytes starting at "
          << start_register_address;
  std::vector<uint16_t> data(length);
  if (modbus_client_ == nullptr || status_ != kConnected) {
    return absl::UnavailableError("Invalid connection.");
  }
  int num_bytes = modbus_read_input_registers(
      modbus_client_.get(), start_register_address, length, data.data());
  if (num_bytes != length) {
    return absl::InternalError(
        absl::StrFormat("Failed to read bytes(%d/%d): %s.", num_bytes, length,
                        modbus_strerror(errno)));
  }
  return data;
}

}  // namespace hardware
}  // namespace intrinsic
