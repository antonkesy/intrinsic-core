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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_DRIVER_INTERFACE_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_DRIVER_INTERFACE_H_

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "include/ur_client_library/comm/control_mode.h"
#include "include/ur_client_library/primary/robot_message/error_code_message.h"
#include "include/ur_client_library/rtde/data_package.h"
#include "include/ur_client_library/types.h"
#include "include/ur_client_library/ur/datatypes.h"
#include "include/ur_client_library/ur/robot_receive_timeout.h"

namespace urcl::primary_interface {
struct ErrorCode;
}  // namespace urcl::primary_interface

namespace intrinsic::icon {

// Interface abstraction for the Universal Robots hardware driver backend.
class UrDriverInterface {
 public:
  virtual ~UrDriverInterface() = default;

  // Dashboard Client Control Interface
  virtual absl::Status ConnectDashboard(absl::string_view robot_ip) = 0;
  virtual void DisconnectDashboard() = 0;
  virtual void ResetDashboard() = 0;
  virtual bool CommandAddToLog(absl::string_view message) = 0;
  virtual bool CommandIsInRemoteControl() = 0;
  virtual bool CommandStop() = 0;
  virtual bool CommandGetRobotModel(std::string& robot_model) = 0;
  virtual bool CommandGetSerialNumber(std::string& serial_number) = 0;
  virtual bool CommandPowerOn() = 0;
  virtual bool CommandBrakeRelease() = 0;
  virtual bool CommandPowerOff() = 0;
  virtual bool CommandSafetyMode(std::string& safety_mode_string) = 0;
  virtual bool CommandUnlockProtectiveStop() = 0;
  virtual bool CommandRestartSafety() = 0;
  virtual bool CommandCloseSafetyPopup() = 0;

  // Primary & RTDE Driver Interface
  virtual absl::Status InitializeDriver(
      absl::string_view robot_ip, absl::string_view script_file,
      absl::string_view output_recipe, absl::string_view input_recipe,
      std::function<void(bool)> handle_program_state, bool headless_mode,
      uint32_t reverse_port, uint32_t script_sender_port, int servoj_gain,
      double servoj_lookahead_time, bool non_blocking_read,
      absl::string_view reverse_ip, uint32_t trajectory_port,
      uint32_t script_command_port) = 0;

  virtual void ResetDriver() = 0;
  virtual bool IsDriverValid() const = 0;
  virtual bool CheckCalibration(absl::string_view checksum) = 0;
  virtual urcl::VersionInformation GetVersion() = 0;
  virtual double GetControlFrequency() = 0;

  // High-Frequency Control Loops
  virtual std::unique_ptr<urcl::rtde_interface::DataPackage>
  GetDataPackage() = 0;
  virtual bool WriteJointCommand(const urcl::vector6d_t& values,
                                 urcl::comm::ControlMode mode,
                                 urcl::RobotReceiveTimeout timeout) = 0;
  virtual void WriteKeepalive() = 0;
  virtual bool StopControl() = 0;

  // Digital I/O Streams
  virtual bool SendStandardDigitalOutput(uint8_t output_mask,
                                         uint8_t output_bits) = 0;
  virtual bool SendConfigurableDigitalOutput(uint8_t output_mask,
                                             uint8_t output_bits) = 0;
  virtual bool SendToolDigitalOutput(uint8_t output_mask,
                                     uint8_t output_bits) = 0;

  // External Sensor Streams
  virtual bool SendExternalForceTorque(
      const urcl::vector6d_t& total_wrench) = 0;
  virtual void FtRtdeInputEnable(bool enable) = 0;
  virtual bool ZeroFTSensor() = 0;

  // Payload & Scales
  virtual bool SendRobotProgram() = 0;
  virtual bool SetPayload(double mass,
                          const urcl::vector3d_t& center_of_gravity) = 0;
  virtual bool SetFrictionScales(const urcl::vector6d_t& viscous,
                                 const urcl::vector6d_t& coulomb) = 0;

  // Comm Management
  virtual void StartRTDECommunication() = 0;
  virtual std::deque<urcl::primary_interface::ErrorCode> GetErrorCodes() = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_DRIVER_INTERFACE_H_
