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

#ifndef INTRINSIC_HARDWARE_GRIPPER_WSG32_WSG32_CLIENT_H_
#define INTRINSIC_HARDWARE_GRIPPER_WSG32_WSG32_CLIENT_H_

#include <memory>
#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "wsg_32/cmd.h"
#include "wsg_32/functions.h"

////////////////////////////////////////////////////////////////////////////////
// Wrapper for Weiss Robotics WSG-32 Gripper using third_party/wsg32 libraries.
// Note: this class is not thread safe.
//

namespace intrinsic::gripper {

class Wsg32Client {
 public:
  // Max/min gripper force.
  const float kMinForceN = 5.0;
  const float kMaxForceN = 50.0;

  // Max gripper speed.
  const float kMinSpeedMmS = 5.0;
  const float kMaxSpeedMmS = 400.0;

  // Max gripper width.
  const float kMinWidthMm = 0.0;
  const float kMaxWidthMm = 68.0;

  // Gripper commands.
  enum GripperCommand {
    kReadOnly = 0,
    kReadAndProvideGoalAndGoalSpeed = 1,
    kReadAndProvideGoalSpeed = 2
  };

  //////////////////////////////////////////////////////////////////////////////
  // Create a Wsg32Client instance, connect to the gripper, and home it
  // Returns a Wsg32Client instance, or nullptr if it could not connect.
  //
  static absl::StatusOr<std::unique_ptr<Wsg32Client>> Create(
      std::string_view ip, int port, bool home);

  //////////////////////////////////////////////////////////////////////////////
  // Disconnect on destruction.
  //
  ~Wsg32Client() { cmd_disconnect(); }

  //////////////////////////////////////////////////////////////////////////////
  // Status struct returned in GetStatus().
  //
  struct Status {
    float position = 0.0;
    float speed = 0.0;
    float f_motor = 0.0;
    bool is_moving = false;
  };

  //////////////////////////////////////////////////////////////////////////////
  // Return status information.
  //
  Status GetStatus() const;

  //////////////////////////////////////////////////////////////////////////////
  // Connect with optional homing.
  //
  absl::Status Connect(bool home = false);

  //////////////////////////////////////////////////////////////////////////////
  // Disconnect.
  //
  void Disconnect() { cmd_disconnect(); };

  //////////////////////////////////////////////////////////////////////////////
  // Return connection status.
  //
  bool IsConnected() { return cmd_is_connected(); }

  //////////////////////////////////////////////////////////////////////////////
  // Close gripper as much as possible, with the desired force.
  //
  // force - in N between kMinForce and kMaxForce
  // speed - movement speed.
  // return true on success, otherwise false.
  //
  bool Close(double force, double speed) { return Move(0.0, speed, force); }

  //////////////////////////////////////////////////////////////////////////////
  // Opens gripper to desired aperture with desired speed.
  // width_mm - desired width in millimeters of the >base< fingers.
  // speed    - movement speed.
  // return true on success, otherwise false.
  //
  bool Open(double width_mm, double speed) {
    return Move(width_mm, speed, kMaxForceN);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Generic move command for fingers.
  // width_mm - desired width in millimeters of the >base< fingers.
  // speed    - movement speed in millimeters/seconds.
  // force    - force in Newton between kMinForce and kMaxForce
  // return true on success, otherwise false.
  //
  bool Move(double width_mm, double speed, double force);

  //////////////////////////////////////////////////////////////////////////////
  // ExecuteCmd performs all motion types of the wsg32 gripper. This code was
  // inspired from third_party/robotics/juggler/devices/robot/wsg32
  //
  // cmd_type   - kReadOnly
  //              kReadAndProvideGoalAndGoalSpeed
  //              kReadAndProvideGoalSpeed
  // cmd_width  - gripper target width in mm
  // cmd_speed  - gripper motion speed in mm/s
  // info       - gripper status information
  //
  absl::Status ExecuteCmd(unsigned char cmd_type, float cmd_width,
                          float cmd_speed, gripper_response* info) const;

 private:
  //////////////////////////////////////////////////////////////////////////////
  // Constructor takes ip and port information of gripper.
  //
  Wsg32Client(absl::string_view& ip, int port) : ip_(ip), port_(port) {}

  // IP and PORT information.
  std::string ip_;
  int port_;
};

}  // namespace intrinsic::gripper
#endif  // INTRINSIC_HARDWARE_GRIPPER_WSG32_WSG32_CLIENT_H_
