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

#include "intrinsic/hardware/gripper/wsg32/wsg32_client.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <string_view>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/substitute.h"
#include "wsg_32/cmd.h"
#include "wsg_32/common.h"
#include "wsg_32/functions.h"

// These are factory default values of the WSG32 Gripper.
ABSL_FLAG(std::string, wsg32_ip, "192.168.1.20",
          "The IP of the wsg32 schunk gripper");
ABSL_FLAG(int32_t, wsg32_port, 50042,
          "The remote port of the wsg32 schunk gripper");

namespace intrinsic::gripper {

//////////////////////////////////////////////////////////////////////////////
// Create a Wsg32Client instance, connect to the gripper, and home it
// Returns a Wsg32Client instance, or nullptr if it could not connect.
//
absl::StatusOr<std::unique_ptr<Wsg32Client>> Wsg32Client::Create(
    std::string_view ip, int port, bool home) {
  std::unique_ptr<Wsg32Client> client(new Wsg32Client(ip, port));
  if (!client->Connect(home).ok()) {
    return absl::InternalError("Could not connect to gripper.");
  }
  return client;
}

////////////////////////////////////////////////////////////////////////////////
// ExecuteCmd performs all motion types of the wsg32 gripper. This code was
// inspired from third_party/robotics/juggler/devices/robot/wsg32
absl::Status Wsg32Client::ExecuteCmd(unsigned char cmd_type, float cmd_width,
                                     float cmd_speed,
                                     gripper_response* info) const {
  status_t status;
  int res;
  constexpr unsigned char CMD_BASE = 0xB0;
  unsigned char payload[9];
  unsigned char* resp;
  unsigned int resp_len;

  // Custom payload format:
  // 0: Unused
  // 1: float, target width, used for 0xB1 command
  // 5: float, target speed, used for 0xB1 and 0xB2 command
  payload[0] = 0x00;
  memcpy(&payload[1], &cmd_width, sizeof(float));
  memcpy(&payload[5], &cmd_speed, sizeof(float));

  // Submit command and process result
  res = cmd_submit(CMD_BASE + cmd_type, payload, 9, true, &resp, &resp_len);
  try {
    if (res < 2) {
      return absl::InternalError("Invalid Response");
    }
    status = cmd_get_response_status(resp);
    if (status == E_CMD_UNKNOWN) {
      return absl::InternalError("WSG32 command unknown.");
    } else if (status != E_SUCCESS) {
      return absl::InternalError("WSG32 command failed.");
    }
    if (res != 15) {
      return absl::InternalError("WSG32 response payload incorrect (" +
                                 std::to_string(res) + ")");
    }

    // Extract data from response
    int off = 2;
    unsigned char resp_state[6] = {0, 0, 0, 0, 0, 0};
    resp_state[2] = resp[2];
    info->state_text = getStateValues(resp_state);

    info->state = resp[2];
    off += 1;
    info->position = convert(&resp[off]);
    off += 4;
    info->speed = convert(&resp[off]);
    off += 4;
    info->f_motor = convert(&resp[off]);
    off += 4;
    info->f_finger0 = convert(&resp[off]);
    off += 4;
    info->f_finger1 = convert(&resp[off]);

    info->ismoving = (info->state & 0x02 /*fingers moving*/) != 0;
    // only in position mode; cannot determine reliably for velocity mode
    // 0x40 /* axis stopped */
  } catch (std::string& msg) {
    if (res > 0) free(resp);
    return absl::InternalError("ExecuteCmd failed for wsg32 gripper");
  }

  free(resp);
  return absl::OkStatus();
}

////////////////////////////////////////////////////////////////////////////////
// Connect to gripper with ip and port. Optional do homing.
//
absl::Status Wsg32Client::Connect(bool home) {
  if (IsConnected()) {
    LOG(INFO) << "WSG32 gripper already connected.";
    return absl::OkStatus();
  }

  if (cmd_connect_tcp(ip_.c_str(), port_) != 0) {
    return absl::InternalError(absl::Substitute(
        "Problem connecting WSG32 at ip: $0 at port: $1", ip_, port_));
  }
  LOG(INFO) << "Connected to Wsg32 Gripper at ip: " << ip_
            << " and port: " << port_;

  if (home && homing() != 0) {
    return absl::InternalError("Problem homing the WSG32 gripper.");
  }
  const double grasping_force = kMaxForceN;
  if (setGraspingForceLimit(grasping_force) != 0) {
    return absl::InternalError(absl::Substitute(
        "Failed to set force limit of WSG32 to: $0", grasping_force));
  }

  return absl::OkStatus();
}

////////////////////////////////////////////////////////////////////////////////
// Read status structure from gripper.
//
Wsg32Client::Status Wsg32Client::GetStatus() const {
  gripper_response info;
  if (!ExecuteCmd(kReadOnly, /*cmd_width=*/0.0, /*cmd_speed=*/0.0, &info)
           .ok()) {
    return Status{};
  };
  return Status{
      .position = info.position,
      .speed = info.speed,
      .f_motor = info.f_motor,
      .is_moving = info.ismoving,
  };
}

////////////////////////////////////////////////////////////////////////////////
// Generic move function for WSG32 gripper.
//
bool Wsg32Client::Move(double width_mm, double speed, double force) {
  // Check force limits.
  auto clamped_force = std::clamp(force, static_cast<double>(kMinForceN),
                                  static_cast<double>(kMaxForceN));
  if ((clamped_force == kMinForceN || clamped_force == kMaxForceN) &&
      clamped_force != force) {
    LOG(INFO) << "Gripper force: " << force
              << " was clipped to limit: " << clamped_force;
  }

  if (setGraspingForceLimit(clamped_force) != 0) {
    LOG(ERROR) << "Failed to set force limit to: " << clamped_force;
    return false;
  }

  // Check speed limits.
  auto clamped_speed = std::clamp(speed, static_cast<double>(kMinSpeedMmS),
                                  static_cast<double>(kMaxSpeedMmS));
  if ((clamped_speed == kMinSpeedMmS || clamped_speed == kMaxSpeedMmS) &&
      clamped_speed != speed) {
    LOG(INFO) << "Gripper speed: " << speed
              << " was clipped to limit: " << clamped_speed;
  }

  // Check position limits.
  auto clamped_width_mm = std::clamp(width_mm, static_cast<double>(kMinWidthMm),
                                     static_cast<double>(kMaxWidthMm));
  if ((clamped_width_mm == kMinWidthMm || clamped_width_mm == kMaxWidthMm) &&
      clamped_width_mm != width_mm) {
    LOG(INFO) << "Gripper width: " << width_mm
              << " was clipped to limit: " << clamped_width_mm;
  }

  gripper_response info;
  if (!ExecuteCmd(kReadAndProvideGoalAndGoalSpeed, clamped_width_mm,
                  clamped_speed, &info)
           .ok()) {
    return false;
  };

  return true;
}

}  // namespace intrinsic::gripper
