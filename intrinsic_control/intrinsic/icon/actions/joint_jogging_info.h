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

#ifndef INTRINSIC_ICON_ACTIONS_JOINT_JOGGING_INFO_H_
#define INTRINSIC_ICON_ACTIONS_JOINT_JOGGING_INFO_H_

#include "intrinsic/icon/actions/joint_jogging.pb.h"

namespace intrinsic {
namespace icon {

struct JointJoggingInfo {
  static constexpr char kActionTypeName[] = "intrinsic.joint_jogging";
  static constexpr char kActionDescription[] =
      "Generates and executes an open-loop joint move with the commanded joint "
      "velocities. The action starts with zero joint velocities at the current "
      "joint position and subsequently executes the joint velocities commanded "
      "by non-realtime streaming commands. The velocity setpoints are achieved "
      "in time-optimal fashion w.r.t. currently configured joint acceleration "
      "and jerk limits. Movement is stopped if no streaming command is "
      "received within `kWatchdogTimeoutInSeconds` of the most "
      "recent streaming command. If a speed override multiplier is set, it "
      "will be used to scale the commanded target joint velocity. You are "
      "required to set appropriate joint velocity limits using `FixedParams`.";
  static constexpr char kSlotName[] = "arm";
  static constexpr char kSlotDescription[] =
      "The action jogs this Part in joint space.";

  static constexpr char kStreamingInputName[] = "joint_jogging_command";
  static constexpr char kStreamingInputDescription[] =
      "Streaming joint velocities. The robot accelerates and decelerates "
      "according to the limits set in 'FixedParams'.\n  "
      "Movement is stopped if no streaming command is received within "
      "'kWatchdogTimeoutInSeconds' of the most recent streaming command.";

  static constexpr char kTimedOut[] = "intrinsic.timed_out";
  static constexpr char kTimedOutDescription[] =
      "`Unavailable` before the action receives a streaming command. Switches "
      "to `False` when a streaming command is received.\n"
      "`True` after the watchdog (commanding zero velocities) is triggered.";

  // Time in seconds without new streaming command after which the watchdog
  // stops the robot.
  static constexpr double kWatchdogTimeoutInSeconds = 0.25;

  using FixedParams =
      ::intrinsic_proto::icon::actions::proto::JointJoggingFixedParams;
  using StreamingParams =
      ::intrinsic_proto::icon::actions::proto::JointJoggingStreamingParams;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_ACTIONS_JOINT_JOGGING_INFO_H_
