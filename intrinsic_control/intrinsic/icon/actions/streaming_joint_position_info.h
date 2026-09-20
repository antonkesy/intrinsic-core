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

#ifndef INTRINSIC_ICON_ACTIONS_STREAMING_JOINT_POSITION_INFO_H_
#define INTRINSIC_ICON_ACTIONS_STREAMING_JOINT_POSITION_INFO_H_

#include "absl/types/span.h"
#include "intrinsic/icon/actions/streaming_joint_position.pb.h"
#include "intrinsic/kinematics/types/joint_limits.h"

namespace intrinsic {
namespace icon {

// A point-to-point move action that also accepts updated joint
// position-velocity goal as a streaming input.
struct StreamingJointPositionInfo {
  static constexpr char kActionTypeName[] =
      "intrinsic.streaming_joint_position_move";
  static constexpr char kActionDescription[] =
      "Moves the part's joints to the desired target position. Target "
      "positions can be updated using non-realtime streaming Action inputs, "
      "which overwrite previously existing setpoints. Replanning towards the "
      "new joint goal is done instantaneously without guarantees w.r.t. the "
      "resulting geometry of the path.";
  static constexpr char kSlotName[] = "arm";
  static constexpr char kSlotDescription[] =
      "The action moves this Part in joint space.";
  static constexpr char kDistanceToSensed[] = "intrinsic.distance_to_sensed";
  static constexpr char kDistanceToSensedDescription[] =
      "Euclidean norm of the difference between the last setpoint and the "
      "sensed joint position.";

  // Streaming input for updating position-velocity goals. Type is the same as
  // `FixedParams`.
  static constexpr char kStreamingInputName[] = "streaming-position-command";

  using FixedParams =
      intrinsic_proto::icon::actions::proto::StreamingJointPositionParams;
};

// Returns a set of params for this action that specifies the
// `goal_position` and `goal_velocity`, used for both fixed and streaming input
// parameters.
StreamingJointPositionInfo::FixedParams GetStreamingJointPositionFixedParams(
    absl::Span<const double> goal_position,
    absl::Span<const double> goal_velocity);

// Returns a set of params for this action that specifies the
// `goal_position` and `goal_velocity` and `joint_limits`, used for both fixed
// and streaming input parameters.
// The `joint_limits` need to be within the range of valid limits of the
// machine.
StreamingJointPositionInfo::FixedParams GetStreamingJointPositionFixedParams(
    absl::Span<const double> goal_position,
    absl::Span<const double> goal_velocity, const JointLimits& joint_limits);

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_ACTIONS_STREAMING_JOINT_POSITION_INFO_H_
