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

#ifndef INTRINSIC_ICON_ACTIONS_JOINT_IMPEDANCE_ACTION_INFO_H_
#define INTRINSIC_ICON_ACTIONS_JOINT_IMPEDANCE_ACTION_INFO_H_

#include "intrinsic/icon/actions/joint_impedance_action.pb.h"

namespace intrinsic {
namespace icon {

struct JointImpedanceInfo {
  static constexpr char kActionTypeName[] = "intrinsic.joint_impedance";
  static constexpr char kActionDescription[] =
      "Action that implements a joint impedance control law for a torque "
      "controlled robot. The action allows for streaming updates of the target "
      "joint position and velocity. A smooth and joint-limited trajectory to "
      "the target state is generated internally and used as the reference for "
      "the impedance control law. If a `Dynamics` interface is provided, the "
      "action computes and applies inverse dynamics feedforward torques. If "
      "the robot is gravity compensated including appropriate end effector "
      "inertia configuration on the robot controller, this is not strictly "
      "necessary and may be omitted. In this case the commanded torques will "
      "be purely the result of the stiffness and damping terms defined by the "
      "user.";
  static constexpr char kSlotName[] = "arm";
  static constexpr char kSlotDescription[] =
      "The action moves this Part in joint space.";
  static constexpr char kDistanceToSensed[] = "intrinsic.distance_to_sensed";
  static constexpr char kDistanceToSensedDescription[] =
      "Euclidean norm of the difference between the last setpoint and the "
      "sensed joint position.";
  static constexpr char kIsSettled[] = "is_settled";
  static constexpr char kIsSettledDescription[] =
      "This Action reports 'settled' as soon as the robot has reached a "
      "settled state after last streaming input reference position has been "
      "reached.";
  // Streaming input for updating position-velocity targets. Type is the same as
  // `FixedParams`.
  static constexpr char kStreamingInputName[] = "joint-impedance-command";

  using FixedParams =
      intrinsic_proto::icon::actions::proto::JointImpedanceParams;

  using StreamingParams =
      intrinsic_proto::icon::actions::proto::JointImpedanceParams;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_ACTIONS_JOINT_IMPEDANCE_ACTION_INFO_H_
