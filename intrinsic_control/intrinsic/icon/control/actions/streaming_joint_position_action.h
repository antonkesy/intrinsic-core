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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_STREAMING_JOINT_POSITION_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_STREAMING_JOINT_POSITION_ACTION_H_

#include <cstddef>
#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/streaming_joint_position_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/joint_position_reflexxes.h"
#include "intrinsic/icon/control/algorithms/joint_velocity_reflexxes.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

class StreamingJointPositionAction final : public RtclActionInterface {
 public:
  // Real-time data type for both fixed and streaming input parameters.
  struct Params {
    JointStatePV goal;
    JointLimits joint_limits;
  };

  StreamingJointPositionAction(RealtimeSlotId slot_id,
                               StreamingInputId streaming_input_id, size_t ndof,
                               double frequency_hz, Params initial_params);

  static absl::StatusOr<std::unique_ptr<StreamingJointPositionAction>> Create(
      StreamingJointPositionInfo::FixedParams params_proto,
      ActionFactoryContext& context);

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

  // Params `application_limits` are before `params` so they can be bound using
  // bind_front.
  static absl::StatusOr<Params> ParseStreamingInput(
      const JointLimits& application_limits,
      const StreamingJointPositionInfo::FixedParams& params);

 private:
  // This threshold for the speed override value defines where the
  // reflexxes-based trajectory generator switches from a position to a velocity
  // based. Between 0 and the threshold a velocity-based controller is used and
  // between the threshold and 1 a position-based controller is used.
  static constexpr double kSwitchSpeedOverride = 0.01;

  RealtimeStatus UpdateGoal(const JointStatePV& goal,
                            const JointLimits& limits);

  RealtimeSlotId slot_id_;
  StreamingInputId streaming_input_id_;
  size_t ndof_;
  JointPositionReflexxes trajectory_generator_position_;
  JointVelocityReflexxes trajectory_generator_velocity_;
  Params current_params_;
  const Params initial_params_;
  // The current position goal, as set by the last call to Command().
  std::optional<JointStatePV> current_goal_;
  // The setpoint from the previous cycle. Used by the reflexxes interpolator.
  std::optional<JointStatePVA> previous_setpoint_;
  // Saves the "done" status of the Action between calls to Control() and
  // Sense().
  bool done_buffer_ = false;
  // Internal variable corresponding to the "done" condition variable. Only
  // changes values when Sense() is called.
  bool done_ = false;
  std::optional<double> distance_to_target_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_STREAMING_JOINT_POSITION_ACTION_H_
