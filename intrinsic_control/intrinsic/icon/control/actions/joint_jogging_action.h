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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_JOINT_JOGGING_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_JOINT_JOGGING_ACTION_H_

#include <cstddef>
#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/joint_jogging_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/joint_velocity_reflexxes.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

class JointJoggingAction final : public RtclActionInterface {
 public:
  // Real-time data type for fixed input parameters.
  struct FixedParams {
    JointLimits joint_limits;
  };

  // Real-time data type for streaming input parameters.
  struct StreamingParams {
    JointStateV goal;
  };

  static absl::StatusOr<std::unique_ptr<JointJoggingAction>> Create(
      const JointJoggingInfo::FixedParams& params_proto,
      ActionFactoryContext& context) INTRINSIC_NON_REALTIME_ONLY;

  RealtimeStatus OnEnter(OnEnterParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  RealtimeStatus Sense(SenseParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  RealtimeStatus Control(ControlParameters params)
      INTRINSIC_CHECK_REALTIME_SAFE override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const INTRINSIC_CHECK_REALTIME_SAFE override;

 private:
  struct InternalState {
    // The setpoint from the previous cycle. Used by the reflexxes interpolator.
    std::optional<JointStatePVA> previous_joint_command;

    // Cycles remaining until a Zero Velocity is commanded.
    int cycles_until_timeout = 0;
  };

  JointJoggingAction(RealtimeSlotId slot_id, size_t ndof, double frequency_hz,
                     StreamingInputId streaming_input_id,
                     FixedParams fixed_params,
                     JointVelocityReflexxes trajectory_generator,
                     JointVelocityReflexxes stop_generator)
      : slot_id_(slot_id),
        ndof_(ndof),
        frequency_hz_(frequency_hz),
        streaming_input_id_(streaming_input_id),
        fixed_params_(fixed_params),
        trajectory_generator_(trajectory_generator),
        stop_generator_(stop_generator) {}

  RealtimeSlotId slot_id_;
  size_t ndof_;
  double frequency_hz_;
  StreamingInputId streaming_input_id_;
  const FixedParams fixed_params_;
  JointVelocityReflexxes trajectory_generator_;
  JointVelocityReflexxes stop_generator_;

  // Cache of the currently set target velocity, which will be scaled down
  // within the Control function according to the speed_override value
  JointStateV target_velocity_;

  // The internal state of the Action. Reset when OnEnter is called.
  std::optional<InternalState> internal_state_;
  // Internal variable corresponding to the "timed_out" condition variable.
  std::optional<bool> timed_out_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_JOINT_JOGGING_ACTION_H_
