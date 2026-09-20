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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_JOINT_IMPEDANCE_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_JOINT_IMPEDANCE_ACTION_H_

#include <cstddef>
#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/joint_impedance_action_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
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

class JointImpedanceAction final : public RtclActionInterface {
 public:
  // Real-time data type for both fixed and streaming input parameters.
  struct Params {
    JointStatePV target_state;
    std::optional<JointStateT> feedforward_torque;
    eigenmath::VectorNd joint_stiffness;
    eigenmath::VectorNd joint_damping;
    JointLimits joint_limits;
  };

  JointImpedanceAction(
      RealtimeSlotId slot_id, StreamingInputId streaming_input_id, size_t ndof,
      double frequency_hz, Params initial_params,
      std::unique_ptr<IsSettledCriterion> is_settled_criterion);

  static absl::StatusOr<std::unique_ptr<JointImpedanceAction>> Create(
      JointImpedanceInfo::FixedParams params_proto,
      ActionFactoryContext& context);

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

  // Params `application_limits` are before `params` so they can be bound using
  // `absl::bind_front`. Since `params` is a streaming input, it is only
  // available at a later stage. `absl::bind_front` binds arguments from left to
  // right, so `application_limits` must come first to be bound. The
  // application_limits are used to check that any limits provided in the
  // streaming input are within the application limits.
  static absl::StatusOr<Params> ParseInput(
      const JointLimits& application_limits,
      const JointImpedanceInfo::StreamingParams& params);

 private:
  // This threshold for the speed override value defines where the
  // reflexxes-based trajectory generator switches from a position to a velocity
  // based. Between 0 and the threshold a velocity-based trajectory generator is
  // used and simply drives the reference to zero velocity.
  static constexpr double kSwitchSpeedOverride = 0.01;

  // This method updates the trajectory generators based on target_state and
  // limits.
  RealtimeStatus UpdateGoal(const JointStatePV& target_state,
                            const JointLimits& limits);

  RealtimeSlotId slot_id_;
  StreamingInputId streaming_input_id_;
  size_t ndof_;
  JointPositionReflexxes trajectory_generator_position_;
  JointVelocityReflexxes trajectory_generator_velocity_;
  Params current_params_;
  const Params initial_params_;

  // The setpoint from the previous cycle. Used by the reflexxes interpolator.
  // we continuously interpolate towards the target joint state (or target joint
  // state) at real time, the result of this interpolation is then used as the
  // reference in the impedance control law.
  std::optional<JointStatePVA> previous_setpoint_;
  std::optional<JointStatePVA> joint_state_sensed_;
  std::unique_ptr<IsSettledCriterion> is_settled_criterion_;

  // Variables used to track action state variables. These are updated in
  // Sense().
  bool is_done_ = false;     // Corresponds to kIsDone
  bool is_settled_ = false;  // Corresponds to kIsSettled
  std::optional<double>
      distance_to_target_;  // Corresponds to kDistanceToSensed
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_JOINT_IMPEDANCE_ACTION_H_
