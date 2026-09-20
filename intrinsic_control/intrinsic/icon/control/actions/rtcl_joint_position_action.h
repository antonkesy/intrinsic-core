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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_RTCL_JOINT_POSITION_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_RTCL_JOINT_POSITION_ACTION_H_

#include <any>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "intrinsic/icon/actions/point_to_point_move_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/algorithms/joint_position_reflexxes.h"
#include "intrinsic/icon/control/algorithms/joint_velocity_reflexxes.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

class JointPositionAction final : public RtclActionInterface {
 public:
  struct Params {
    JointStatePV joint_goal;
    intrinsic::JointLimits joint_limits;
  };

  JointPositionAction(RealtimeSlotId slot_id, size_t ndof, double frequency_hz,
                      Params params,
                      std::unique_ptr<IsSettledCriterion> is_settled_criterion);

  static absl::StatusOr<std::unique_ptr<JointPositionAction>> Create(
      const PointToPointMoveInfo::FixedParams& param_proto,
      ActionFactoryContext& context);

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

 private:
  // This threshold for the speed override value defines where the
  // reflexxes-based trajectory generator switches from a position to a velocity
  // based. Between 0 and the threshold a velocity-based controller is used and
  // between the threshold and 1 a position-based controller is used.
  static constexpr double kSwitchSpeedOverride = 0.01;

  struct StateVariables {
    bool done = false;
    bool settled = false;
    double seconds_since_start = 0.0;
    std::optional<double> distance_to_target = std::nullopt;
    // Only set when done_ == true
    std::optional<double> seconds_when_done = std::nullopt;
  };

  RealtimeSlotId slot_id_;
  size_t ndof_;
  const Params params_;
  JointPositionReflexxes trajectory_generator_position_;
  JointVelocityReflexxes trajectory_generator_velocity_;
  // The current position goal, as set by the last call to Command().
  std::optional<JointStatePV> current_goal_;
  // The setpoint from the previous cycle. Used by the reflexxes interpolator.
  std::optional<JointStatePVA> previous_setpoint_;
  // Saves the "done" status of the Action between calls to Control() and
  // Sense().
  bool done_buffer_ = false;
  // Internal variable corresponding to the Action's state variables. Only
  // changes values when Sense() is called.
  std::optional<StateVariables> state_variables_;
  double frequency_hz_;
  // Internal variable corresponding to the evaluator of the settled state.
  std::unique_ptr<IsSettledCriterion> is_settled_criterion_;
};

// Converts `param_proto` to the realtime-safe data type used by
// JointPositionAction.
//
// Returns an error if `param_proto` contains an invalid command (such as
// exceeding the limits in `part_config`).
absl::StatusOr<JointPositionAction::Params> FromProto(
    const PointToPointMoveInfo::FixedParams& param_proto,
    const ::intrinsic_proto::icon::GenericPartConfig& part_config);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_RTCL_JOINT_POSITION_ACTION_H_
