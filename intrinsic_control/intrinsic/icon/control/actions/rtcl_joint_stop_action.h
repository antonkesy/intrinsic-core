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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_RTCL_JOINT_STOP_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_RTCL_JOINT_STOP_ACTION_H_

#include <cstddef>
#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/algorithms/joint_velocity_reflexxes.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

class JointStopAction final : public RtclActionInterface {
 public:
  JointStopAction() = delete;

  static absl::StatusOr<std::unique_ptr<JointStopAction>> Create(
      ActionFactoryContext& context);

  static intrinsic_proto::icon::v1::ActionSignature GetStopSignature();

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

 private:
  JointStopAction(RealtimeSlotId slot_id, size_t ndof, double frequency_hz,
                  std::unique_ptr<IsSettledCriterion> is_settled_criterion,
                  bool use_joint_jerk_limits);
  RealtimeSlotId slot_id_;
  size_t ndof_;
  double frequency_hz_;
  JointVelocityReflexxes trajectory_generator_;
  // The setpoint from the previous cycle. Used by the reflexxes interpolator.
  std::optional<JointStatePVA> previous_setpoint_;
  // Saves the "done" status of the Action between calls to Control() and
  // Sense().
  bool done_buffer_ = false;
  // Internal variable corresponding to the "done" condition variable. Only
  // changes values when Sense() is called.
  bool done_ = false;
  // Internal variables corresponding to the evaluation of the settled state.
  bool is_settled_ = false;
  // Internal variable corresponding to the elapsed time since the action
  // started. It is increased by one cycle in every Sense() call.
  double elapsed_time_seconds_ = 0.0;
  std::unique_ptr<IsSettledCriterion> is_settled_criterion_;
  bool use_joint_jerk_limits_ = true;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_RTCL_JOINT_STOP_ACTION_H_
