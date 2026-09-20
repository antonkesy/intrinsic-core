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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_SIMPLE_GRIPPER_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_SIMPLE_GRIPPER_ACTION_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/simple_gripper_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Action to be used with a part implementing the SimpleGripper Feature
// Interface.
// The SimpleGripperAction and SimpleGripper Interface implement the most basic
// functionality of any gripper. It is possible to command grasp/release, as
// well as get basic status information (unknown, grasped, released) via the
// Action's StateVariables. See
// intrinsic/icon/actions/simple_gripper_info.h
class SimpleGripperAction final : public RtclActionInterface {
 public:
  explicit SimpleGripperAction(RealtimeSlotId slot_id,
                               SimpleGripper::GripperCommand gripper_command)
      : slot_id_(slot_id), gripper_command_(gripper_command) {}

  static absl::StatusOr<std::unique_ptr<SimpleGripperAction>> Create(
      const SimpleGripperActionInfo::FixedParams& params_proto,
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
  // Internal variable corresponding to the 'kIsDone' condition variable.
  bool done_ = false;
  // Internal variable corresponding to the
  // 'SimpleGripperActionInfo::kSentCommand' condition variable.
  bool sent_command_ = false;
  // Saves the "sent_command_" status of the Action between calls to Control()
  // and Sense().
  bool sent_command_buffer_ = false;

  RealtimeSlotId slot_id_;
  const SimpleGripper::GripperCommand gripper_command_;

  // Internal variable corresponding to the "sensed" state of the Gripper. Only
  // changes values when Sense() is called.
  SimpleGripper::GripperState gripper_state_ =
      SimpleGripper::GripperState::kUnknown;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_SIMPLE_GRIPPER_ACTION_H_
