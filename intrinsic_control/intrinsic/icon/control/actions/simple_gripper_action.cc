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

#include "intrinsic/icon/control/actions/simple_gripper_action.h"

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/simple_gripper.pb.h"
#include "intrinsic/icon/actions/simple_gripper_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

using ::intrinsic_proto::icon::actions::proto::SimpleGripperFixedParams;
using GripperCommand = ::intrinsic::icon::SimpleGripper::GripperCommand;
using GripperState = ::intrinsic::icon::SimpleGripper::GripperState;

namespace {

absl::StatusOr<GripperCommand> FromProto(
    const SimpleGripperActionInfo::FixedParams& proto) {
  INTRINSIC_ASSERT_NON_REALTIME();
  switch (proto.command()) {
    case SimpleGripperFixedParams::GRASP:
      return GripperCommand::kGrasp;
    case SimpleGripperFixedParams::RELEASE:
      return GripperCommand::kRelease;
    default:
      return absl::InvalidArgumentError(absl::StrFormat(
          "'%s' is not a valid command for the SimpleGripperAction.",
          SimpleGripperFixedParams::Command_Name(proto.command())));
  }
}

}  // namespace

// static
absl::StatusOr<std::unique_ptr<SimpleGripperAction>>
SimpleGripperAction::Create(
    const SimpleGripperActionInfo::FixedParams& params_proto,
    ActionFactoryContext& context) {
  INTR_ASSIGN_OR_RETURN(
      SlotInfo slot_info,
      context.GetSlotInfo(SimpleGripperActionInfo::kSlotName));

  INTR_ASSIGN_OR_RETURN(GripperCommand command, FromProto(params_proto));

  return std::make_unique<SimpleGripperAction>(slot_info.slot_id,
                                               std::move(command));
}

RealtimeStatus SimpleGripperAction::OnEnter(OnEnterParameters params) {
  done_ = false;
  sent_command_buffer_ = false;

  return OkStatus();
}

RealtimeStatus SimpleGripperAction::Sense(SenseParameters params) {
  if (gripper_command_ == GripperCommand::kUnknown) {
    return FailedPreconditionError(
        "SimpleGripperAction::Sense() called with "
        "'GripperCommand::kUnknown'. This means no command is set.");
  }

  const SimpleGripper* const gripper_interface =
      params.slot_map.GetInterfaceForSlot<SimpleGripper>(slot_id_);
  if (gripper_interface == nullptr) {
    return InternalError("Slot doesn't have SimpleGripper.");
  }

  gripper_state_ = gripper_interface->GetGripperState();
  sent_command_ = sent_command_buffer_;

  done_ = (gripper_state_ == GripperState::kGrasped &&
           gripper_command_ == GripperCommand::kGrasp) ||
          (gripper_state_ == GripperState::kReleased &&
           gripper_command_ == GripperCommand::kRelease);

  return OkStatus();
}

RealtimeStatus SimpleGripperAction::Control(ControlParameters params) {
  if (gripper_command_ == GripperCommand::kUnknown) {
    return FailedPreconditionError(
        "SimpleGripperAction::Control() called with "
        "'GripperCommand::kUnknown'. This means no command is set.");
  }

  SimpleGripper* const gripper_interface =
      params.slot_map.GetMutableInterfaceForSlot<SimpleGripper>(slot_id_);
  if (gripper_interface == nullptr) {
    return InternalError("Slot doesn't have SimpleGripper.");
  }

  RealtimeStatus status =
      gripper_interface->SetGripperCommand(gripper_command_);
  if (status == OkStatus()) {
    sent_command_buffer_ = true;
  }
  return status;
}

RealtimeStatusOr<StateVariableValue> SimpleGripperAction::GetStateVariable(
    absl::string_view name) const {
  if (name == kIsDone) {
    return StateVariableValue(done_);
  }
  if (name == SimpleGripperActionInfo::kSentCommand) {
    return StateVariableValue(sent_command_);
  }
  if (name == SimpleGripperActionInfo::kGrasped) {
    return StateVariableValue(gripper_state_ == GripperState::kGrasped);
  }
  if (name == SimpleGripperActionInfo::kReleased) {
    return StateVariableValue(gripper_state_ == GripperState::kReleased);
  }
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      "SimpleGripperAction, state variable not found ", name));
}

}  // namespace intrinsic::icon
