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

#ifndef INTRINSIC_ICON_CONTROL_REACTION_H_
#define INTRINSIC_ICON_CONTROL_REACTION_H_

#include <cstdint>
#include <optional>

#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/realtime_condition.h"
#include "intrinsic/icon/proto/v1/condition_types.pb.h"

namespace intrinsic::icon {

// Defines a reaction that fires when
// 1) the current action is 'from_action_index' and 'condition' matches.
// or 2) If `from_action_index` is not set and 'condition' matches.
//
// Used for both realtime switching and notifying the action session.
struct RealtimeReaction {
  // Describes the association of the reaction to an action. If not set, the
  // reaction is a free-standing reaction that fires when the condition matches.
  std::optional<int64_t> from_action_index = std::nullopt;
  RealtimeCondition condition{
      .elements = {RealtimeComparison{
          /*operand =*/{}, /*required_part_index =*/{},
          /*operation =*/intrinsic_proto::icon::v1::Comparison::EQUAL,
          /*value =*/StateVariableValue(false),
          /*max_abs_error=*/kDefaultMaxAbsError}}};
  // Only used for non-RT notifications and logging.
  ReactionId id;
  // Action index to switch to or action to start in parallel, depending on
  // `stop_associated_action`. If not set, no action switch/start will happen.
  // TODO(b/278083799): Rename this variable to align with new naming of
  // related variables.
  std::optional<int64_t> target_action_index = std::nullopt;
  // Used to only signal the non-real-time thread on rising edges of Reaction
  // triggering. When the Reaction fires, this variable is set to true.
  // This value is reset to false, when the state switches from true to false or
  // when the Reaction becomes active again (i.e. the associated Action becomes
  // active again). Note that a Reaction that switches to the same Action that
  // is currently running counts as that Action momentarily becoming inactive.
  bool fired = false;
  // If true, the reaction will only trigger once as long as the
  // associated action is active. It can trigger again if the action is executed
  // again. If the reaction is free-standing, it will only trigger once.
  //
  // If false, the reaction will trigger on every rising edge again.
  bool fire_once = false;
  // If set to true, the active, associated action will be stopped. If set to
  // false, this action will remain active if the `target_action_index` does not
  // use an overlapping part set. However, having this overlap is prevented by a
  // check in `VerifySessionData()` since it does not make sense to request to
  // start a parallel action that preempts the action associated with the
  // reaction.
  bool stop_associated_action = true;
  // Signal to be triggered from false to true in the case that the reaction
  // should fire. If unset no signals are triggered. Only a single real-time
  // signal may be triggered.
  std::optional<int64_t> triggered_signal_id = std::nullopt;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REACTION_H_
