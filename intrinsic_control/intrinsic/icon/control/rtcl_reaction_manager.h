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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_REACTION_MANAGER_H_
#define INTRINSIC_ICON_CONTROL_RTCL_REACTION_MANAGER_H_

#include <cstddef>
#include <optional>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/types/span.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_part_manager.h"
#include "intrinsic/icon/control/realtime_session_channels.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/control/rtcl_realtime_session.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

// The RtclReactionManager class handles reactions by evaluating their
// conditions and by executing the responses associated with them.
class RtclReactionManager {
 public:
  // Creates the reaction manager. Usually this should be temporary object since
  // most parameters of the constructor must outlive this instance.
  // `active_action_indices` should contain the real-time action indices of the
  // currently active actions. HandleReactions() and StartActions() can update
  // this array.
  RtclReactionManager(
      RealtimePartManager* part_manager ABSL_ATTRIBUTE_LIFETIME_BOUND,
      RealtimeSessionChannels* channels ABSL_ATTRIBUTE_LIFETIME_BOUND,
      RtclRealtimeSession* current_session ABSL_ATTRIBUTE_LIFETIME_BOUND,
      FixedVector<std::optional<size_t>, kMaxRealtimeParts>*
          active_action_indices ABSL_ATTRIBUTE_LIFETIME_BOUND)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Handles the reactions stored in the `current_session_` using
  // `robot_status` for condition evaluation.
  RealtimeStatus HandleReactions(
      Time cycle_start_time, double speed_override,
      intrinsic_proto::icon::v1::BehaviorOverrideRequest
          behavior_override_request,
      const AggregatedRobotStatus& robot_status) INTRINSIC_CHECK_REALTIME_SAFE;

  // Starts all actions given by `action_indices`. Active actions, which have an
  // overlapping slot set with one of the actions to start, will be deactivated.
  // Forwards the first encountered error while calling `OnEnter()` of the
  // actions referenced by `action_indices`.
  //
  // Returns `AbortedError` if the action does not support the
  // `behavior_override_request`.
  RealtimeStatus StartActions(
      absl::Span<const size_t> action_indices, double speed_override,
      intrinsic_proto::icon::v1::BehaviorOverrideRequest
          behavior_override_request) INTRINSIC_CHECK_REALTIME_SAFE;

 private:
  // Assign `action_index_to_start` at the first suitable entry in
  // `active_action_indices_`. Suitable entries are (checked in this order):
  //  1. Entry at which the given action is already running
  //  2. Unused entry
  //
  // Returns an error if no suitable entry is found or if an action with an
  // overlapping part slot set is active.
  RealtimeStatus AssignActionToSuitableActionEntry(size_t action_index_to_start)
      INTRINSIC_CHECK_REALTIME_SAFE;

  RealtimeStatusOr<RtclActionInstance*> GetAction(size_t action_index) const
      INTRINSIC_CHECK_REALTIME_SAFE {
    return GetActionFromSession(*current_session_, action_index);
  }

  // Deactivates an action by finding the action referenced by
  // `action_index_to_deactivate` and calling `DeactivateAction()`.
  bool DeactivateActionByIndex(size_t action_index_to_deactivate)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Deactivates the given action so that it is not active anymore and
  // Sense()/Control() are not called anymore for this action until it get
  // activated again.
  void DeactivateAction(RtclActionInstance& action_instance)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Evaluates the condition of `reaction` using the action state variables of
  // `current_action` and state variables of `robot_status`.
  // `from_action_id` refers to the action referenced by
  // `reaction.from_action_index`. If `reaction.from_action_index` is set,
  // `from_action_id` must be set as well and `action` must be this referenced
  // action. If `reaction.from_action_index` is not set, `action` must be an
  // EmptyAction instance.
  //
  // If the reaction triggers, the non-rt event is sent and the function
  // returns true.
  RealtimeStatusOr<bool> EvaluateReactionAndSendEvent(
      RealtimeReaction& reaction, const RtclActionInterface& action,
      const AggregatedRobotStatus& robot_status,
      std::optional<ActionInstanceId> from_action_id)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Checks reaction effects for validity (i.e. the same slot cannot be used by
  // two different reaction's actions).
  RealtimeStatus CheckReactionEffects(
      const FixedVector<size_t, kMaxRealtimeParts>& triggered_reactions)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Starts and deactivates actions based on `triggered_reactions`.
  // Calls `OnEnter()` and `Sense()` for all activated actions.
  RealtimeStatus ExecuteRtReactions(
      Time cycle_start_time, double speed_override,
      intrinsic_proto::icon::v1::BehaviorOverrideRequest
          behavior_override_request,
      absl::Span<const size_t> triggered_reactions)
      INTRINSIC_CHECK_REALTIME_SAFE;

  RealtimePartManager* part_manager_ = nullptr;
  RealtimeSessionChannels* channels_ = nullptr;
  RtclRealtimeSession* current_session_ = nullptr;
  FixedVector<std::optional<size_t>, kMaxRealtimeParts>* active_action_indices_;
};
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_REACTION_MANAGER_H_
