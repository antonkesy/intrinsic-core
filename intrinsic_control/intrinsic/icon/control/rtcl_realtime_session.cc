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

#include "intrinsic/icon/control/rtcl_realtime_session.h"

#include <array>
#include <cstddef>
#include <optional>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/base/optimization.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

RealtimeStatusOr<RtclActionInstance*> GetActionFromSession(
    const RtclRealtimeSession& session, size_t action_index) {
  if (ABSL_PREDICT_FALSE(action_index >= session.actions.size())) {
    return OutOfRangeError(RealtimeStatus::StrCat(
        "Action index ", action_index,
        " is out of range. Range: ", session.actions.size()));
  }
  RtclActionInstance* action_pointer = session.actions[action_index];
  if (ABSL_PREDICT_FALSE(!action_pointer)) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "Requested action pointer is null for index ", action_index));
  }
  return action_pointer;
}

RealtimeStatusOr<std::optional<std::pair<size_t, size_t>>> GetFirstSlotOverlap(
    const RtclRealtimeSession& session,
    absl::Span<const size_t> action_indices) {
  std::array<std::optional<size_t>, kMaxRealtimeParts> slot_usage{std::nullopt};
  for (size_t index : action_indices) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const RtclActionInstance* next_action,
                                  GetActionFromSession(session, index));
    if (next_action->visibility_by_slot_id->size() > slot_usage.size()) {
      return InternalError(RealtimeStatus::StrCat(
          "visibility_by_slot_id of action ", next_action->id.value(),
          " is larger than the capacity of slot_usage. This is a bug"));
    }

    for (size_t i = 0; i < next_action->visibility_by_slot_id->size(); ++i) {
      if (next_action->visibility_by_slot_id->at(i)) {
        if (slot_usage[i].has_value()) {
          if (*slot_usage[i] != index) {
            return std::optional<std::pair<size_t, size_t>>{
                std::make_pair(*slot_usage[i], index)};
          } else {
            // The same action is given multiple times, which is allowed.
          }
        } else {
          // Save that slot i is used by this action.
          slot_usage[i] = index;
        }
      }
    }
  }
  return std::optional<std::pair<size_t, size_t>>{};
}

absl::Status VerifySessionData(const RtclRealtimeSession& session) {
  for (RtclActionInstance* const action : session.actions) {
    if (action == nullptr) {
      continue;
    }
    if (action->action != nullptr && action->visibility_by_slot_id == nullptr) {
      return absl::InternalError(
          "Action is set in RtclRealtimeSession but visibility_by_slot_id is "
          "null.");
    }
    if (action->action == nullptr && action->visibility_by_slot_id != nullptr) {
      return absl::InternalError(
          "Action is null in RtclRealtimeSession but visibility_by_slot_id is "
          "set.");
    }
    if (action->action == nullptr && action->visibility_by_slot_id == nullptr) {
      return absl::InternalError(
          absl::StrCat("Action and visibility_by_slot_id are null in "
                       "RtclRealtimeSession for action with ID ",
                       action->id.value()));
    }
  }

  // Check that all actions to start have indices that point to valid actions.
  for (size_t action_index : session.actions_to_start) {
    const RealtimeStatus action_found =
        GetActionFromSession(session, action_index).status();
    if (!action_found.ok()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Requested action to start with index ", action_index,
                       " does not exist."));
    }
  }

  auto sorted_actions_to_start = session.actions_to_start;
  absl::c_sort(sorted_actions_to_start);
  if (auto it = absl::c_adjacent_find(sorted_actions_to_start);
      it != sorted_actions_to_start.end()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto action,
                                  GetActionFromSession(session, *it));
    return absl::InvalidArgumentError(absl::StrCat(
        "Duplicated action to start found with ID: ", action->id.value()));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const auto slot_overlap,
      GetFirstSlotOverlap(session, session.actions_to_start));
  if (slot_overlap.has_value()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Actions to start with IDs ", slot_overlap->first, " and ",
                     slot_overlap->second, " have overlapping slots."));
  }

  if (session.reactions.size() > RtclRealtimeSession::kMaxReactionsPerSession) {
    return absl::InvalidArgumentError(
        absl::StrCat("Too many reactions in session. Maximum allowed: ",
                     RtclRealtimeSession::kMaxReactionsPerSession,
                     ", actual: ", session.reactions.size()));
  }

  for (const RealtimeReaction& reaction : session.reactions) {
    if (reaction.from_action_index.has_value() &&
        reaction.target_action_index.has_value() &&
        !reaction.stop_associated_action) {
      // Check that a reaction that is marked with non-stopping does not
      // switch from an action to an action that uses an overlapping part set.
      FixedVector<size_t, 2> action_indices = {
          static_cast<size_t>(*reaction.from_action_index),
          static_cast<size_t>(*reaction.target_action_index)};
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          const auto slot_overlap,
          GetFirstSlotOverlap(session, action_indices));
      if (slot_overlap.has_value()) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Reaction (ID ", reaction.id.value(),
            ") with the actions with IDs ", slot_overlap->first, " and ",
            slot_overlap->second,
            " have overlapping slots, but the target action ",
            slot_overlap->second, " was configured to run in parallel."));
      }
    }
  }
  return absl::OkStatus();
}
}  // namespace intrinsic::icon
