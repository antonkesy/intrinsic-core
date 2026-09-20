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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_REALTIME_SESSION_H_
#define INTRINSIC_ICON_CONTROL_RTCL_REALTIME_SESSION_H_

#include <cstddef>
#include <optional>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/container/fixed_array.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

struct RtclRealtimeSession {
  // While the number of actions has no real-time performance impact, and the
  // number of active actions is already limited by the number of parts,
  // reactions and their real-time conditions are evaluated every tick and need
  // to be limited.
  static constexpr int kMaxReactionsPerSession = 100;

  SessionId session_id;
  // If true, stop all active actions in this session upon installing it in the
  // RT thread. If false, they continue to run if they are not preempted by
  // actions that are requested to start by `actions_to_start`.
  bool stop_active_actions = true;
  // If set, the realtime thread should start the Actions at the given indices
  // when it receives this RtclRealtimeSession.
  absl::FixedArray<size_t> actions_to_start = {};

  // These pointers are not owned – their lifetimes are managed by
  // RtclControllerSession.
  //
  // Indices into this vector are never invalidated. Instead of removing
  // elements, the corresponding pointer is set to nullptr. This makes it
  // trivial for the realtime thread to efficiently maintain the current active
  // Action, without having to use a hashmap with potentially unpredictable
  // lookup times.
  absl::FixedArray<RtclActionInstance*> actions = {};
  // RealtimeReactions are lightweight enough that we can copy them instead of
  // passing pointers.
  absl::FixedArray<RealtimeReaction> reactions = {};
};

// Convenience function to get an action instance from a session safely. The
// returned pointer is always non-null. `session` must outlive the returned
// pointer.
RealtimeStatusOr<RtclActionInstance*> GetActionFromSession(
    const RtclRealtimeSession& session ABSL_ATTRIBUTE_LIFETIME_BOUND,
    size_t action_index) INTRINSIC_CHECK_REALTIME_SAFE;

// Gets efficiently the first found actions with overlapping part slot sets. If
// the function did not find any overlaps, returns std::nullopt.
//
// Index duplicates in `action_indices` are tolerated and do not cause a slot
// overlap.
//
// Run-time complexity is O(number of action indices * number of slots).
RealtimeStatusOr<std::optional<std::pair<size_t, size_t>>> GetFirstSlotOverlap(
    const RtclRealtimeSession& session,
    absl::Span<const size_t> action_indices) INTRINSIC_CHECK_REALTIME_SAFE;

// Performs some session data verification checks. There might still be errors
// in the session data, which can only be discovered during the runtime of the
// session.
absl::Status VerifySessionData(const RtclRealtimeSession& session)
    INTRINSIC_NON_REALTIME_ONLY;

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_REALTIME_SESSION_H_
