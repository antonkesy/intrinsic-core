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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_SESSION_MANAGER_H_
#define INTRINSIC_ICON_CONTROL_RTCL_SESSION_MANAGER_H_

#include <cstddef>
#include <optional>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_set.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_part_manager.h"
#include "intrinsic/icon/control/realtime_session_channels.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/control/rtcl_realtime_session.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

// This class encapsulates the behavior of an RtclSession:
//
// * How it reacts to inputs from a non-realtime thread via
//   RealtimeSessionChannels
// * How it handles Reactions, both those that switch Actions in realtime and
//   those that do not
// * How it handles failure at different points in the process
class RtclSessionManager {
 public:
  // Empty default constructor so that this class works more smoothly with
  // containers (vectors etc.)
  RtclSessionManager() INTRINSIC_CHECK_REALTIME_SAFE = default;

  // Creates a new RealtimeSessionData object that holds a pointer to
  // `channels`, and immediately sets its session status buffer to OK. It
  // receives `part_indices` and `context` to set the context of all parts used
  // by this session. It optionally receives a SafetyStatusMessage* which will
  // enable the reaction to safety events.
  //
  // On any error, whether it's induced externally via SetErrorCleanupNotify()
  // or encountered internally in one of the other functions, RtclSessionManager
  // * Stores the error in `channels`' status buffer
  // * Sets `channels.finish_done` to true
  // * Relinquishes the pointer (but does not attempt to free it, since it's
  //   non-owned)
  static RealtimeStatusOr<RtclSessionManager> Create(
      SessionId session_id,
      RealtimePartManager& part_manager ABSL_ATTRIBUTE_LIFETIME_BOUND,
      RealtimeSessionChannels& channels ABSL_ATTRIBUTE_LIFETIME_BOUND,
      const absl::flat_hash_set<size_t>& part_indices,
      const RealtimeLogContext& context,
      const intrinsic_fbs::SafetyStatusMessage* safety_status_message = nullptr)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Move-only, because the destructor does unique cleanup related to the
  // RealtimeSessionChannels pointer, which renders that pointer invalid.
  // Copying an RtclSessionManager introduces additional references to that
  // pointer, which are then dangerously dangling.
  RtclSessionManager(RtclSessionManager&& other) INTRINSIC_CHECK_REALTIME_SAFE;
  RtclSessionManager& operator=(RtclSessionManager&& other)
      INTRINSIC_CHECK_REALTIME_SAFE;

  RtclSessionManager(const RtclSessionManager&) = delete;
  RtclSessionManager& operator=(const RtclSessionManager&) = delete;

  // If the RtclSessionManager is active (see IsActive() below), then use our
  // RealtimeSessionChannels to signal that the Session has ended.
  ~RtclSessionManager() INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns the Session ID associated with this RtclSessionManager. This is
  // constant over the life of the RtclSessionManager object.
  SessionId GetSessionId() const INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns true if the RtclSessionManager currently has a
  // valid RealtimeSessionChannels pointer.
  bool IsActive() const INTRINSIC_CHECK_REALTIME_SAFE;

  bool IsReadOnly() const INTRINSIC_CHECK_REALTIME_SAFE {
    return part_indices_.empty();
  };

  FixedVector<size_t, kMaxRealtimeParts> GetPartIndices() const
      INTRINSIC_CHECK_REALTIME_SAFE {
    return part_indices_;
  };

  // Saves `status` in the RealtimeSessionChannel's status buffer, sets
  // finish_done to true, and deactivates this RtclSessionManager.
  //
  // No-op if RtclSessionManager is not active.
  void SetErrorCleanupNotify(RealtimeStatus status)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns a pointer to the first found, currently active Action for this
  // Session, or nullptr if there is none (or the RtclSessionManager is not
  // active).
  const RtclActionInstance* CurrentActiveActionTestOnly() const
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns a vector of pointers to the currently active Actions for this
  // Session, or an empty vector if there are none (or the RtclSessionManager is
  // not active).
  FixedVector<const RtclActionInstance*, kMaxRealtimeParts>
  CurrentActiveActionsTestOnly() const INTRINSIC_CHECK_REALTIME_SAFE;

  // Does all Action-related computations for the current control cycle (which
  // started at `cycle_start_time`). `speed_override` is the current speed
  // override value, a scalar between 0 and 1. `robot_status` needs to be the
  // most recent robot status.
  //
  // If the RtclSessionManager is active (see IsActive()), does the following:
  // 1. Checks for a "finish session command" and, if there is one, drops all
  //    references to resources owned by others. Then signals that the Session
  //    is done finishing.
  // 2. Checks for new RtclRealtimeSession data, and installs it if applicable.
  // 3a. If there is no active Action after step 2, returns OkStatus without
  //     doing any further work.
  // 3b. If there *is* an active Action:
  //     1. If the active Action is different from the previous cycle, call
  //        OnEnter() on the newly active Action.
  //     2. Calls Sense() on that Action, marking any Parts it uses as
  //        controlled by that Action in `part_manager_`.
  //     3. Evaluates any Reactions on the current Action. If there is a
  //        Reaction, send a Reaction event to the non-realtime thread. If the
  //        Reaction also changes the active Action, we call the newly active
  //        Action's OnEnter() and Sense() methods.
  //     4. Calls Control() on the active Action.
  //
  // Returns an error if any of these steps fail. Those errors automatically end
  // the Session, but are not necessarily fatal to the surrounding system.
  RealtimeStatus ExecuteCycle(Time cycle_start_time, double speed_override,
                              const AggregatedRobotStatus& robot_status)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns true, if the safety request should block session creation.
  bool IsSafetyBlockingSessionCreation() const INTRINSIC_CHECK_REALTIME_SAFE;

 private:
  friend class RtclSessionManagerTestPeer;
  SessionId session_id_;
  RealtimePartManager* part_manager_ = nullptr;
  RealtimeSessionChannels* channels_ = nullptr;
  RtclRealtimeSession* current_session_ = nullptr;
  // An entry in this vector contains the index of an active action or a
  // std::nullopt if the entry is unused. If an action is deactivated, the
  // vector entry will be set to std::nullopt again.
  //
  // The vector never changes size and is initialized to the size of the number
  // of parts. It is initialized to all std::nullopt, i.e. no action is active.
  // Unlike other vectors in RTCL, the order of values is arbitrary (put another
  // way, indices into this vector carry no additional meaning). Newly scheduled
  // actions will use the next suitable entry in the vector (see
  // `RtclReactionManager::AssignActionToSuitableActionEntry()`).
  FixedVector<std::optional<size_t>, kMaxRealtimeParts> active_action_indices_;
  RealtimeStatus session_status_ = OkStatus();
  const intrinsic_fbs::SafetyStatusMessage* safety_status_message_ = nullptr;
  FixedVector<size_t, kMaxRealtimeParts> part_indices_;

  RtclSessionManager(
      SessionId session_id,
      RealtimePartManager& part_manager ABSL_ATTRIBUTE_LIFETIME_BOUND,
      RealtimeSessionChannels& channels ABSL_ATTRIBUTE_LIFETIME_BOUND,
      const absl::flat_hash_set<size_t>& part_indices,
      const intrinsic_fbs::SafetyStatusMessage* safety_status_message = nullptr)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Runs the Sense() method for the currently active Action, if any.
  // If the Sense() method returns an error, saves the error and deactivates the
  // RtclSessionManager.
  // E.g. Sense() returns an error when the currently active action does not
  // support the `behavior_override_request`.
  RealtimeStatus RunSense(Time cycle_start_time, double speed_override,
                          intrinsic_proto::icon::v1::BehaviorOverrideRequest
                              behavior_override_request)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Runs the Control() method for the currently active Action, if any.
  // If the Control() method returns an error, saves the error and deactivates
  // the RtclSessionManager.
  // E.g. Control() returns an error when the currently active action does not
  // support the `behavior_override_request`.
  RealtimeStatus RunControl(double speed_override,
                            intrinsic_proto::icon::v1::BehaviorOverrideRequest
                                behavior_override_request)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Evaluates the Conditions for any Reactions in the currently installed
  // RtclRealtimeSession.
  // If a Condition evaluates to "true":
  // * Sends an event for the corresponding Reaction to the non-realtime thread
  //   using `channels_` if the Condition did not match in the previous cycle.
  //   This means if a Reaction's Condition *stays* true for consecutive cycles,
  //   the Reaction is only triggered once.
  // * If the Reaction is a realtime Reaction, runs OnEnter() and Sense() for
  //   the newly active Action.
  //
  // If there is more than one realtime Reaction, only one of them is
  // triggered/reported. Which one is undefined.
  //
  // Returns any errors that happen during this process, whether from evaluation
  // Conditions, or from calling OnEnter() / Sense().
  // E.g. OnEnter() / Sense() return an error when the currently active action
  // does not support the `behavior_override_request`.
  RealtimeStatus HandleReactions(
      Time cycle_start_time, double speed_override,
      intrinsic_proto::icon::v1::BehaviorOverrideRequest
          behavior_override_request,
      const AggregatedRobotStatus& robot_status) INTRINSIC_CHECK_REALTIME_SAFE;

  // Checks `channels_.finish_requested`, and if it is true, calls
  // CleanupAndNotify() below.
  void FinishSessionIfRequested() INTRINSIC_CHECK_REALTIME_SAFE;

  // Handles an outstanding call to `channels_.install_session_data_bridge`, if
  // any, by:
  // * Discarding any previous RtclRealtimeSession pointer
  // * Switching to a new Action, if requested, by calling OnEnter() on the new
  //   active Action
  // Returns any errors encountered during OnEnter().
  RealtimeStatus HandleNewSessionData(
      double speed_override,
      intrinsic_proto::icon::v1::BehaviorOverrideRequest
          behavior_override_request) INTRINSIC_CHECK_REALTIME_SAFE;

  // Actual implementation of HandleNewSessionData().
  RealtimeStatus HandleNewSessionDataImpl(
      RtclRealtimeSession* new_session, double speed_override,
      intrinsic_proto::icon::v1::BehaviorOverrideRequest
          behavior_override_request) INTRINSIC_CHECK_REALTIME_SAFE;

  // Discards the current `channels_` pointer, and sets its `finish_done` member
  // to true.
  void CleanupAndNotify() INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns true, if safety requests a SAFE_STOP_1_TIME_MONITORED.
  bool IsSafetyRequestingToTerminateSession() const
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns false.
  // Returns true if safety requests PAUSE, or SAFE_STOP_2_TIME_MONITORED.
  bool IsSafetyRequestingToPauseCurrentSession() const
      INTRINSIC_CHECK_REALTIME_SAFE;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_SESSION_MANAGER_H_
