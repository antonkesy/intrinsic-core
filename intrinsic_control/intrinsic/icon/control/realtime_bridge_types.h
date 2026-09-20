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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_BRIDGE_TYPES_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_BRIDGE_TYPES_H_

#include <cstddef>
#include <variant>

#include "absl/time/time.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/parts/realtime_part_status.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

// This has been set to 8 to mitigate segmentation faults caused by stack size.
// Keep this in mind if you need to increase this value.
static constexpr size_t kMaxRealtimeParts = 8;

// Realtime safe representation of intrinsic_proto::icon::SafetyStatus
// (intrinsic/icon/proto/safety_status.proto). Matches
// intrinsic/icon/control/safety/safety_messages.fbs
// Initialized matching the proto and flatbuffer defaults.
struct SafetyStatus {
  intrinsic_fbs::ModeOfSafeOperation mode_of_safe_operation =
      intrinsic_fbs::ModeOfSafeOperation::UNKNOWN;
  intrinsic_fbs::ButtonStatus estop_button_status =
      intrinsic_fbs::ButtonStatus::UNKNOWN;
  intrinsic_fbs::ButtonStatus enable_button_status =
      intrinsic_fbs::ButtonStatus::UNKNOWN;
  intrinsic_fbs::RequestedBehavior requested_behavior =
      intrinsic_fbs::RequestedBehavior::UNKNOWN;

  friend bool operator==(const SafetyStatus& lhs, const SafetyStatus& rhs) {
    return lhs.mode_of_safe_operation == rhs.mode_of_safe_operation &&
           lhs.estop_button_status == rhs.estop_button_status &&
           lhs.enable_button_status == rhs.enable_button_status &&
           lhs.requested_behavior == rhs.requested_behavior;
  }
};

// Aggregates the part status of all parts and the safety status into one
// struct.
struct AggregatedRobotStatus {
  // The part statuses of all parts of the current ICON instance.
  intrinsic::FixedVector<RealtimePartStatus, kMaxRealtimeParts> part_statuses;
  // The publicly visible status of the safety implementation. Exported as proto
  // defaults if not populated.
  SafetyStatus safety_status;
};
// This is a rough estimate of the size of the AggregatedRobotStatus for which
// it has been tested that we do not reach stack size limits. You may encounter
// segfault if this assertion fails.
static_assert(sizeof(AggregatedRobotStatus) <= 486736,
              "The size of the AggregatedRobotStatus is too large.");

// Stores status messages to be passed from realtime to non-realtime.
struct PublishOutput {
  // System time of the control cycle at which the PublishOutput was
  // generated.
  //
  // Since the system time is synchronized via NTP, the monotonic time in
  // PartStatus.timestamp_ns is a more accurate reference when comparing
  // Part status messages from the same ICON instance. The system time is a more
  // useful reference against events that occurred on a different node. Likely
  // has issues with e.g. `smear seconds`. More information in part_status.proto
  // and b/207633644.
  //
  // Explicitly constructing wall_time so it's clear that the default value is
  // the Unix epoch, even though it should not be used uninitialized. Not using
  // absl::Now() to minimize calls to the clock.
  absl::Time wall_time = absl::UnixEpoch();
  // Timestamp from the monotonic clock.
  absl::Duration timestamp_control;
  uint64_t cycle = 0;
  AggregatedRobotStatus robot_status;
  intrinsic::FixedVector<RealtimeLogContext, kMaxRealtimeParts> part_contexts;
};

// Notification event that a RealtimeSession failed.
struct SessionFailure {
  SessionId session_id;
  RealtimeStatus error;
};

// Generic notification from realtime thread to ICON server.
using RealtimeEvent = std::variant<ReactionEvent, SessionFailure>;

struct StartSessionRequest {
  SessionId session_id;
  FixedVector<size_t, kMaxRealtimeParts> part_indices;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_BRIDGE_TYPES_H_
