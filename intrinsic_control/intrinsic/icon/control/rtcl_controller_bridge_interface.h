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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_BRIDGE_INTERFACE_H_
#define INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_BRIDGE_INTERFACE_H_

#include <memory>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/parts/realtime_part_status.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/rtcl_session_bridge_interface.h"

namespace intrinsic::icon {

struct RealtimePartStatuses {
  // System time of the control cycle at which the PublishOutput was
  // generated (see intrinsic/icon/control/realtime_bridge_types.h).
  absl::Time wall_time;
  absl::flat_hash_map<std::string, RealtimePartStatus> status_by_part_name;
};

// SafetyStatus  with an additional timestamp.
struct RealtimeSafetyStatus {
  // System time of the control cycle at which the RealtimeSafetyStatus was
  // generated.
  absl::Time wall_time;
  SafetyStatus safety_status;
};

// This virtual class defines the interface between an RtclController and the
// (gRPC) non-realtime thread communicating with it.
class RtclControllerBridgeInterface {
 public:
  virtual ~RtclControllerBridgeInterface() = default;

  // Returns the latest part statuses. Note that this means multiple calls may
  // return the same result (if the realtime thread has not published updated
  // status data in between).
  //
  // Thread-safe.
  virtual absl::StatusOr<RealtimePartStatuses> GetPartStatuses(
      absl::Time deadline) = 0;

  // Starts a Session with the given `session_id` and `part_names`, and returns
  // an RtclSessionBridgeInterface object for that Session. The `context` is
  // used to tag the part status.
  //
  // RtclSessionBridgeInterface should behave in an RAII fashion, i.e. once it
  // goes out of scope, the Session ends.
  //
  // Thread-safe.
  virtual absl::StatusOr<std::unique_ptr<RtclSessionBridgeInterface>>
  StartSession(SessionId session_id,
               const absl::flat_hash_set<std::string>& part_names,
               const RealtimeLogContext& context) = 0;

  virtual void SetSpeedOverride(double new_speed_override) = 0;

  virtual double GetSpeedOverride() const = 0;

  // Returns the latest RealtimeSafetyStatus. Note that this means multiple
  // calls may return the same result (if the realtime thread has not published
  // updated status data in between).
  //
  // Thread-safe.
  virtual absl::StatusOr<RealtimeSafetyStatus> GetSafetyStatus(
      absl::Time deadline) const = 0;

  // Retrieves the latest part property values, including wall and control
  // timestamps.
  //
  // Thread-safe
  virtual absl::StatusOr<TimestampedPartProperties> GetPartProperties() = 0;

  // Applies the given part property values atomically.
  //
  // Thread-safe, but there is no guarantee that the realtime thread reads a
  // property value before another call overwrites it.
  // For example, consider the following calls:
  //
  // SetPartProperties({.properties={{"my_part", {"my_prop", 1}}}});
  // SetPartProperties({.properties={{"my_part", {"my_prop", 2}}}});
  //
  // The realtime thread may or may not tick between these calls. If it does
  // not, it will never see "my_prop" == 1.
  //
  // Returns NotFoundError if any of the part or property names in `properties`
  // are unknown.
  // Returns InvalidArgument if a property value has the wrong type (bool
  // instead of double, or vice versa).
  virtual absl::Status SetPartProperties(const PartPropertyMap& properties) = 0;

  // Returns the latest operational state of each part.
  //
  // Thread-safe.
  virtual absl::StatusOr<absl::flat_hash_map<std::string, OperationalState>>
  GetPartStates() = 0;

  virtual void SetLoggingMode(LoggingMode logging_mode) = 0;

  virtual LoggingMode GetLoggingMode() const = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_BRIDGE_INTERFACE_H_
