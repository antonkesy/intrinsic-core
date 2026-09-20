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

#ifndef INTRINSIC_ICON_SERVER_ROBOT_CONNECTION_INTERFACE_H_
#define INTRINSIC_ICON_SERVER_ROBOT_CONNECTION_INTERFACE_H_

#include <memory>
#include <string>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/slot_part_map.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/server/operational_state_interface.h"
#include "intrinsic/icon/server/part_collection.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/logging/proto/context.pb.h"

namespace intrinsic::icon {

// Common interface to the different classes that need a connection to the robot
// to work, including Session creation. They are used by the ICON Application
// Layer service. Note that implementations may or may not actually own the
// handle(s) to the underlying medium.
//
// Classes implementing this interface must be thread safe!
class RobotConnectionInterface {
 public:
  virtual ~RobotConnectionInterface() = default;

  // The Sessions returned from this must not outlive the
  // RobotConnectionInterface.
  virtual absl::StatusOr<std::unique_ptr<SessionInterface>> CreateSession(
      const absl::flat_hash_set<std::string>& part_names, SessionId session_id,
      const RealtimeLogContext& log_context) ABSL_ATTRIBUTE_LIFETIME_BOUND = 0;

  // Returns a reference to an OperationalStateInterface. Use this to manage and
  // inspect the robot's operational state (Enabled/Disabled/Faulted). Note that
  // OperationalStateInterface itself is guaranteed to be thread safe.
  virtual OperationalStateInterface& MutableOperationalStateInterface()
      ABSL_ATTRIBUTE_LIFETIME_BOUND = 0;

  // Returns a map from Action type name to the corresponding ActionSignature
  // proto.
  virtual absl::flat_hash_map<std::string,
                              intrinsic_proto::icon::v1::ActionSignature>
  ActionTypeToSignature() const = 0;

  // The following two functions return a (mutable) PartCollection. Use the
  // PartCollection to
  // * Get Part status data
  // * Get Part configuration data
  // * Access a PartsManager (intrinsic/icon/server/parts_manager.h)
  //   to track availability of Parts
  virtual PartCollection& MutablePartCollection() = 0;
  virtual const PartCollection& GetPartCollection() const
      ABSL_ATTRIBUTE_LIFETIME_BOUND = 0;

  virtual const intrinsic_proto::icon::v1::ServerConfig& config() const
      ABSL_ATTRIBUTE_LIFETIME_BOUND = 0;

  // Returns OkStatus if `action_type_name` can be instantiated with
  // `slot_part_map`.
  // That is, all Slot names in `slot_part_map` must match those in
  // `action_type_name`'s ActionSignature, and the Part assigned to each Slot
  // must be compatible with that Slot.
  // At a minimum, being compatible with a Slot requires the Part to have all
  // Feature Interfaces that the Slot requires, but implementations of
  // RobotConnectionInterface can enforce additional conditions.
  virtual absl::Status ActionCompatibleWithSlotPartMap(
      absl::string_view action_type_name,
      const SlotPartMap& slot_part_map) const = 0;

  // Returns OkStatus if the Part `part_name` is compatible with *any* of the
  // Slots in `action_type_name`'s ActionSignature.
  // Compatibility is defined as for ActionCompatibleWithSlotPartMap above.
  virtual absl::Status ActionCompatibleWithPart(
      absl::string_view action_type_name,
      absl::string_view part_name) const = 0;

  // Takes `new_speed_override` (a number between 0 and 1) and hands it to
  // the robot, where the actions will be adjust their actual speed as
  // quickly as is possible without violating limits.
  virtual void SetSpeedOverride(double new_speed_override) = 0;

  // Returns the current speed override value (a number between 0 and 1).
  virtual double GetSpeedOverride() const = 0;

  virtual void SetLoggingMode(LoggingMode logging_mode) = 0;
  virtual LoggingMode GetLoggingMode() const = 0;

  virtual absl::StatusOr<absl::flat_hash_map<std::string, OperationalState>>
  GetPartStates() = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_ROBOT_CONNECTION_INTERFACE_H_
