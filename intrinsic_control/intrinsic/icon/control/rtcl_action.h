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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_ACTION_H_
#define INTRINSIC_ICON_CONTROL_RTCL_ACTION_H_

#include <cstddef>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/realtime_signal_access.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/streaming_io_realtime.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// RtclActionInterface implementations can run in the realtime control
// layer and control motion and devices.
// This interface is kept generic so users can link in custom realtime actions
// with new parameter types.
//
// In contrast to RealtimeActionInterface, RtclActionInterface more strictly
// divides non-realtime and realtime data. For example, RealtimeActionInterface
// factories have direct access to Part instances, while RtclActionInterface
// factories do not.
//
// The cyclic interface of RtclActionInterface consists of the Sense() and
// Control() functions.
//
// The order of function calls during each cycle is as follows:
// * RealtimePart::ReadStatus() for all Parts in the system
// * Sense() for all Actions
// * Control() for all Actions
// * RealtimePart::ApplyCommand() for all Parts
class RtclActionInterface {
 public:
  // Parameters for the action's OnEnter() function. Since this only holds
  // references and small scalars, we can pass it to functions by value
  // efficiently.
  struct OnEnterParameters {
    // Use this to read the state of any Slots the Action has access to. Is
    // *guaranteed* to have the slots that are registered in the Action's
    // signature, with the same mapping between Slot names and RealtimeSlotIds
    // that was handed to the Action's Factory.
    const RealtimeSlotMap& slot_map;
    // Current speed override. This is a value between 0 and 1 that indicates to
    // an action whether (and how much) it should slow down its generated
    // motion.
    double speed_override = 1.0;
    // Currently requested behavior override.
    // The action is expected to resume normal operation once the override is
    // `BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN`.
    intrinsic_proto::icon::v1::BehaviorOverrideRequest
        requested_behavior_override = intrinsic_proto::icon::v1::
            BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN;
  };

  // Parameters for the action's Sense() function. Since this only holds
  // references and small scalars, we can pass it to functions by value
  // efficiently.
  struct SenseParameters {
    // Use this to read the state of any Slots the Action has access to. Is
    // *guaranteed* to have the slots that are registered in the Action's
    // signature, with the same mapping between Slot names and RealtimeSlotIds
    // that was handed to the Action's Factory.
    const RealtimeSlotMap& slot_map;
    // Use this to read streaming input values and write streaming output
    // values. Is guaranteed to provide access to all
    // streaming I/Os that were registered in the factory, if any.
    StreamingIoRealtimeAccess& streaming_io_access;
    // Use this to read real time signal values. It is guaranteed to provide
    // access to all the real time signals that are declared in the signature.
    RealtimeSignalAccess& signal_access;
    // Current speed override. This is a value between 0 and 1 that indicates to
    // an action whether (and how much) it should slow down its generated
    // motion.
    double speed_override = 1.0;
    // Currently requested behavior override.
    // The action is expected to resume normal operation once the override is
    // `BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN`.
    intrinsic_proto::icon::v1::BehaviorOverrideRequest
        requested_behavior_override = intrinsic_proto::icon::v1::
            BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN;
  };

  // Parameters for the action's Control() function. Since this only holds
  // references and small scalars, we can pass it to functions by value
  // efficiently.
  struct ControlParameters {
    // Use this to send commands to any Slots the Action has access to. Is
    // *guaranteed* to have the slots that are registered in the Action's
    // signature, with the same mapping between Slot names and RealtimeSlotIds
    // that was handed to the Action's Factory.
    RealtimeSlotMap& slot_map;
    // Current speed override. This is a value between 0 and 1 that indicates to
    // an action whether (and how much) it should slow down its generated
    // motion.
    double speed_override = 1.0;
    // Currently requested behavior override.
    // The action is expected to resume normal operation once the override is
    // `BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN`.
    intrinsic_proto::icon::v1::BehaviorOverrideRequest
        requested_behavior_override = intrinsic_proto::icon::v1::
            BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN;
  };

  virtual ~RtclActionInterface() = default;

  // (Re-)Sets any real-time state of the Action and prepares for cyclic
  // execution.
  //
  // Timeslicer calls this once each time the Action becomes active, before the
  // corresponding cycle's Sense().
  virtual RealtimeStatus OnEnter(OnEnterParameters parameters) = 0;

  // Updates the state of the Action at the beginning of a cycle, including:
  // * Reading information (via `slot_map`) from Slots it controls
  // * State Variables (which are used to evaluate conditions for Reactions)
  // * Action-specific data (for example, sampling a trajectory based on the
  //   number of ticks the Action has been active for)
  // * Reading from and writing to streaming I/O values via
  //   `streaming_io_access`
  //
  // (Not every Action needs to do all of the above)
  //
  // Timeslicer calls this at the beginning of each cycle, just after the Parts
  // read their current status from the HAL.
  virtual RealtimeStatus Sense(SenseParameters sense_parameters) = 0;

  // Sends commands to the Slots the Action controls, via `slot_map`. Should not
  // modify the externally visible state of the Action (i.e. State Variables).
  //
  // This is called at the end of each cycle, just before Parts apply their
  // commands.
  virtual RealtimeStatus Control(ControlParameters parameters) = 0;

  // Gets the current value of a state variable with 'name'.
  // Returns kNotFound if the state variable is not known for this action.
  virtual RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_ACTION_H_
