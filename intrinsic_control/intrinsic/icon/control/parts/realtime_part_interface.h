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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_INTERFACE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_INTERFACE_H_

#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/realtime_part_property_access.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Defines the (abstract) interface for all Parts in RTCL.
//
//
// This interface is made accessible only to the RTCL runtime, not to RTCL
// Actions.
// The only way for a Part to offer interfaces to RTCL Actions is to register
// PartInterfaces with the registry that it returns from GetFeatureInterfaces().
class RealtimePartInterface {
 public:
  virtual ~RealtimePartInterface() = default;

  struct ReadStatusParameters {
    RealtimePartPropertyAccess& part_properties;
    SafetyStatus safety_status;
  };

  // Returns the operational state of the part, potentially a combined state of
  // all sub-components, e.g. hardware modules.
  // If the hardware modules have different states, the most critical state
  // (kEnabled < kDisabled < kFaultedConnected < kFatallyFaulted) and fault
  // reason is returned.
  virtual RealtimeStatusOr<RealtimeOperationalStatus> GetOperationalStatus()
      const = 0;

  // Returns the group of hardware modules that this part depends on, e.g.
  // the group of operational or cell control hardware modules.
  virtual HardwareGroupSet GetHardwareDependencies() const = 0;

  // Reads the hardware status – the values returned by any interfaces in the
  // FeatureInterfaceRegistry must remain the same in between calls to this.
  //
  // The RTCL runtime calls this at the beginning of each cycle, i.e. in
  // predictable intervals.
  //
  // Returns an error if there was a problem reading the hardware status.
  virtual RealtimeStatus ReadStatus(ReadStatusParameters params) = 0;

  struct ApplyCommandParameters {
    const RealtimePartPropertyAccess& part_properties;
    SafetyStatus safety_status;
  };

  // Forwards commands from RTCL to the hardware. This means that any interfaces
  // in the FeatureInterfaceRegistry are expected to "latch" their values until
  // this is called.
  // The RTCL runtime calls this at the end of each cycle.
  // Returns an error if there is a problem applying the commands.
  virtual RealtimeStatus ApplyCommand(ApplyCommandParameters params) = 0;

  // Returns a registry of part interfaces. The RTCL runtime will hand that
  // registry to any Actions that use this part.
  // Note that the decision whether a RealtimePart directly inherits from
  // the interfaces in
  // intrinsic/icon/control/parts/feature_interfaces.h (and populates
  // the registry with multiple references to its `this` pointer) or implements
  // interfaces by composition is left up to the RealtimePart implementer.
  virtual FeatureInterfaceRegistry& GetFeatureInterfaces() = 0;
  virtual const FeatureInterfaceRegistry& GetFeatureInterfaces() const = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_INTERFACE_H_
