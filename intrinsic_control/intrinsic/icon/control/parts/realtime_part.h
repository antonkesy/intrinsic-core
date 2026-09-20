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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_H_

#include <memory>
#include <string>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// RealtimePart defines the (abstract) interface for all Parts in RTCL.
//
// This interface is made accessible only to the RTCL runtime, not to RTCL
// Actions.
// The only way for a Part to offer interfaces to RTCL Actions is to register
// PartInterfaces with the registry that it returns from GetFeatureInterfaces.
class RealtimePart {
 public:
  // Creates a RealtimePart with the given `name` for `part`.
  RealtimePart(absl::string_view name,
               std::unique_ptr<RealtimePartInterface> part);

  // Returns the current state of the Part. RealtimeControlManager updates this
  // at the beginning of each cycle. It stays constant for the rest of the
  // cycle.
  RealtimeStatusOr<RealtimeOperationalStatus> GetOperationalStatus() const;

  absl::string_view GetName() const;

  // First reads the hardware status using the RealtimePartInterface's
  // ReadStatus method, then updates state using its GetNextState method.
  //
  // The RTCL runtime calls this at the beginning of each cycle, i.e. in
  // predictable intervals.
  //
  // Forwards any errors from the RealtimePartInterface.
  RealtimeStatus ReadStatus(RealtimePartInterface::ReadStatusParameters params);

  // Forwards commands from RTCL to the hardware. This means that any interfaces
  // in the FeatureInterfaceRegistry are expected to "latch" their values until
  // this is called.
  // The RTCL runtime calls this at the end of each cycle.
  // Forwards any errors from the RealtimePartInterface.
  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params);

  // Returns a registry of part interfaces. The RTCL runtime will hand that
  // registry to any Actions that use this part.
  // Note that the decision whether a RealtimePart directly inherits from
  // the interfaces in
  // intrinsic/icon/control/parts/feature_interfaces.h (and populates
  // the registry with multiple references to its `this` pointer) or implements
  // interfaces by composition is left up to the RealtimePart implementer.
  FeatureInterfaceRegistry& GetFeatureInterfaces();
  const FeatureInterfaceRegistry& GetFeatureInterfaces() const;

  // Returns the group of hardware modules that this part depends on, e.g.
  // the group of operational or cell control hardware modules.
  HardwareGroupSet GetHardwareDependencies() const;

 private:
  std::string name_;
  std::unique_ptr<RealtimePartInterface> part_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_H_
