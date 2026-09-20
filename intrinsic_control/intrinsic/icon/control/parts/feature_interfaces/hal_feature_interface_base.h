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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HAL_FEATURE_INTERFACE_BASE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HAL_FEATURE_INTERFACE_BASE_H_

#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// This class provides hooks that HAL parts can call from their ReadStatus() and
// ApplyCommand() methods, and one to reset a HalFeatureInterface.
//
// This allows HalFeatureInterface implementations to execute "prepare" or
// "finalize" steps in each cycle, and to directly modify Part properties.
//
// This class has no-op implementations for all three hooks, so that
// HalFeatureInterfaces need only implement them if they actually have something
// to do.
//
// Most HalFeatureInterfaces will not need to implement any of these methods!
class HalFeatureInterfaceBase {
 public:
  virtual ~HalFeatureInterfaceBase() = default;

  // Parts call this in their ReadStatus() method every cycle. Classes that
  // inherit from HalFeatureInterfaceBase can, but do not have to, override
  // this. Example use cases:
  //
  // * A HalFeatureInterface wants to offer read access to values from the
  //   hardware module. The HalFeatureInterface can copy/convert that data once
  //   per cycle in its override of ReadStatus(), rather than doing it on demand
  //   whenever an Action calls the corresponding ICON FeatureInterface method.
  // * A HalFeatureInterface wants to influence Part Property values (this
  //   should be rare). It can do so by using `params`.
  // * A HalFeatureInterface wants to report critical errors. Returning non-OK
  //   from ReadStatus() will safely bring down ICON, regardless of whether any
  //   Actions call methods on the HalFeatureInterface.
  virtual RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) {
    return OkStatus();
  }

  // Parts call this in their ApplyCommand() method every cycle. Classes that
  // inherit from HalFeatureInterfaceBase can, but do not have to, override
  // this. Example use cases:
  //
  // * A HalFeatureInterface offers write access to a hardware module. The
  //   HalFeatureInterface class can delay converting/copying the data to the
  //   hardware module's flatbuffer until it receives an ApplyCommand() call,
  //   thus saving unnecessary copies.
  // * A HalFeatureInterface class can also do additional housekeeping /
  //   validation on its inputs in ApplyCommand(). This is different from doing
  //   said validation in the FeatureInterface methods, because the class can be
  //   certain that no further calls wil be made in the same cycle after
  //   ApplyCommand.
  // * A HalFeatureInterface wants to read Part Property values to forward
  //   commands to a hardware module (this should be rare). It can do so by
  //   using `params`.
  // * A HalFeatureInterface wants to report critical errors. Returning non-OK
  //   from ApplyCommand() will safely bring down ICON, regardless of whether
  //   any Actions call methods on the HalFeatureInterface.
  virtual RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) {
    return OkStatus();
  }

  // Resets the internal state of a feature interface. Parts that use
  // HalFeatureInterfaceBase classes should call this when they transition from
  // disabled to enabled.
  virtual RealtimeStatus Reset() { return OkStatus(); }
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HAL_FEATURE_INTERFACE_BASE_H_
