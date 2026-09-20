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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_PREFILTER_INTERFACE_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_PREFILTER_INTERFACE_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Abstract strategy interface defining generic N-DoF contracts for dynamic
// trajectory shaping and prefiltering operations within the hardware module.
class PrefilterInterface {
 public:
  virtual ~PrefilterInterface() = default;

  // Computes the single-cycle dynamic trajectory shaping filter response
  // for the target joint position setpoints.
  virtual RealtimeStatusOr<eigenmath::VectorNd> ComputeControl(
      const eigenmath::VectorNd& setpoints) = 0;

  // Resets internal filter state equations.
  virtual void Reset() = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_PREFILTER_INTERFACE_H_
