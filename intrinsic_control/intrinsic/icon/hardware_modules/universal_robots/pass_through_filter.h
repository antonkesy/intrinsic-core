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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_PASS_THROUGH_FILTER_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_PASS_THROUGH_FILTER_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hardware_modules/universal_robots/prefilter_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Header-only filter implementation providing exact inline pass-through
// (identity) behavior for generic N-DoF trajectory setpoints.
class PassThroughFilter : public PrefilterInterface {
 public:
  PassThroughFilter() = default;
  ~PassThroughFilter() override = default;

  // Pass-through (identity) implementation returning the input setpoints
  // unmodified.
  RealtimeStatusOr<eigenmath::VectorNd> ComputeControl(
      const eigenmath::VectorNd& setpoints) override {
    return eigenmath::VectorNd(setpoints);
  }

  void Reset() override {}
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_PASS_THROUGH_FILTER_H_
