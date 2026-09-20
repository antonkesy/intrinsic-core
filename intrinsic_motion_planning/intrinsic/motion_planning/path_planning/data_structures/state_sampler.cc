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

#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

absl::Status StateSampler::SetSamplingLimitsCommon(
    const JointLimitsXd& limits) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto check_result,
                                IsWithinLimits(limits, system_limits_));
  if (!check_result) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "New sampling limits are outside the system limits: "
           << absl::string_view(ToFixedString(check_result));
  }
  sampling_limits_ = limits;
  return absl::OkStatus();
}

}  // namespace intrinsic
