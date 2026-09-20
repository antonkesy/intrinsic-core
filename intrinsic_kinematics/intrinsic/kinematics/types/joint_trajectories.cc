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

#include "intrinsic/kinematics/types/joint_trajectories.h"

#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/time_series_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace joint_trajectories_internal {

absl::Status CheckCartesianArcLength(
    const std::vector<absl::Duration>& time_stamps,
    const std::optional<std::vector<double>>& cartesian_arc_lengths) {
  if (cartesian_arc_lengths.has_value()) {
    if (cartesian_arc_lengths->size() != time_stamps.size()) {
      return absl::FailedPreconditionError(
          absl::StrCat("Size of cartesian_arc_lengths, which is ",
                       cartesian_arc_lengths->size(),
                       " does not match size of time_stamps, which is ",
                       time_stamps.size()));
    }
    if (!AlmostEquals(cartesian_arc_lengths->front(), 0.0)) {
      return absl::FailedPreconditionError(
          absl::StrCat("First Cartesian arc length must be zero, but is ",
                       cartesian_arc_lengths->front(), "."));
    }
    INTR_RETURN_IF_ERROR(ElementsIncreaseMonotonically(
        absl::MakeConstSpan(cartesian_arc_lengths.value())))
        << "Trajectory Cartesian arc lengths must be monotonically "
           "increasing.";
  }

  return absl::OkStatus();
};

}  // namespace joint_trajectories_internal

}  // namespace intrinsic
