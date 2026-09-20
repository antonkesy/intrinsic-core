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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_TYPES_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_TYPES_H_

#include <limits>

namespace intrinsic::icon {

// One-dimensional speed override factor (in short sof) state with its state
// `sof`, its first time derivative `dsof_dt`, and its second time derivative
// `d2sof_dt2`.
struct SpeedOverrideFactorStateWithSecondDerivative {
  double sof;
  double dsof_dt;
  double d2sof_dt2;
};

// One-dimensional speed override factor (in short sof) state with its state
// `sof` and its first time derivative `dsof_dt`.
struct SpeedOverrideFactorStateWithDerivative {
  double sof;
  double dsof_dt;
};

// One-dimensional speed override factor (in short sof) limits for the state
// `sof` (`min_sof` and `max_sof`), for the first time derivative `dsof_dt`
// (`min_dsof_dt` and `max_dsof_dt`), and for the second time derivative
// `d2sof_dt2` (`min_d2sof_dt2` and `max_d2sof_dt2`).
struct SpeedOverrideFactorLimits {
  // Lower and upper limits for time derivatives of the speed override factor.
  struct LimitPair {
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
  };

  LimitPair sof_limits;
  LimitPair dsof_dt_limits;
  LimitPair d2sof_dt2_limits;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_TYPES_H_
