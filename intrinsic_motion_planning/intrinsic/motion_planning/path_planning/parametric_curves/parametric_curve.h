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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_PARAMETRIC_CURVE_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_PARAMETRIC_CURVE_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

namespace intrinsic {

// A struct to define the parameters of a blending arc. The `start` and `end`
// members can provide more information than the `blending_corner` (e.g. T1 may
// contain information about the first- and second-order derivatives, while T2
// may only contain the positional information without derivatives).
template <typename T1, typename T2 = T1>
struct BlendingArcParameters {
  T1 start;
  T2 blending_corner;
  T1 end;
};

// An interface to define custom parametric curves as a function of a single
// curve parameter.
template <typename T>
class ParametricCurve {
 public:
  static absl::StatusOr<std::unique_ptr<ParametricCurve>> Create(
      const BlendingArcParameters<T>& blending_arc_parameters) {
    return absl::UnimplementedError(
        "ParametricCurve::Create() method is not implemented.");
  }

  virtual ~ParametricCurve() = default;

  virtual absl::string_view Name() const = 0;

  // Returns the curve sample at the given `normalized_curve_parameter` in the
  // interval [0,1].
  virtual absl::StatusOr<T> Sample(double normalized_curve_parameter) const = 0;

  // A batch version of the above function. Returns the curve samples for a
  // sequence of `normalized_curve_parameters` in the interval [0,1]. Some
  // implementation might have improved performance when sampling is batched.
  virtual absl::StatusOr<std::vector<T>> Sample(
      absl::Span<const double> normalized_curve_parameters) const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_PARAMETRIC_CURVE_H_
