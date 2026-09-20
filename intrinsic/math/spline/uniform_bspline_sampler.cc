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

#include "intrinsic/math/spline/uniform_bspline_sampler.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/linspace.h"
#include "intrinsic/math/spline/bspline.h"

namespace intrinsic {

absl::StatusOr<std::vector<double>>
UniformBSplineSampler::GenerateCurveParameters(
    const BSplineNd& spline, const double reference_sampling_step) const {
  if (!(reference_sampling_step > 0.0)) {
    return absl::InvalidArgumentError(
        "The reference sampling step must be strictly positive.");
  }
  std::vector<double> unique_knots;
  if (!spline.GetValidUniqueKnots(unique_knots)) {
    return absl::InternalError("Couldn't get the spline unique knots.");
  }
  if (unique_knots.size() < 2) {
    return absl::InvalidArgumentError(
        "The spline knot vector must have at least two unique knots.");
  }

  const int num_samples =
      1 + static_cast<int>(
              std::max(std::round((unique_knots.back() - unique_knots.front()) /
                                  reference_sampling_step),
                       1.0));

  return Linspace(unique_knots.front(), unique_knots.back(), num_samples);
}

}  // namespace intrinsic
