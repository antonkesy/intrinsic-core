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

#include "intrinsic/math/spline/uniform_between_knots_bspline_sampler.h"

#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/spline/bspline_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

absl::StatusOr<std::vector<double>>
UniformBetweenKnotsBSplineSampler::GenerateCurveParameters(
    const BSplineNd& spline, const double reference_sampling_step) const {
  if (!(reference_sampling_step > 0.0)) {
    return absl::InvalidArgumentError(
        "The reference sampling step must be strictly positive.");
  }

  std::vector<double> unique_knots;
  if (!spline.GetValidUniqueKnots(unique_knots)) {
    return absl::InternalError("Couldn't get the spline unique knots.");
  }
  INTR_ASSIGN_OR_RETURN(
      const std::vector<int> num_uniform_samples_per_knot_span,
      NumSamplesPerValidKnotSpan(spline, reference_sampling_step));
  if (unique_knots.size() != num_uniform_samples_per_knot_span.size() + 1) {
    // This should never be the case.
    return absl::InternalError(
        "Inconsistent number of unique knots and knot spans.");
  }

  const int num_samples =
      absl::c_accumulate(num_uniform_samples_per_knot_span, 0) -
      num_uniform_samples_per_knot_span.size() + 1;
  std::vector<double> knot_path_vars;
  knot_path_vars.reserve(num_samples);
  knot_path_vars.push_back(unique_knots.front());

  for (int i = 0; i < num_uniform_samples_per_knot_span.size(); ++i) {
    const double start_knot = unique_knots[i];
    const double end_knot = unique_knots[i + 1];
    const double knot_step_in_knot_span =
        (end_knot - start_knot) / (num_uniform_samples_per_knot_span[i] - 1);

    for (int j = 1; j < num_uniform_samples_per_knot_span[i]; ++j) {
      knot_path_vars.push_back(start_knot +
                               static_cast<double>(j) * knot_step_in_knot_span);
    }
  }

  return knot_path_vars;
}

}  // namespace intrinsic
