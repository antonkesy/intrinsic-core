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

#ifndef INTRINSIC_MATH_SPLINE_UNIFORM_BSPLINE_SAMPLER_H_
#define INTRINSIC_MATH_SPLINE_UNIFORM_BSPLINE_SAMPLER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/spline/bspline_sampler.h"

namespace intrinsic {

// A B-spline sampler that generates parameter values uniformly spaced along the
// entire parameter domain of the spline.
//
// This sampler divides the parameter range (from the first knot to the last
// knot) into equal-sized steps. The number of steps is determined by dividing
// the parameter range by the `reference_sampling_step`, rounded to the nearest
// integer, with a minimum of 2 samples (1 step).
class UniformBSplineSampler : public BSplineSampler {
 public:
  UniformBSplineSampler() = default;
  ~UniformBSplineSampler() override = default;

  // Generates parameter values uniformly distributed between the first and last
  // knots of the spline, adjusting the `reference_sampling_step` to fit an
  // integer number of samples.
  absl::StatusOr<std::vector<double>> GenerateCurveParameters(
      const BSplineNd& spline, double reference_sampling_step) const override;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_UNIFORM_BSPLINE_SAMPLER_H_
