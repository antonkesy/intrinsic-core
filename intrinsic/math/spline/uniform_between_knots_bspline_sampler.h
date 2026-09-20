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

#ifndef INTRINSIC_MATH_SPLINE_UNIFORM_BETWEEN_KNOTS_BSPLINE_SAMPLER_H_
#define INTRINSIC_MATH_SPLINE_UNIFORM_BETWEEN_KNOTS_BSPLINE_SAMPLER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/spline/bspline_sampler.h"

namespace intrinsic {

// A B-spline sampler that generates parameter values uniformly within each knot
// span of the B-spline.
//
// This sampler divides the active domain of the spline into knot spans. For
// each knot span, it computes a number of uniform samples such that the step
// size is as close as possible to the `reference_sampling_step`. It guarantees
// that each knot span is sampled and that the knot boundaries themselves are
// included in the generated samples.
class UniformBetweenKnotsBSplineSampler : public BSplineSampler {
 public:
  UniformBetweenKnotsBSplineSampler() = default;
  ~UniformBetweenKnotsBSplineSampler() override = default;

  // Generates parameter values uniformly distributed within each individual
  // knot span, ensuring knot boundaries are preserved. The
  // `reference_sampling_step` is adjusted for each span to fit an integer
  // number of samples.
  absl::StatusOr<std::vector<double>> GenerateCurveParameters(
      const BSplineNd& spline, double reference_sampling_step) const override;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_UNIFORM_BETWEEN_KNOTS_BSPLINE_SAMPLER_H_
