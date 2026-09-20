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

#ifndef INTRINSIC_MATH_SPLINE_BSPLINE_SAMPLER_H_
#define INTRINSIC_MATH_SPLINE_BSPLINE_SAMPLER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/spline/bspline.h"

namespace intrinsic {

// Interface for generating parameter values for sampling a B-spline.
//
// Subclasses implement different strategies (e.g., uniform, between knots) for
// determining the parameter values (on the spline's knot domain) at which
// the spline should be discretized during path refinement.
// TODO: b/527015156 - Add unit tests for BSplineSampler implementations.
class BSplineSampler {
 public:
  virtual ~BSplineSampler() = default;

  // Generates the raw curve parameter values (in the spline's knot domain)
  // at which the spline should be sampled.
  //
  // @param spline The B-spline curve to sample.
  // @param reference_sampling_step The reference spacing between samples in the
  //   knot domain. The concrete sampler implementation may adjust this step
  //   (e.g., to fit an integer number of samples per knot span, or based on
  //   local curvature).
  // @return A sorted vector of parameter values, or an error status.
  virtual absl::StatusOr<std::vector<double>> GenerateCurveParameters(
      const BSplineNd& spline, double reference_sampling_step) const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_BSPLINE_SAMPLER_H_
