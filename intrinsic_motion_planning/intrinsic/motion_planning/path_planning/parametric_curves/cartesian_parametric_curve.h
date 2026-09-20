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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_PARAMETRIC_CURVE_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_PARAMETRIC_CURVE_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/parametric_curve.h"

namespace intrinsic {

// Holds the length of a Cartesian curve as two separate values, representing
// the total translation and rotation on the curve, respectively.
struct CartesianCurveLength {
  double translation_m;
  double rotation_rad;
};

// Holds the sampled Cartesian `poses` at the `normalized_curve_parameters`.
struct PosesAtNormalizedCurveParameters {
  std::vector<Pose3d> poses;
  std::vector<double> normalized_curve_parameters;
};

// An interface to define custom Cartesian curves as a function of a single
// curve parameter.
class CartesianParametricCurve : virtual public ParametricCurve<Pose3d> {
 public:
  ~CartesianParametricCurve() override = default;

  // Returns the length of the Cartesian curve (separately for translation and
  // rotation).
  virtual CartesianCurveLength GetCurveLength() const = 0;

  // Samples the curve uniformly. The sampling step is normalized, i.e. the
  // sampling step is scaled by the curve length. The number of samples is
  // determined by the `normalized_sampling_step`, so that it produces an
  // integer number of samples. It returns the sampled poses and the normalized
  // curve parameters. If `force_odd_number_of_samples` is true, the total
  // number of samples is forced to be odd.
  virtual absl::StatusOr<PosesAtNormalizedCurveParameters> SampleUniformly(
      double normalized_sampling_step,
      bool force_odd_number_of_samples) const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_PARAMETRIC_CURVE_H_
