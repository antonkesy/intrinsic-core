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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_PARAMETRIC_CURVE_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_PARAMETRIC_CURVE_UTILS_H_

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve.h"

namespace intrinsic {

// Computes a normalized sampling step for sampling a `CartesianParametricCurve`
// of given `curve_length`, based on the provided `sampling_step_translation_m`
// and `sampling_step_rotation_rad`. The result is the minimum normalized
// sampling step that guarantees that the translational and rotational parts of
// the curve are sampled with a step no wider than `sampling_step_translation_m`
// and `sampling_step_rotation_rad`, respectively.
// Returns an error status for non-positive `sampling_step_translation_m` or
// `sampling_step_rotation_rad`, and if the components of `curve_length` are
// negative or both null.
absl::StatusOr<double> NormalizedSamplingStepForCartesianCurve(
    double sampling_step_translation_m, double sampling_step_rotation_rad,
    const CartesianCurveLength& curve_length);

struct CartesianCurveMinimalSamplingStepsAndControlFrequency {
  double sampling_step_translational_m;
  double sampling_step_rotational_rad;

  // The control frequency for `PathIk`. Please refer to the documentation of
  // `PathIk` in
  // intrinsic/motion_planning/path_planning/path_ik/path_ik.h for
  // more details.
  double path_ik_control_frequency_hz;
};

// Computes the minimal translational, rotational sampling steps,
// and control frequency in a
// `CartesianCurveMinimalSamplingStepsAndControlFrequency` struct, given the
// user-specified translational sampling step (in meters), rotational sampling
// step (in radians), and the `CartesianCurveLength`.
absl::StatusOr<CartesianCurveMinimalSamplingStepsAndControlFrequency>
ComputeCartesianCurveMinimalSamplingStepsAndControlFrequency(
    double user_specified_sampling_step_translation_m,
    double user_specified_sampling_step_rotation_rad,
    const CartesianCurveLength& curve_length);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_PARAMETRIC_CURVE_UTILS_H_
