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

#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve_utils.h"

#include <algorithm>
#include <limits>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve.h"

namespace intrinsic {

absl::StatusOr<double> NormalizedSamplingStepForCartesianCurve(
    double sampling_step_translation, double sampling_step_rotation,
    const CartesianCurveLength& curve_length) {
  if (sampling_step_translation <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The sampling step for translation must be positive. Got ",
                     sampling_step_translation, "."));
  }
  if (sampling_step_rotation <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The sampling step for rotation must be positive. Got ",
                     sampling_step_rotation, "."));
  }
  if (curve_length.translation_m < 0.0 || curve_length.rotation_rad < 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Invalid curve length. Translational and rotational lengths must not "
        "be negative. Got ",
        curve_length.translation_m, " and ", curve_length.rotation_rad, "."));
  }
  if (intrinsic::AlmostEquals(curve_length.translation_m, 0.0) &&
      intrinsic::AlmostEquals(curve_length.rotation_rad, 0.0)) {
    return absl::InvalidArgumentError(
        "Invalid curve length. Translational and rotational lengths must not "
        "be both null.");
  }

  const double normalized_sampling_step_translation =
      (curve_length.translation_m > 0.0)
          ? sampling_step_translation / curve_length.translation_m
          : std::numeric_limits<double>::infinity();

  const double normalized_sampling_step_rotation =
      (curve_length.rotation_rad > 0.0)
          ? sampling_step_rotation / curve_length.rotation_rad
          : std::numeric_limits<double>::infinity();

  return std::min(normalized_sampling_step_translation,
                  normalized_sampling_step_rotation);
}

absl::StatusOr<CartesianCurveMinimalSamplingStepsAndControlFrequency>
ComputeCartesianCurveMinimalSamplingStepsAndControlFrequency(
    double user_specified_sampling_step_translation_m,
    double user_specified_sampling_step_rotation_rad,
    const CartesianCurveLength& curve_length) {
  if (user_specified_sampling_step_translation_m <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The user-specified sampling step for translation must be positive. "
        "Got ",
        user_specified_sampling_step_translation_m, " meters instead."));
  }
  if (user_specified_sampling_step_rotation_rad <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The user-specified sampling step for rotation must be positive. Got ",
        user_specified_sampling_step_rotation_rad, " radians instead."));
  }
  if (curve_length.translation_m < 0.0 || curve_length.rotation_rad < 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Invalid curve length. Translational and rotational lengths must "
        "either be zero or positive. Got ",
        curve_length.translation_m, " meters and ", curve_length.rotation_rad,
        " radians, respectively, instead."));
  }

  // Set sampling step based on actual or estimated curve length, but
  // guarantee a minimum number of samples. We cannot exclude the curve
  // length to be zero in either its translational or rotational part.
  constexpr int kMinNumberOfSamples = 10;
  const double translational_sampling_step_m =
      (curve_length.translation_m > 0.0)
          ? std::min(user_specified_sampling_step_translation_m,
                     curve_length.translation_m / (kMinNumberOfSamples - 1))
          : user_specified_sampling_step_translation_m;
  const double rotational_sampling_step_rad =
      (curve_length.rotation_rad > 0.0)
          ? std::min(user_specified_sampling_step_rotation_rad,
                     curve_length.rotation_rad / (kMinNumberOfSamples - 1))
          : user_specified_sampling_step_rotation_rad;

  // The control frequency for FinePathIk is set using a fixed ratio w.r.t.
  // path sampling distance that is known to work well. As path sampling
  // distance we take the one producing the most conservative frequency.
  const double path_ik_control_frequency_hz =
      0.2 /
      std::max(translational_sampling_step_m, rotational_sampling_step_rad);
  return CartesianCurveMinimalSamplingStepsAndControlFrequency{
      .sampling_step_translational_m = translational_sampling_step_m,
      .sampling_step_rotational_rad = rotational_sampling_step_rad,
      .path_ik_control_frequency_hz = path_ik_control_frequency_hz};
}

}  // namespace intrinsic
