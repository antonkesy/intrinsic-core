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

#include "intrinsic/motion_planning/path_planning/parametric_curves/parametric_curve_utils.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/joint_parametric_curve.h"
#include "intrinsic/motion_planning/path_planning/planners/joint_sampling_utils.h"
#include "intrinsic/motion_planning/trajectory_planning/path_refinement/path_refinement_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

absl::StatusOr<std::optional<double>>
JointSpaceUniformArcLengthPreservingSamplingOptions::ImpliedSamplingDistanceRad(
    const double path_length_rad) const {
  std::optional<double> implied_sampling_distance_rad = sampling_distance_rad;
  if (max_num_samples.has_value()) {
    if (*max_num_samples < 2) {
      return absl::InvalidArgumentError(absl::StrCat(
          "The value of `max_num_samples` must be >= 2 (if specified). Got ",
          *max_num_samples, "."));
    }
    if (path_length_rad <= 0.0) {
      return absl::InvalidArgumentError(
          absl::StrCat("The path length must be strictly positive. Got ",
                       path_length_rad, "."));
    }
    const double implied_sampling_distance_rad_from_max_num_samples =
        path_length_rad / (*max_num_samples - 1);
    if (implied_sampling_distance_rad.has_value()) {
      implied_sampling_distance_rad =
          std::max(*implied_sampling_distance_rad,
                   implied_sampling_distance_rad_from_max_num_samples);
    } else {
      implied_sampling_distance_rad =
          implied_sampling_distance_rad_from_max_num_samples;
    }
  }
  return implied_sampling_distance_rad;
}

absl::StatusOr<double> NormalizedSamplingStepForJointCurve(
    const double sampling_step_rad, const double curve_length_rad) {
  if (sampling_step_rad <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The sampling step must be strictly positive. Got ",
                     sampling_step_rad, "."));
  }
  if (curve_length_rad <= 0.0 || std::isinf(curve_length_rad)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The curve length must be strictly positive and finite. Got ",
        curve_length_rad, "."));
  }

  return std::min(sampling_step_rad / curve_length_rad, 1.0);
}

UniformResamplingStepsForJointSpacePathFunction
CreateUniformResamplingStepsForJointSpacePathFunction(
    const JointSpaceUniformArcLengthPreservingSamplingOptions&
        joint_space_uniform_sampling_options) {
  return [&](absl::Span<const eigenmath::VectorNd> joint_space_path)
             -> absl::StatusOr<std::optional<std::vector<double>>> {
    INTR_ASSIGN_OR_RETURN(
        const double path_length,
        topp::ComputePathLength(absl::MakeConstSpan(joint_space_path)));
    INTR_ASSIGN_OR_RETURN(
        const std::optional<double> sampling_distance_rad,
        joint_space_uniform_sampling_options.ImpliedSamplingDistanceRad(
            path_length));
    if (sampling_distance_rad.has_value()) {
      return std::vector<double>{*sampling_distance_rad};
    }
    return std::nullopt;
  };
}

absl::StatusOr<std::vector<eigenmath::VectorNd>>
SampleJointCurveAndResampleForUniformJointSampling(
    const JointParametricCurve& joint_curve,
    double joint_curve_sampling_distance_rad,
    const UniformResamplingStepsForJointSpacePathFunction&
        resampling_steps_function,
    bool force_odd_number_of_samples) {
  if (resampling_steps_function == nullptr) {
    return absl::InvalidArgumentError(
        "The `resampling_steps_function` must be set.");
  }
  INTR_ASSIGN_OR_RETURN(
      const double normalized_sampling_step,
      NormalizedSamplingStepForJointCurve(joint_curve_sampling_distance_rad,
                                          joint_curve.GetCurveLength()));
  INTR_ASSIGN_OR_RETURN(
      const JointConfigurationsAtNormalizedCurveParameters
          joint_path_at_normalized_curve_parameters,
      joint_curve.SampleUniformly(normalized_sampling_step,
                                  force_odd_number_of_samples));
  std::vector<eigenmath::VectorNd> sampled_joint_path =
      std::move(joint_path_at_normalized_curve_parameters.joint_configurations);

  // If `joint_space_uniform_sampling_steps_rad` computed by the
  // `resampling_steps_function` is set, we resample the path in joint space
  // accordingly.
  INTR_ASSIGN_OR_RETURN(const std::optional<std::vector<double>>
                            joint_space_uniform_sampling_steps_rad,
                        resampling_steps_function(sampled_joint_path));
  if (joint_space_uniform_sampling_steps_rad.has_value()) {
    // The `ApproximateJointUniformSampling()` implementation ensures that the
    // returned `curve_parameters_for_uniform_joint_sampling`'s size is odd.
    INTR_ASSIGN_OR_RETURN(
        const std::vector<double> curve_parameters_for_uniform_joint_sampling,
        ApproximateJointUniformSampling(
            joint_path_at_normalized_curve_parameters
                .normalized_curve_parameters,
            sampled_joint_path, *joint_space_uniform_sampling_steps_rad));

    INTR_ASSIGN_OR_RETURN(
        sampled_joint_path,
        joint_curve.Sample(curve_parameters_for_uniform_joint_sampling));
  }

  if (force_odd_number_of_samples && sampled_joint_path.size() % 2 == 0) {
    return absl::InternalError(
        "The number of sampled joint configurations in the path is even. This "
        "should not happen.");
  }

  return sampled_joint_path;
}

}  // namespace intrinsic
