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

#include "intrinsic/motion_planning/path_planning/planners/joint_sampling_utils.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

absl::Status ValidateSamplingStepIsPositive(const double sampling_step) {
  if (sampling_step <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The `joint_sampling_step_rad` must be positive. Got: ", sampling_step,
        "."));
  }
  return absl::OkStatus();
}

// Validates that the `normalized_curve_parameters` match in size with the
// `joint_configurations`, that the curve parameters are at least 2, that they
// are in the valid range [0.0, 1.0], and that they are sorted. It also checks
// that all joint configurations have the same size. Returns an error otherwise.
absl::Status ValidateNormalizedCurveParametersAreValid(
    absl::Span<const double> normalized_curve_parameters,
    absl::Span<const eigenmath::VectorNd> joint_configurations) {
  if (normalized_curve_parameters.size() != joint_configurations.size()) {
    return absl::InvalidArgumentError(
        "The `normalized_curve_parameters` and `joint_configurations` must "
        "have the same size.");
  }
  if (normalized_curve_parameters.size() < 2) {
    return absl::InvalidArgumentError(
        "The  `normalized_curve_parameters` must have at least two elements.");
  }
  if (!absl::c_is_sorted(normalized_curve_parameters)) {
    return absl::InvalidArgumentError(
        "The  `normalized_curve_parameters` must be sorted.");
  }
  const double kEpsilon = 1.0e-8;
  if (normalized_curve_parameters.front() < -kEpsilon) {
    return absl::InvalidArgumentError(
        "The first element of `normalized_curve_parameters` is smaller than "
        "0.0.");
  }
  if (normalized_curve_parameters.back() > 1.0 + kEpsilon) {
    return absl::InvalidArgumentError(
        "The last element of  `normalized_curve_parameters` is greater than "
        "1.0.");
  }
  for (int i = 1; i < joint_configurations.size(); ++i) {
    if (joint_configurations[i].size() != joint_configurations[0].size()) {
      return absl::InvalidArgumentError(
          "The `joint_configurations` must have the same size.");
    }
  }
  return absl::OkStatus();
}

// Computes the accumulated joint distances for the given vector of
// `joint_configurations`. It accumulates the L2 norm of the differences
// between consecutive `joint_configurations`. Returns an error if the
// `joint_configurations` has less than two elements or if the vectors in the
// `joint_configurations` have different sizes.
absl::StatusOr<std::vector<double>> ComputeAccumulatedJointDistances(
    absl::Span<const eigenmath::VectorNd> joint_configurations) {
  std::vector<double> indices(joint_configurations.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::vector<double> accumulated_joint_dist(joint_configurations.size());
  absl::c_partial_sum(indices, accumulated_joint_dist.begin(),
                      [&](double current_sum, int index) {
                        if (index == 0) return 0.0;
                        return current_sum + (joint_configurations[index] -
                                              joint_configurations[index - 1])
                                                 .norm();
                      });
  return accumulated_joint_dist;
}

// Computes the interpolated normalized curve parameters that lead to an
// approximate uniform sampling of the arc-length of the path. The
// `accumulated_joint_dist` is the accumulated L2 norm of the differences
// between consecutive joint configurations sampled along the given
// `normalized_curve_parameters`. The `target_joint_lengths` represent the
// target arc-length to be interpolated. It appends the first and last elements
// from `normalized_curve_parameters` to the resulting vector.
absl::StatusOr<std::vector<double>>
ComputeNormalizedCurveParametersAtTargetJointLengths(
    absl::Span<const double> normalized_curve_parameters,
    absl::Span<const double> accumulated_joint_dist,
    absl::Span<const double> target_joint_lengths) {
  const int num_joint_samples = target_joint_lengths.size() + 2;
  std::vector<double> interpolated_normalized_curve_parameters;
  interpolated_normalized_curve_parameters.reserve(num_joint_samples);
  interpolated_normalized_curve_parameters.push_back(
      normalized_curve_parameters.front());

  int q_index = 0;
  for (int i = 0; i < target_joint_lengths.size(); ++i) {
    const double target_joint_length = target_joint_lengths[i];
    while (q_index + 1 < accumulated_joint_dist.size()) {
      if ((accumulated_joint_dist[q_index] <= target_joint_length) &&
          (target_joint_length < accumulated_joint_dist[q_index + 1]))
        break;
      q_index += 1;
    }
    // Make sure that we do not go beyond the last knot.
    q_index = std::clamp(
        q_index, 0, static_cast<int>(normalized_curve_parameters.size()) - 2);

    const double knot_curr = normalized_curve_parameters[q_index];
    const double knot_next = normalized_curve_parameters[q_index + 1];
    const double dist_curr = accumulated_joint_dist[q_index];
    const double dist_next = accumulated_joint_dist[q_index + 1];
    const double excess_joint_length = target_joint_length - dist_curr;
    const double ratio = excess_joint_length / (dist_next - dist_curr);
    const double normed_knot = knot_curr + ratio * (knot_next - knot_curr);
    interpolated_normalized_curve_parameters.push_back(normed_knot);
  }
  interpolated_normalized_curve_parameters.push_back(
      normalized_curve_parameters.back());

  return interpolated_normalized_curve_parameters;
}

absl::StatusOr<std::vector<double>> ApproximateJointUniformSampling(
    absl::Span<const double> normalized_curve_parameters,
    absl::Span<const eigenmath::VectorNd> joint_configurations,
    const double joint_sampling_step_rad,
    const bool force_odd_number_of_samples) {
  INTR_RETURN_IF_ERROR(ValidateSamplingStepIsPositive(joint_sampling_step_rad));
  INTR_RETURN_IF_ERROR(ValidateNormalizedCurveParametersAreValid(
      normalized_curve_parameters, joint_configurations));

  // Compute accumulated joint distances for samples.
  INTR_ASSIGN_OR_RETURN(const std::vector<double> accumulated_joint_dist,
                        ComputeAccumulatedJointDistances(joint_configurations));

  // Compute the number of samples that would lead to an ideal uniform sampling
  // step considering that the ideal number of samples can be forced to be odd
  // or can be free and that the actual sampling step must be as close as
  // possible to the desired one.
  const double ideal_num_joint_samples =
      (accumulated_joint_dist.back() / joint_sampling_step_rad) + 1.0;
  const int ideal_odd_num_joint_samples =
      2 * std::round((ideal_num_joint_samples - 1) / 2) + 1;
  const int ideal_possibly_non_odd_num_joint_samples =
      std::round(ideal_num_joint_samples);

  constexpr int kMinimumOddNumJointSamples = 3;
  constexpr int kMinimumEvenNumJointSamples = 2;
  const int num_joint_samples =
      force_odd_number_of_samples
          ? std::max(kMinimumOddNumJointSamples, ideal_odd_num_joint_samples)
          : std::max(kMinimumEvenNumJointSamples,
                     ideal_possibly_non_odd_num_joint_samples);

  // Compute the equivalent joint sampling step.
  const double actual_uniform_joint_sampling_step_rad =
      accumulated_joint_dist.back() / (num_joint_samples - 1);

  // Compute the target joint lengths to be sampled.
  std::vector<double> target_joint_lengths;
  target_joint_lengths.reserve(num_joint_samples);
  for (int i = 1; i < num_joint_samples - 1; ++i) {
    target_joint_lengths.push_back(static_cast<double>(i) *
                                   actual_uniform_joint_sampling_step_rad);
  }

  // Run resampling step to get the interpolated normalized curve parameters
  // that lead to an approximate uniform sampling of the joint samples.
  return ComputeNormalizedCurveParametersAtTargetJointLengths(
      normalized_curve_parameters, accumulated_joint_dist,
      target_joint_lengths);
}

absl::StatusOr<std::vector<double>> ApproximateJointUniformSampling(
    absl::Span<const double> normalized_curve_parameters,
    absl::Span<const eigenmath::VectorNd> joint_configurations,
    double joint_sampling_step_rad_1, double joint_sampling_step_rad_2,
    bool force_odd_number_of_samples) {
  INTR_RETURN_IF_ERROR(
      ValidateSamplingStepIsPositive(joint_sampling_step_rad_1));
  INTR_RETURN_IF_ERROR(
      ValidateSamplingStepIsPositive(joint_sampling_step_rad_2));
  INTR_RETURN_IF_ERROR(ValidateNormalizedCurveParametersAreValid(
      normalized_curve_parameters, joint_configurations));

  // Compute accumulated joint distances for samples.
  INTR_ASSIGN_OR_RETURN(const std::vector<double> accumulated_joint_dist,
                        ComputeAccumulatedJointDistances(joint_configurations));

  // Compute the number of samples with each sampling step that would lead to an
  // ideal uniform sampling step considering that the ideal number of total
  // samples can be forced to be odd or can be free and that the actual sampling
  // steps must be as close as possible to the desired ones.
  const double path_length = accumulated_joint_dist.back();
  const double half_path_length = path_length / 2.0;

  const double ideal_num_joint_samples1 =
      half_path_length / joint_sampling_step_rad_1;
  const double ideal_num_joint_samples2 =
      half_path_length / joint_sampling_step_rad_2;
  constexpr int kMinimumNumSamplesPerHalfPath = 1;
  int rounded_num_joint_samples_1 =
      std::max(kMinimumNumSamplesPerHalfPath,
               static_cast<int>(std::round(ideal_num_joint_samples1)));
  int rounded_num_joint_samples_2 =
      std::max(kMinimumNumSamplesPerHalfPath,
               static_cast<int>(std::round(ideal_num_joint_samples2)));

  // If the number of samples must be odd, we need to ensure that the sum of the
  // two rounded numbers of joint samples is even.
  if (force_odd_number_of_samples) {
    if ((rounded_num_joint_samples_1 + rounded_num_joint_samples_2) % 2 != 0) {
      // Change the number of samples corresponding to the shortest sampling
      // step or denser half-path.
      int& rounded_num_joint_samples_to_update =
          (joint_sampling_step_rad_1 < joint_sampling_step_rad_2)
              ? rounded_num_joint_samples_1
              : rounded_num_joint_samples_2;

      // Reduce by one if possible, otherwise increase by one.
      rounded_num_joint_samples_to_update +=
          (rounded_num_joint_samples_to_update > 1) ? -1 : 1;
    }
  }

  // Compute the actual joint sampling steps.
  const double actual_uniform_joint_sampling_step_rad_1 =
      half_path_length / rounded_num_joint_samples_1;
  const double actual_uniform_joint_sampling_step_rad_2 =
      half_path_length / rounded_num_joint_samples_2;

  // Compute the target joint lengths to be sampled.
  std::vector<double> target_joint_lengths;
  target_joint_lengths.reserve(rounded_num_joint_samples_1 +
                               rounded_num_joint_samples_2 + 1);
  for (int i = 1; i <= rounded_num_joint_samples_1; ++i) {
    target_joint_lengths.push_back(static_cast<double>(i) *
                                   actual_uniform_joint_sampling_step_rad_1);
  }
  const double offset_path_length = target_joint_lengths.back();
  for (int i = 1; i < rounded_num_joint_samples_2; ++i) {
    target_joint_lengths.push_back(
        offset_path_length +
        static_cast<double>(i) * actual_uniform_joint_sampling_step_rad_2);
  }

  // Run resampling step to get the interpolated normalized curve parameters
  // that lead to an approximate uniform sampling of the joint samples.
  return ComputeNormalizedCurveParametersAtTargetJointLengths(
      normalized_curve_parameters, accumulated_joint_dist,
      target_joint_lengths);
}

}  // namespace

absl::StatusOr<std::vector<double>> ApproximateJointUniformSampling(
    absl::Span<const double> normalized_curve_parameters,
    absl::Span<const eigenmath::VectorNd> joint_configurations,
    absl::Span<const double> joint_sampling_steps_rad,
    bool force_odd_number_of_samples) {
  if (joint_sampling_steps_rad.size() == 1) {
    return ApproximateJointUniformSampling(
        normalized_curve_parameters, joint_configurations,
        joint_sampling_steps_rad.front(), force_odd_number_of_samples);
  } else if (joint_sampling_steps_rad.size() == 2) {
    return ApproximateJointUniformSampling(
        normalized_curve_parameters, joint_configurations,
        joint_sampling_steps_rad.front(), joint_sampling_steps_rad.back(),
        force_odd_number_of_samples);
  }

  return absl::InvalidArgumentError(absl::StrCat(
      "The size of `joint_sampling_steps_rad` must be 1 or 2, got: ",
      joint_sampling_steps_rad.size(), "."));
}

absl::StatusOr<UpsampledCurveParameters> CreateUpsampledCurveParameters(
    absl::Span<const double> sparse_curve_parameters,
    const int upsampling_factor) {
  if (upsampling_factor <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Upsampling factor must be positive, got: ", upsampling_factor, "."));
  }
  if (upsampling_factor == 1) {
    return absl::InvalidArgumentError(
        "Upsampling factor equal to 1 means there is no upsampling done.");
  }
  UpsampledCurveParameters result;
  result.dense_curve_parameters.reserve(sparse_curve_parameters.size() *
                                        upsampling_factor);
  result.sparse_curve_parameters_indices_in_dense_vector.reserve(
      sparse_curve_parameters.size());
  for (int i = 1; i < sparse_curve_parameters.size(); ++i) {
    const double previous_curve_parameter = sparse_curve_parameters[i - 1];
    const double next_curve_parameter = sparse_curve_parameters[i];
    result.sparse_curve_parameters_indices_in_dense_vector.push_back(
        result.dense_curve_parameters.size());
    for (int j = 0; j < upsampling_factor; ++j) {
      const double ratio = static_cast<double>(j) / upsampling_factor;
      const double interpolated_curve_parameter =
          previous_curve_parameter +
          ratio * (next_curve_parameter - previous_curve_parameter);
      result.dense_curve_parameters.push_back(interpolated_curve_parameter);
    }
  }
  result.dense_curve_parameters.push_back(sparse_curve_parameters.back());
  result.sparse_curve_parameters_indices_in_dense_vector.push_back(
      result.dense_curve_parameters.size() - 1);
  return result;
}

absl::StatusOr<std::vector<eigenmath::VectorNd>> FilterJointConfigurations(
    absl::Span<const eigenmath::VectorNd> dense_joint_configurations,
    absl::Span<const int> indices_to_keep) {
  std::vector<eigenmath::VectorNd> filtered_joint_configurations;
  filtered_joint_configurations.reserve(indices_to_keep.size());
  for (int i : indices_to_keep) {
    if (i < 0 || i >= dense_joint_configurations.size()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Index to keep is out of bounds: ", i, ", valid range is [0, ",
          dense_joint_configurations.size() - 1, "]."));
    }
    filtered_joint_configurations.push_back(dense_joint_configurations[i]);
  }
  return filtered_joint_configurations;
}

}  // namespace intrinsic
