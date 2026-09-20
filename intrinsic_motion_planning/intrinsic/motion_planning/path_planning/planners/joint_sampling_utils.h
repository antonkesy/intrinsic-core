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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_JOINT_SAMPLING_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_JOINT_SAMPLING_UTILS_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Returns a vector of interpolated normalized curve parameters which achieve an
// approximate uniform sampling in joint space. The
// `normalized_curve_parameters` and `joint_configurations` represent the
// path progress and joint configurations, respectively, (e.g. representing the
// inverse kinematics solution sequence of a Cartesian path, or a joint path
// sampled by a `JointParametricCurve`) to be uniformly sampled in joint space
// with the given `joint_sampling_steps_rad`. If a single joint sampling step is
// provided, it is assumed that this value is the same for the entire arc-length
// of the `joint_configurations`. If two joint sampling steps are provided,
// the function will use the first ()`joint_sampling_steps_rad[0]`) sampling
// steps to sample the first half of the arc-length implied by the
// `joint_configurations` and the second ()`joint_sampling_steps_rad[1]`)
// sampling step to sample the second half of the arc-length of the
// `joint_configurations`. More values in `joint_sampling_steps_rad` are not
// supported. If the flag `force_odd_number_of_samples` is set to true, the
// function will ensure that the resulting vector of interpolated normalized
// curve parameters has an odd/uneven number of samples and to that end, it will
// update the actual spacing between the samples (which will be selected close
// to `joint_sampling_steps_rad`, but will differ from it due to the odd number
// of samples requirement).
//
// Returns an error if
// - the `joint_sampling_step_rad` is not positive,
// - the size of `normalized_curve_parameters` and `joint_configurations` do
//   not match,
// - the size of all elements in `joint_configurations` are not the same,
// - the size of `joint_sampling_steps_rad` is not 1 or 2,
// - the normalized curve parameters are not sorted.
// - the normalized curve parameters are outside the range [0.0, 1.0].
absl::StatusOr<std::vector<double>> ApproximateJointUniformSampling(
    absl::Span<const double> normalized_curve_parameters,
    absl::Span<const eigenmath::VectorNd> joint_configurations,
    absl::Span<const double> joint_sampling_steps_rad,
    bool force_odd_number_of_samples = true);

// Structure to store the result of upsampling a curve parameters vector. See
// the function `CreateUpsampledCurveParameters` for more details.
struct UpsampledCurveParameters {
  std::vector<double> dense_curve_parameters;
  std::vector<int> sparse_curve_parameters_indices_in_dense_vector;
};

// Starting from a vector of `sparse_curve_parameters`, this function creates a
// denser version with the desired `upsampling_factor`. The result is an
// `UpsampledCurveParameters` object that contains a vector of
// `dense_curve_parameters` and a vector of
// `sparse_curve_parameters_indices_in_dense_vector`.
// The `dense_curve_parameters` vector is created by taking each pair of
// consecutive parameters from the `sparse_curve_parameters` vector and
// interpolating in between them with an additional `upsampling_factor` number
// of parameters. The `sparse_curve_parameters_indices_in_dense_vector`
// represent the indices of the original values in the sparse parameters vector
// in the `dense_curve_parameters` vector. The dense curve parameters will be
// used in all computations in place of the sparse curve parameters, but at the
// end we return only a subset, which are the ones corresponding to the indices
// in `sparse_curve_parameters_indices_in_dense_vector`.
//
// For instance, if the sparse curve parameters are {0.0, 1.0}, and the density
// factor to generate dense curve parameters is 10. We would have the following
// values:
// - dense curve parameters: {0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
//                            0.6, 0.7, 0.8, 0.9, 1.0}.
// - indices sparse curve parameters in dense vector: {0, 10}.
// so that the sparse parameters can be mapped to the dense parameters.
absl::StatusOr<UpsampledCurveParameters> CreateUpsampledCurveParameters(
    absl::Span<const double> sparse_curve_parameters, int upsampling_factor);

// Filters the given `dense_joint_configurations` vector by keeping only the
// joint configurations at the indices specified in `indices_to_keep`. Returns
// an error if any of the indices is out of bounds.
absl::StatusOr<std::vector<eigenmath::VectorNd>> FilterJointConfigurations(
    absl::Span<const eigenmath::VectorNd> dense_joint_configurations,
    absl::Span<const int> indices_to_keep);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_JOINT_SAMPLING_UTILS_H_
