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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_PARAMETRIC_CURVE_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_PARAMETRIC_CURVE_UTILS_H_

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/joint_parametric_curve.h"

namespace intrinsic {

using JointBlendingArcParameters = BlendingArcParameters<eigenmath::VectorNd>;
using CartesianBlendingArcParameters = BlendingArcParameters<Pose3d>;
using JointBlendingArcKinematicParameters =
    BlendingArcParameters<JointStatePVA, JointStateP>;

// A function that for a given `joint_space_path` returns a vector of
// joint-space uniform resampling steps. The function can return `std::nullopt`
// to mean that no resampling should be performed. Otherwise, the returned
// vector of resampling steps is used to resample the joint-space path.
using UniformResamplingStepsForJointSpacePathFunction =
    std::function<absl::StatusOr<std::optional<std::vector<double>>>(
        absl::Span<const eigenmath::VectorNd> /*joint_space_path*/)>;

struct JointSpaceUniformArcLengthPreservingSamplingOptions {
  // Optional distance at which the joint configuration path will be uniformly
  // sampled in joint space. Note that the resulting sampling distance is
  // adjusted to produce an uneven number of samples. If set, an approximately
  // uniform joint sampling scheme is applied. Otherwise, no resampling is
  // applied. There is currently no default value (thus, non-uniform sampling is
  // the default behaviour). As a reference to select this value, Cartesian
  // paths are resampled at sampling steps of 0.05 rad.
  std::optional<double> sampling_distance_rad = std::nullopt;

  // Optional maximum number of samples for uniform sampling of the joint
  // configuration path. If `sampling_distance_rad` is set, this parameter only
  // upper bounds the number of samples produced. If `sampling_distance_rad` is
  // unset, this parameter has no effect.
  std::optional<int> max_num_samples = std::nullopt;

  // Computes the implied (by the combination of `sampling_distance_rad`,
  // max_num_samples, and `path_length_rad`) joint-space sampling distance to
  // achieve a uniform sampling of the path of given `path_length_rad`. The
  // implied sampling distance is computed as the maximum between the
  // user-provided `sampling_distance_rad` and the sampling distance implied by
  // the path length and the `max_num_samples`.
  absl::StatusOr<std::optional<double>> ImpliedSamplingDistanceRad(
      double path_length_rad) const;
};

// Computes a normalized sampling step in the range (0.0, 1.0] for sampling a
// `JointParametricCurve` of given `sampling_step_rad` and `curve_length_rad`.
// The result is the minimum normalized sampling step that guarantees that the
// curve is sampled with a step no wider than `sampling_step_rad`. Returns an
// error status if either of `sampling_step_rad` or `curve_length_rad` is not
// strictly positive, or if `curve_length_rad` is infinite.
absl::StatusOr<double> NormalizedSamplingStepForJointCurve(
    double sampling_step_rad, double curve_length_rad);

// Creates a `UniformResamplingStepsForJointSpacePathFunction` that defines
// resampling to be a uniform in joint space arc-length preserving sampling
// performed according to the provided `joint_space_uniform_sampling_options`
// with a unique sampling distance for the entire path.
UniformResamplingStepsForJointSpacePathFunction
CreateUniformResamplingStepsForJointSpacePathFunction(
    const JointSpaceUniformArcLengthPreservingSamplingOptions&
        joint_space_uniform_sampling_options);

// Performs sampling on a `joint_curve` with the given
// `joint_curve_sampling_distance_rad`. The curve will be uniformly-sampled with
// the minimum normalized sampling step ensuring that the curve is sampled with
// a step no wider than `joint_curve_sampling_distance_rad`. The function
// `resampling_steps_function` allows to compute the sampling steps for uniform
// resampling the joint configuration path (if desired). If
// `force_odd_number_of_samples` is true, the curve will be sampled such that
// the number of samples is odd.
absl::StatusOr<std::vector<eigenmath::VectorNd>>
SampleJointCurveAndResampleForUniformJointSampling(
    const JointParametricCurve& joint_curve,
    double joint_curve_sampling_distance_rad,
    const UniformResamplingStepsForJointSpacePathFunction&
        resampling_steps_function,
    bool force_odd_number_of_samples = true);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_PARAMETRIC_CURVE_UTILS_H_
