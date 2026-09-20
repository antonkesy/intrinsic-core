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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_IK_PATH_IK_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_IK_PATH_IK_H_

#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/parametric_curve_utils.h"

namespace intrinsic {

// Computes the inverse kinematics at each of the Cartesian waypoints in
// `base_t_target_waypoints` for the given `kinematics`. The
// `hint_joint_configuration` is used as initial hint to compute the ik at the
// first waypoint. Thereinafter, every ik solution is used as hint for the
// computation of the next one. The waypoints are assumed to represent desired
// poses for the target frame of the `kinematics`, with `tip_t_target`
// expressing the offset between tip and target frames. Setting
// `ensure_same_branch_ik` to true (defaults to false) requires the ik solver to
// select solutions that are in the same partition of the configuration space as
// the `hint_joint_configuration`. Every ik solution is guaranteed to satisfy
// the joint position limits of the `kinematics`. Alternatively, custom joint
// position limits can be provided in `joint_limits`.
absl::StatusOr<std::vector<eigenmath::VectorNd>> CoarsePathIk(
    const icon::ManipulatorKinematics* kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_waypoints,
    const Pose3d& tip_t_target, bool ensure_same_branch_ik = false,
    const std::optional<JointLimits> joint_limits = std::nullopt);

// Same as above, except that the kinematics is accessed via a
// KinematicsSystemProxy, which works with dynamic size Eigen types.
absl::StatusOr<std::vector<eigenmath::VectorXd>> CoarsePathIk(
    const KinematicsSystemProxy& kinematics_proxy,
    const eigenmath::VectorXd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_waypoints,
    const Pose3d& tip_t_target, bool ensure_same_branch_ik = false,
    const std::optional<JointLimitsXd> joint_limits = std::nullopt);

// Computes the inverse kinematics on a finely interpolated Cartesian path
// described by the poses in `base_t_target_path` for the given `kinematics`.
// The samples in `base_t_target_path` represent desired poses for the
// target frame of the `kinematics`, with `tip_t_target` expressing the offset
// between tip and target frames. Moreover, the poses are assumed to be given
// with the specified `control_frequency_hz`, which is used alongside the
// joint acceleration limits to compute hints for the ik solver. The
// `hint_joint_configuration` is used as the IK hint for the first Cartesian
// path sample. If `validate_first_ik_solution_against_hint_joint_configuration`
// is enabled, the first IK solution is expected to be similar to
// `hint_joint_configuration`, up to `kJointDeviationToleranceRad` numerical
// error in the sense of the L-infinity norm (i.e., the maximum among the
// absolute individual joint/degree-of-freedom error). Otherwise, an error will
// be returned. Every ik solution is guaranteed to satisfy the joint position
// limits of the `kinematics`. Custom joint limits can be provided in
// `joint_limits`.
absl::StatusOr<std::vector<eigenmath::VectorNd>> FinePathIk(
    const icon::ManipulatorKinematics* kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_path, const Pose3d& tip_t_target,
    double control_frequency_hz,
    const std::optional<JointLimits> joint_limits = std::nullopt,
    bool validate_first_ik_solution_against_hint_joint_configuration = false);

// Same as above, except that the kinematics is accessed via a
// KinematicsSystemProxy, which works with dynamic size Eigen types.
absl::StatusOr<std::vector<eigenmath::VectorXd>> FinePathIk(
    const KinematicsSystemProxy& kinematics_proxy,
    const eigenmath::VectorXd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_path, const Pose3d& tip_t_target,
    double control_frequency_hz,
    const std::optional<JointLimitsXd> joint_limits = std::nullopt,
    bool validate_first_ik_solution_against_hint_joint_configuration = false);

// Same as above, except that the kinematics is given as ManipulatorKinematics
// and the input path as Cartesian parametric curve. The curve will be uniformly
// sampled with the minimum normalized sampling step ensuring that the
// translational and rotational parts of the curve are sampled with a step no
// wider than `sampling_step_translation_m` and `sampling_step_rotation_rad`,
// respectively. If `force_odd_number_of_samples` is true, the curve will be
// sampled so that the number of samples is odd by possibly increasing the
// number of samples by one.
absl::StatusOr<std::vector<eigenmath::VectorNd>> FinePathIk(
    const icon::ManipulatorKinematics* kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    const CartesianParametricCurve& base_t_target_path,
    const Pose3d& tip_t_target, double sampling_step_translation_m,
    double sampling_step_rotation_rad, double control_frequency_hz,
    const std::optional<JointLimits> joint_limits = std::nullopt,
    bool force_odd_number_of_samples = false,
    bool validate_first_ik_solution_against_hint_joint_configuration = false);

// Same as above, except that the kinematics is accessed via a
// KinematicsSystemProxy, which works with dynamic size Eigen types.
absl::StatusOr<std::vector<eigenmath::VectorXd>> FinePathIk(
    const KinematicsSystemProxy& kinematics_proxy,
    const eigenmath::VectorXd& hint_joint_configuration,
    const CartesianParametricCurve& base_t_target_path,
    const Pose3d& tip_t_target, double sampling_step_translation_m,
    double sampling_step_rotation_rad, double control_frequency_hz,
    const std::optional<JointLimitsXd> joint_limits = std::nullopt,
    bool force_odd_number_of_samples = false,
    bool validate_first_ik_solution_against_hint_joint_configuration = false);

// Returns the path IK solutions for the given Cartesian path
// `base_t_target_path`, with the tip-to-target pose `tip_t_target` and the
// control frequency `path_ik_control_frequency_hz`. The
// `hint_joint_configuration` is used as the IK hint for the first Cartesian
// path sample. If `validate_first_ik_solution_against_hint_joint_configuration`
// is enabled, the first IK solution is expected to be similar to
// `hint_joint_configuration`, up to `kJointDeviationToleranceRad` numerical
// error in the sense of the L-infinity norm (i.e., the maximum among the
// absolute individual joint/degree-of-freedom error). Otherwise, an error will
// be returned. Returns a verbose error if the path IK cannot be computed.
absl::StatusOr<std::vector<eigenmath::VectorNd>> PathIkWithErrorHandling(
    const icon::ManipulatorKinematics* manipulator_kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_path, const Pose3d& tip_t_target,
    double path_ik_control_frequency_hz,
    const std::optional<JointLimits> joint_limits = std::nullopt,
    bool validate_first_ik_solution_against_hint_joint_configuration = false);

// Performs path IK on a Cartesian curve with the given
// `translational_path_sampling_distance_m` and
// `rotational_path_sampling_distance_rad`, given a `manipulator_kinematics`.
// The curve will be uniformly-sampled with the minimum normalized sampling step
// ensuring that the translational and rotational parts of the curve are sampled
// with a step no wider than `translational_path_sampling_distance_m` and
// `rotational_path_sampling_distance_rad`, respectively. The
// `resampling_steps_function` allows to compute the resampling steps that will
// be used to resample the IK joint-space path to achieve a uniform-sampling in
// joint-space. If the `resampling_steps_function` returns std::nullopt, then no
// resampling will be performed. The `hint_joint_configuration` is used as the
// IK hint for the first Cartesian path sample. If
// `validate_first_ik_solution_against_hint_joint_configuration` is enabled, the
// first IK solution is expected to be similar to `hint_joint_configuration`, up
// to `kJointDeviationToleranceRad` numerical error in the sense of the
// L-infinity norm (i.e., the maximum among the absolute individual joint/
// degree-of-freedom error). Otherwise, an error will be returned. If
// `force_odd_number_of_samples` is true, the curve will be sampled such that
// the number of samples is odd.
absl::StatusOr<std::vector<eigenmath::VectorNd>>
PathIkOnCartesianCurveAndResampleForUniformJointSampling(
    const CartesianParametricCurve& cartesian_curve,
    double translational_path_sampling_distance_m,
    double rotational_path_sampling_distance_rad,
    const icon::ManipulatorKinematics* manipulator_kinematics,
    const eigenmath::VectorXd& hint_joint_configuration,
    const Pose3d& tip_t_target,
    const UniformResamplingStepsForJointSpacePathFunction&
        resampling_steps_function,
    const std::optional<JointLimits> joint_limits = std::nullopt,
    bool force_odd_number_of_samples = true,
    bool validate_first_ik_solution_against_hint_joint_configuration = false);

// Performs path IK on a Cartesian curve with the given
// `translational_path_sampling_distance_m` and
// `rotational_path_sampling_distance_rad`, given a `kinematics_proxy`.
// The curve will be uniformly-sampled with the minimum normalized sampling step
// ensuring that the translational and rotational parts of the curve are sampled
// with a step no wider than `translational_path_sampling_distance_m` and
// `rotational_path_sampling_distance_rad`, respectively. The
// `resampling_steps_function` allows to compute the resampling steps that will
// be used to resample the IK joint-space path to achieve a uniform-sampling in
// joint-space. If the `resampling_steps_function` returns std::nullopt, then no
// resampling will be performed. The `hint_joint_configuration` is used as the
// IK hint for the first Cartesian path sample. If
// `validate_first_ik_solution_against_hint_joint_configuration` is enabled, the
// first IK solution is expected to be similar to `hint_joint_configuration`, up
// to `kJointDeviationToleranceRad` numerical error in the sense of the
// L-infinity norm (i.e., the maximum among the absolute individual joint/
// degree-of-freedom error). Otherwise, an error will be returned. If
// `force_odd_number_of_samples` is true, the curve will be sampled such that
// the number of samples is odd.
absl::StatusOr<std::vector<eigenmath::VectorNd>>
PathIkOnCartesianCurveAndResampleForUniformJointSampling(
    const CartesianParametricCurve& cartesian_curve,
    double translational_path_sampling_distance_m,
    double rotational_path_sampling_distance_rad,
    const KinematicsSystemProxy& kinematics_proxy,
    const eigenmath::VectorXd& hint_joint_configuration,
    const Pose3d& tip_t_target,
    const UniformResamplingStepsForJointSpacePathFunction&
        resampling_steps_function,
    const std::optional<JointLimitsXd> joint_limits = std::nullopt,
    bool force_odd_number_of_samples = true,
    bool validate_first_ik_solution_against_hint_joint_configuration = false);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_IK_PATH_IK_H_
