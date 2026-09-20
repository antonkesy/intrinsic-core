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

#include "intrinsic/motion_planning/path_planning/path_ik/path_ik.h"

#include <cstddef>
#include <memory>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/pose3_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_newton_raphson_ik_solver.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/to_fixed_string.h"
#include "intrinsic/kinematics/utils/compute_ik_util.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/motion_planner/motion_planning_error_utils.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve_utils.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/parametric_curve_utils.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/path_planning_utils.h"
#include "intrinsic/motion_planning/path_planning/planners/joint_sampling_utils.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/path_refinement/path_refinement_utils.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

using IKResult = kinematics::InverseKinematicsInterface::IKResult;

constexpr int kDebugLogLevel = 2;

// Max. allowed translational and rotational distance between consecutive input
// Cartesian samples in FinePathIk.
constexpr double kMaxSampleTranslationDistanceM = 0.1;
constexpr double kMaxSampleRotationDistanceRad = 0.2;  // about 11 deg.

namespace {

absl::Status ValidateKinematics(const icon::ManipulatorKinematics* kinematics) {
  if (!kinematics) {
    return absl::InvalidArgumentError("Got a null pointer as kinematics.");
  }
  return absl::OkStatus();
}

absl::Status ValidateHintJointConfiguration(
    const eigenmath::VectorNd& hint_joint_configuration, int kinematics_num_dof,
    const JointLimits& joint_limits) {
  if (hint_joint_configuration.size() != kinematics_num_dof) {
    return absl::InvalidArgumentError(
        absl::StrCat("Size mismatch. Got hint joint configuration of size ",
                     hint_joint_configuration.size(), " with a ",
                     kinematics_num_dof, "-DoF manipulator kinematics."));
  }

  if (!joint_limits.IsValid()) {
    return absl::InvalidArgumentError("Got invalid joint limits.");
  }

  if (hint_joint_configuration.size() != joint_limits.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Size mismatch. Got joint limits of size ", joint_limits.size(),
        "with a hint joint configuration of size ",
        hint_joint_configuration.size(), "."));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const bool configuration_within_limits,
      IsWithinLimits(hint_joint_configuration, joint_limits));
  if (!configuration_within_limits) {
    return absl::InvalidArgumentError(
        "The hint joint configuration violates the joint position limits.");
  }

  return absl::OkStatus();
}

// Perform validation/sanity-check on the "delta" between the new `ik_solution`
// and the expected-closest joint configuration --which is either the
// `hint_joint_configuration` (if `path_ik_solutions_so_far` is empty) or the
// last IK solution joint configuration in `path_ik_solutions_so_far`-- to catch
// any large un-expected configuration change. If `path_ik_solutions_so_far` is
// empty (and therefore `hint_joint_configuration` will be validated against),
// the validation is only done if and only if
// `validate_first_ik_solution_against_hint_joint_configuration` is true.
absl::Status ValidateIkSolutionForFinePathIk(
    absl::Span<const eigenmath::VectorNd> path_ik_solutions_so_far,
    const eigenmath::VectorNd& hint_joint_configuration,
    const eigenmath::VectorNd& ik_solution,
    const bool validate_first_ik_solution_against_hint_joint_configuration) {
  constexpr double kMaxJointLimitStep = std::numbers::pi / 4.0;
  constexpr absl::string_view kPrefixErrorMessageForFirstIkSolution =
      "FinePathIK Error: The initial solution does not match the provided hint "
      "joint configuration, although expected. Likely cause: Internal error in "
      "the underlying Inverse Kinematics (IK) solver. Please file a bug along "
      "with the corresponding Motion Planning Logging ID to help with this "
      "issue's reproductions during the debugging.";
  constexpr absl::string_view kPrefixErrorMessageForRemainingIkSolution =
      "Could not solve FinePathIK without excessive change in joint config. "
      "This might happen when planning close to joint limits or singular "
      "configurations. To avoid this error, please try with other start and/or "
      "end pose(s), joint configuration(s), and/or constraint(s) which are "
      "further away from joint limits and/or singular joint configurations.";

  const eigenmath::VectorNd expected_closest_joint_configuration =
      path_ik_solutions_so_far.empty() ? hint_joint_configuration
                                       : path_ik_solutions_so_far.back();

  // The `tolerance` below varies between two (2) cases:
  // * for the first IK solution versus the hint joint configuration, we expect
  //   the error/"delta" to be numerical (i.e. very small) not bigger than
  //   `kJointDeviationToleranceRad`, because the first IK solution has to match
  //   the hint joint configuration precisely.
  // * for the subsequent IK solution versus the IK solution right before it (in
  //   the previous iteration), we expect some error/"delta" but not bigger than
  //   `kMaxJointLimitStep`. Please note that near singularity, these
  //   error/"delta" can be bigger than that of the joint configurations
  //   further-away from singularity.
  const double tolerance = path_ik_solutions_so_far.empty()
                               ? kJointDeviationToleranceRad
                               : kMaxJointLimitStep;

  std::string error_message(path_ik_solutions_so_far.empty()
                                ? kPrefixErrorMessageForFirstIkSolution
                                : kPrefixErrorMessageForRemainingIkSolution);

  // We are measuring the maximum absolute joint configuration error across
  // all joints/degrees-of-freedom, and therefore we are using the L-infinity
  // norm here.
  const double maximum_absolute_joint_configuration_error =
      (ik_solution - expected_closest_joint_configuration)
          .lpNorm<Eigen::Infinity>();
  if ((validate_first_ik_solution_against_hint_joint_configuration ||
       !path_ik_solutions_so_far.empty()) &&
      (maximum_absolute_joint_configuration_error >= tolerance)) {
    // Also add some additional error information to assist future debugging, if
    // needed.
    std::stringstream additional_error_info_sstream;
    additional_error_info_sstream << " ik_solution: "
                                  << ik_solution.transpose();
    additional_error_info_sstream
        << "; expected_closest_joint_configuration: "
        << expected_closest_joint_configuration.transpose();
    additional_error_info_sstream
        << "; maximum_absolute_joint_configuration_error="
        << maximum_absolute_joint_configuration_error;
    additional_error_info_sstream << " >= tolerance=" << tolerance;
    absl::StrAppend(&error_message, additional_error_info_sstream.str());
    return absl::InvalidArgumentError(error_message);
  }
  return absl::OkStatus();
}

absl::StatusOr<std::vector<Pose3d>> SampleCartesianParametricCurveUniformly(
    const CartesianParametricCurve& curve, double sampling_step_translation_m,
    double sampling_step_rotation_rad, bool force_odd_number_of_samples) {
  if (sampling_step_translation_m > kMaxSampleTranslationDistanceM) {
    return absl::InvalidArgumentError(
        absl::StrCat("The sampling step for translation is too large. Max is ",
                     kMaxSampleTranslationDistanceM, "m, got ",
                     sampling_step_translation_m, "m."));
  }
  if (sampling_step_rotation_rad > kMaxSampleRotationDistanceRad) {
    return absl::InvalidArgumentError(
        absl::StrCat("The sampling step for rotation is too large. Max is ",
                     kMaxSampleRotationDistanceRad, "rad, got ",
                     sampling_step_rotation_rad, "rad."));
  }

  INTR_ASSIGN_OR_RETURN(
      const double normalized_sampling_step,
      NormalizedSamplingStepForCartesianCurve(sampling_step_translation_m,
                                              sampling_step_rotation_rad,
                                              curve.GetCurveLength()));
  INTR_ASSIGN_OR_RETURN(const std::vector<double> normalized_path_params,
                        topp::ComputeUniformlyDiscretizedPathVariables(
                            /*path_length=*/1.0, normalized_sampling_step,
                            force_odd_number_of_samples));

  return curve.Sample(normalized_path_params);
}

// Returns a pair holding the index of the joint that is closest to its
// `limits` (either upper or lower limit) and the related minimum distance.
std::pair<size_t, double> GetClosestJointToLimits(
    const eigenmath::VectorNd& joint_values, const JointLimits& limits) {
  const eigenmath::VectorNd dist_from_upper_limits =
      limits.max_position - joint_values;
  const eigenmath::VectorNd dist_from_lower_limits =
      joint_values - limits.min_position;

  eigenmath::VectorNd::Index min_index;
  const double min_dist_from_limits =
      dist_from_upper_limits.cwiseMin(dist_from_lower_limits)
          .minCoeff(&min_index);

  return std::make_pair(min_index, min_dist_from_limits);
}

absl::Status ComputeAndLogDataForDebugging(
    const icon::ManipulatorKinematics* kinematics,
    const eigenmath::VectorNd& prev_ik_solution,
    const eigenmath::VectorNd& curr_ik_hint,
    const eigenmath::VectorNd& curr_ik_solution,
    const Pose3d& base_t_tip_desired, const JointLimits& joint_limits) {
  // Get Ik branch for previous ik solution, ik hint and current ik
  // solution (if available).
  std::string branch_prev_ik_solution_string, branch_curr_ik_hint_string,
      branch_curr_ik_solution_string;
  if (kinematics->GetInverseKinematicsSolver().ImplementsSameBranchIK()) {
    INTR_ASSIGN_OR_RETURN(
        const size_t branch_prev_ik_solution,
        kinematics->GetInverseKinematicsSolver().ComputeBranch(
            prev_ik_solution));
    INTR_ASSIGN_OR_RETURN(
        const size_t branch_curr_ik_hint,
        kinematics->GetInverseKinematicsSolver().ComputeBranch(curr_ik_hint));
    INTR_ASSIGN_OR_RETURN(
        const size_t branch_curr_ik_solution,
        kinematics->GetInverseKinematicsSolver().ComputeBranch(
            curr_ik_solution));
    branch_prev_ik_solution_string =
        " -> Branch id: " + std::to_string(branch_prev_ik_solution);
    branch_curr_ik_hint_string =
        " -> Branch id: " + std::to_string(branch_curr_ik_hint);
    branch_curr_ik_solution_string =
        " -> Branch id: " + std::to_string(branch_curr_ik_solution);
  }

  // Collect data for joint limits and singularity detection.
  const std::pair<size_t, double> closest_joint_to_limits =
      GetClosestJointToLimits(curr_ik_hint, joint_limits);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double min_sv_prev_ik_solution,
      GetJacobianSmallestSingularValue(kinematics, prev_ik_solution));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double min_sv_curr_ik_hint,
      GetJacobianSmallestSingularValue(kinematics, curr_ik_hint));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double min_sv_curr_ik_solution,
      GetJacobianSmallestSingularValue(kinematics, curr_ik_solution));

  // Compute all possible ik solutions.
  constexpr int kMaxIkSolutions = 8;
  std::vector<JointStateP> ik_solutions(kMaxIkSolutions);
  INTR_ASSIGN_OR_RETURN(IKResult ik_result,
                        kinematics->GetInverseKinematicsSolver().ComputeIK(
                            curr_ik_hint, base_t_tip_desired, joint_limits,
                            absl::MakeSpan(ik_solutions)));

  std::string all_ik_solutions_string = "Available IK solutions: \n";
  if (ik_result.status ==
      kinematics::InverseKinematicsInterface::IKResult::OK) {
    for (auto& ik : ik_solutions) {
      if (ik.size() > 0) {
        all_ik_solutions_string += toString(ik.position);
        int branch = -1;
        if (kinematics->GetInverseKinematicsSolver().ImplementsSameBranchIK()) {
          INTR_ASSIGN_OR_RETURN(
              branch,
              kinematics->GetInverseKinematicsSolver().ComputeBranch(ik));
          all_ik_solutions_string += " -> Branch id: ";
          all_ik_solutions_string += std::to_string(branch);
        }
        all_ik_solutions_string += "\n";
      }
    }
  }

  DVLOG(kDebugLogLevel)
      << "Debug info for base_t_tip pose: " << base_t_tip_desired << "\n"
      << "Prev. IK solution: " << prev_ik_solution.transpose()
      << branch_prev_ik_solution_string << "\n"
      << "Curr. IK hint    : " << curr_ik_hint.transpose()
      << branch_curr_ik_hint_string << "\n"
      << "Curr. IK solution: " << curr_ik_solution.transpose()
      << branch_curr_ik_solution_string << "\n"
      << "IK hint min. distance from joint limits: "
      << closest_joint_to_limits.second << " on joint idx "
      << closest_joint_to_limits.first << "\n"
      << "Min. Jacobian singular value (prev. IK solution, IK hint, "
         "curr. IK solution): "
      << min_sv_prev_ik_solution << ", " << min_sv_curr_ik_hint << ", "
      << min_sv_curr_ik_solution << "\n"
      << all_ik_solutions_string << "\n\n";

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<std::vector<eigenmath::VectorNd>> CoarsePathIk(
    const icon::ManipulatorKinematics* kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_waypoints,
    const Pose3d& tip_t_target, bool ensure_same_branch_ik,
    const std::optional<JointLimits> joint_limits) {
  INTR_RETURN_IF_ERROR(ValidateKinematics(kinematics));

  const JointLimits applied_joint_limits = joint_limits.value_or(
      kinematics->GetKinematicsModel().GetDofSystemLimits());

  INTR_RETURN_IF_ERROR(ValidateHintJointConfiguration(
      hint_joint_configuration,
      kinematics->GetKinematicsModel().GetNumberDegreesOfFreedom(),
      applied_joint_limits));

  if (base_t_target_waypoints.empty()) {
    return absl::InvalidArgumentError("Got an empty sequence of waypoints.");
  }

  std::vector<eigenmath::VectorNd> path_ik_solutions;
  path_ik_solutions.reserve(base_t_target_waypoints.size());
  eigenmath::VectorNd ik_hint = hint_joint_configuration;
  for (const auto& base_t_target : base_t_target_waypoints) {
    const Pose3d base_t_tip_desired_pose =
        base_t_target * tip_t_target.inverse();
    JointStateP ik_solution;

    INTR_ASSIGN_OR_RETURN(
        IKResult ik_result,
        kinematics->GetInverseKinematicsSolver().ComputeIK(
            ik_hint, base_t_tip_desired_pose, applied_joint_limits,
            ensure_same_branch_ik, &ik_solution));

    if (ik_result.status != IKResult::OK) {
      return absl::InternalError(
          absl::StrCat("Could not compute the IK for Cartesian waypoint with "
                       "transform base_t_target=",
                       toString(base_t_target), "."));
    }

    path_ik_solutions.push_back(ik_solution.position);
    ik_hint = ik_solution.position;
  }
  return path_ik_solutions;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> CoarsePathIk(
    const KinematicsSystemProxy& kinematics_proxy,
    const eigenmath::VectorXd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_waypoints,
    const Pose3d& tip_t_target, bool ensure_same_branch_ik,
    const std::optional<JointLimitsXd> joint_limits) {
  // TODO(b/279222142) rewrite this function body to directly work with
  // KinematicsSystemProxy. For now, a ManipulatorKinematics is extracted from
  // the proxy and the above function is called. This also implies that the size
  // of the hint_joint_configuration should not exceed eigenmath::VectorNd max
  // size. If provided, joint limits additionally need also to be converted from
  // JointLimitsXd to JointLimits (will fail if eigenmath::VectorNd max size is
  // exceeded).
  if (hint_joint_configuration.size() >
      eigenmath::VectorNd::MaxSizeAtCompileTime) {
    return absl::InvalidArgumentError(
        absl::StrCat("Can not handle hint joint configuration of size ",
                     hint_joint_configuration.size(), ". Max size is ",
                     eigenmath::VectorNd::MaxSizeAtCompileTime, "."));
  }

  std::optional<JointLimits> joint_limits_fixed_size = std::nullopt;
  if (joint_limits.has_value()) {
    INTR_ASSIGN_OR_RETURN(joint_limits_fixed_size,
                          ToJointLimits(joint_limits.value()));
  }

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<icon::ManipulatorKinematics> kinematics,
                        kinematics_proxy.GetManipulatorKinematics());

  INTR_ASSIGN_OR_RETURN(
      std::vector<eigenmath::VectorNd> path_ik_solutions,
      CoarsePathIk(kinematics.get(), hint_joint_configuration,
                   base_t_target_waypoints, tip_t_target, ensure_same_branch_ik,
                   joint_limits_fixed_size));

  return std::vector<eigenmath::VectorXd>{path_ik_solutions.begin(),
                                          path_ik_solutions.end()};
}

absl::StatusOr<std::vector<eigenmath::VectorNd>> FinePathIk(
    const icon::ManipulatorKinematics* kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_path, const Pose3d& tip_t_target,
    double control_frequency_hz, const std::optional<JointLimits> joint_limits,
    const bool validate_first_ik_solution_against_hint_joint_configuration) {
  absl::string_view kMotionPlanningErrorSuffix = "FinePathIk";

  INTR_RETURN_IF_ERROR(ValidateKinematics(kinematics));

  const JointLimits applied_joint_limits = joint_limits.value_or(
      kinematics->GetKinematicsModel().GetDofSystemLimits());

  INTR_RETURN_IF_ERROR(ValidateHintJointConfiguration(
      hint_joint_configuration,
      kinematics->GetKinematicsModel().GetNumberDegreesOfFreedom(),
      applied_joint_limits));

  if (control_frequency_hz <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Control frequency must be positive. Got ", control_frequency_hz, "."));
  }

  if (base_t_target_path.empty()) {
    return absl::InvalidArgumentError("Got an empty path.");
  }

  std::vector<eigenmath::VectorNd> path_ik_solutions;
  path_ik_solutions.reserve(base_t_target_path.size());

  // Initialize data for the computation of the hint joint values to the ik
  // solver.
  eigenmath::VectorNd prev_joint_values = hint_joint_configuration;
  eigenmath::VectorNd pprev_joint_values = hint_joint_configuration;
  double half_timestep_squared =
      0.5 / ::intrinsic::IPow(control_frequency_hz, 2);

  // Used to check distance between consecutive base_t_target path waypoints, as
  // one of the actions discussed in go/intrinsic-fine-path-ik-singularities.
  Pose3d prev_base_t_target = base_t_target_path.front();

  // Create a singularity robust IK to be used in the proximity of singular
  // configurations, as discussed in go/intrinsic-fine-path-ik-singularities.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      kinematics::ElementId tip,
      kinematics->GetKinematicsModel().FindNonBranchingKinematicChainTip());
  INTRINSIC_RT_ASSIGN_OR_RETURN(const kinematics::Chain* chain,
                                kinematics->GetKinematicsChain(tip));
  auto singularity_robust_ik_solver =
      std::make_unique<kinematics::KinematicChainRandomSeedIKSolver>(chain);
  singularity_robust_ik_solver->setTimeout(kTimeoutSingularityRobustIkSolver);

  for (const auto& base_t_target : base_t_target_path) {
    const eigenmath::PoseError base_t_target_distance =
        eigenmath::PoseErrorBetween(base_t_target, prev_base_t_target);
    if (base_t_target_distance.translation > kMaxSampleTranslationDistanceM ||
        base_t_target_distance.rotation > kMaxSampleRotationDistanceRad) {
      return absl::InternalError(absl::StrCat(
          "Found base_t_target_path samples to be too distant for FinePathIk, "
          "consider using CoarsePathIk. sample_i: ",
          toString(prev_base_t_target), "\n sample_i+1: ,",
          toString(base_t_target), "\n Distance (translation, rotation): ",
          base_t_target_distance.translation, ",",
          base_t_target_distance.rotation,
          ". Max allowed distance (translation, rotation): ",
          kMaxSampleTranslationDistanceM, ", ", kMaxSampleRotationDistanceRad,
          "."));
    }

    // A hint to the IK solver is built by applying maximum deceleration to the
    // previous joint values. This is helpful to resolve the solution redundancy
    // when a numerical singularity is reached. Without applying maximum
    // deceleration, the solver would return a solution that introduces an
    // instant velocity of 0 which induces infinite acceleration.
    eigenmath::VectorNd max_joint_acceleration =
        applied_joint_limits.max_acceleration;
    eigenmath::VectorNd dp = prev_joint_values - pprev_joint_values;

    // The acceleration is limited such that the robot does not accelerate pass
    // the zero velocity. If the robot can't decelerate fast enough before
    // reaching the joint limit, it could exceed acceleration limits.
    eigenmath::VectorNd max_joint_acc_to_zero_vel =
        dp.cwiseAbs() * ::intrinsic::IPow(control_frequency_hz, 2.0);
    if (!eigenmath::ClampVector(
            eigenmath::VectorNd::Zero(applied_joint_limits.size()),
            max_joint_acc_to_zero_vel, max_joint_acceleration)) {
      return absl::InternalError(absl::StrCat(
          "Could not clamp vector  ", toString(max_joint_acc_to_zero_vel),
          " within zero and [", toString(max_joint_acceleration), "]"));
    }
    max_joint_acceleration = dp.array().sign() * max_joint_acceleration.array();

    // Acceleration is only applied to the joints with non-zero velocity.
    constexpr double kVelocityThreshold = 1e-5;
    eigenmath::VectorNd hint_dp =
        dp - max_joint_acceleration * half_timestep_squared;
    hint_dp = hint_dp.array() *
              (dp.array().abs() > kVelocityThreshold).cast<double>();
    eigenmath::VectorNd ik_hint = prev_joint_values + hint_dp;

    // Check that hint_joint_values are within joint_limits and avoid being
    // exactly at the limit.
    constexpr double kMinDistanceFromLimit = 1e-5;
    if (!eigenmath::ClampVector(
            applied_joint_limits.min_position.array() + kMinDistanceFromLimit,
            applied_joint_limits.max_position.array() - kMinDistanceFromLimit,
            ik_hint)) {
      return absl::InternalError(absl::StrCat(
          "Could not clamp hint joint values to limits, value=",
          toString(ik_hint),
          ", limits=", absl::AlphaNum(ToFixedString(applied_joint_limits))));
    }

    std::optional<eigenmath::VectorNd> closest_hint_joint_configuration =
        std::nullopt;
    if (!path_ik_solutions.empty()) {
      closest_hint_joint_configuration = path_ik_solutions.back();
    }
    INTR_ASSIGN_OR_RETURN(
        const std::vector<eigenmath::VectorNd> ik_solutions,
        ComputeSingularityRobustIK(*kinematics,
                                   /*hint_joint_configuration=*/ik_hint,
                                   base_t_target, tip_t_target,
                                   /*joint_limits=*/applied_joint_limits,
                                   closest_hint_joint_configuration,
                                   singularity_robust_ik_solver.get(),
                                   /*ensure_same_branch=*/false));
    if (ik_solutions.empty()) {
      return absl::InternalError(
          "ComputeSingularityRobustIK() returned no solutions!");
    }

    const absl::Status ik_validation_status = ValidateIkSolutionForFinePathIk(
        path_ik_solutions, hint_joint_configuration,
        /*ik_solution=*/ik_solutions.front(),
        validate_first_ik_solution_against_hint_joint_configuration);
    if (!ik_validation_status.ok()) {
      const Pose3d base_t_tip_desired = base_t_target * tip_t_target.inverse();
      if (ABSL_VLOG_IS_ON(kDebugLogLevel)) {
        INTR_RETURN_IF_ERROR(ComputeAndLogDataForDebugging(
            kinematics, prev_joint_values, ik_hint, ik_solutions.front(),
            base_t_tip_desired, applied_joint_limits));
      }

      return CreateStatusWithFinePathIKError(
          ik_validation_status.message(), prev_joint_values, ik_hint,
          ik_solutions.front(), base_t_tip_desired, applied_joint_limits,
          kMotionPlanningErrorSuffix,
          intrinsic_proto::motion_planning::v1::ErrorContext::
              FINE_PATH_IK_ERROR,
          absl::StatusCode::kInternal);
    }

    path_ik_solutions.push_back(ik_solutions.front());
    pprev_joint_values = prev_joint_values;
    prev_joint_values = ik_solutions.front();
    prev_base_t_target = base_t_target;
  }
  return path_ik_solutions;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> FinePathIk(
    const KinematicsSystemProxy& kinematics_proxy,
    const eigenmath::VectorXd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_path, const Pose3d& tip_t_target,
    double control_frequency_hz,
    const std::optional<JointLimitsXd> joint_limits,
    const bool validate_first_ik_solution_against_hint_joint_configuration) {
  // TODO(b/279222142) rewrite this function body to directly work with
  // KinematicsSystemProxy. For now, a ManipulatorKinematics is extracted from
  // the proxy and the above function is called. This also implies that the
  // size of the hint_joint_configuration should not exceed
  // eigenmath::VectorNd max size. If provided, joint limits additionally need
  // also to be converted from JointLimitsXd to JointLimits (will fail if
  // eigenmath::VectorNd max size is exceeded).
  if (hint_joint_configuration.size() >
      eigenmath::VectorNd::MaxSizeAtCompileTime) {
    return absl::InvalidArgumentError(
        absl::StrCat("Can not handle hint joint configuration of size ",
                     hint_joint_configuration.size(), ". Max size is ",
                     eigenmath::VectorNd::MaxSizeAtCompileTime, "."));
  }

  std::optional<JointLimits> joint_limits_fixed_size = std::nullopt;
  if (joint_limits.has_value()) {
    INTR_ASSIGN_OR_RETURN(joint_limits_fixed_size,
                          ToJointLimits(joint_limits.value()));
  }

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<icon::ManipulatorKinematics> kinematics,
                        kinematics_proxy.GetManipulatorKinematics());

  INTR_ASSIGN_OR_RETURN(
      std::vector<eigenmath::VectorNd> path_ik_solutions,
      FinePathIk(kinematics.get(), hint_joint_configuration, base_t_target_path,
                 tip_t_target, control_frequency_hz, joint_limits_fixed_size,
                 validate_first_ik_solution_against_hint_joint_configuration));

  return std::vector<eigenmath::VectorXd>{path_ik_solutions.begin(),
                                          path_ik_solutions.end()};
}

absl::StatusOr<std::vector<eigenmath::VectorNd>> FinePathIk(
    const icon::ManipulatorKinematics* kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    const CartesianParametricCurve& base_t_target_path,
    const Pose3d& tip_t_target, double sampling_step_translation_m,
    double sampling_step_rotation_rad, double control_frequency_hz,
    const std::optional<JointLimits> joint_limits,
    bool force_odd_number_of_samples,
    const bool validate_first_ik_solution_against_hint_joint_configuration) {
  INTR_ASSIGN_OR_RETURN(
      const std::vector<Pose3d> sampled_base_t_target_path,
      SampleCartesianParametricCurveUniformly(
          base_t_target_path, sampling_step_translation_m,
          sampling_step_rotation_rad, force_odd_number_of_samples));

  return FinePathIk(
      kinematics, hint_joint_configuration, sampled_base_t_target_path,
      tip_t_target, control_frequency_hz, joint_limits,
      validate_first_ik_solution_against_hint_joint_configuration);
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> FinePathIk(
    const KinematicsSystemProxy& kinematics_proxy,
    const eigenmath::VectorXd& hint_joint_configuration,
    const CartesianParametricCurve& base_t_target_path,
    const Pose3d& tip_t_target, double sampling_step_translation_m,
    double sampling_step_rotation_rad, double control_frequency_hz,
    const std::optional<JointLimitsXd> joint_limits,
    bool force_odd_number_of_samples,
    const bool validate_first_ik_solution_against_hint_joint_configuration) {
  INTR_ASSIGN_OR_RETURN(
      const std::vector<Pose3d> sampled_base_t_target_path,
      SampleCartesianParametricCurveUniformly(
          base_t_target_path, sampling_step_translation_m,
          sampling_step_rotation_rad, force_odd_number_of_samples));

  return FinePathIk(
      kinematics_proxy, hint_joint_configuration, sampled_base_t_target_path,
      tip_t_target, control_frequency_hz, joint_limits,
      validate_first_ik_solution_against_hint_joint_configuration);
}

absl::StatusOr<std::vector<eigenmath::VectorNd>> PathIkWithErrorHandling(
    const icon::ManipulatorKinematics* manipulator_kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    absl::Span<const Pose3d> base_t_target_path, const Pose3d& tip_t_target,
    const double path_ik_control_frequency_hz,
    const std::optional<JointLimits> joint_limits,
    const bool validate_first_ik_solution_against_hint_joint_configuration) {
  absl::StatusOr<std::vector<eigenmath::VectorNd>> status_path_ik_solutions =
      FinePathIk(manipulator_kinematics, hint_joint_configuration,
                 base_t_target_path, tip_t_target, path_ik_control_frequency_hz,
                 joint_limits,
                 validate_first_ik_solution_against_hint_joint_configuration);
  if (!status_path_ik_solutions.ok()) {
    return UpdateStatusErrorContext(
        status_path_ik_solutions.status(),
        intrinsic_proto::motion_planning::v1::ErrorContext::
            LINEAR_CARTESIAN_PATH_PLANNER_ERROR,
        absl::StatusCode::kNotFound);
  }
  return std::move(status_path_ik_solutions.value());
}

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
    const std::optional<JointLimits> joint_limits,
    bool force_odd_number_of_samples,
    const bool validate_first_ik_solution_against_hint_joint_configuration) {
  const CartesianCurveLength curve_length = cartesian_curve.GetCurveLength();
  INTR_ASSIGN_OR_RETURN(
      const CartesianCurveMinimalSamplingStepsAndControlFrequency
          cartesian_curve_sampling_steps_and_control_frequency,
      ComputeCartesianCurveMinimalSamplingStepsAndControlFrequency(
          translational_path_sampling_distance_m,
          rotational_path_sampling_distance_rad, curve_length));

  const double kFrequencyHz =
      cartesian_curve_sampling_steps_and_control_frequency
          .path_ik_control_frequency_hz;
  INTR_ASSIGN_OR_RETURN(const double normalized_sampling_step,
                        NormalizedSamplingStepForCartesianCurve(
                            cartesian_curve_sampling_steps_and_control_frequency
                                .sampling_step_translational_m,
                            cartesian_curve_sampling_steps_and_control_frequency
                                .sampling_step_rotational_rad,
                            curve_length));
  INTR_ASSIGN_OR_RETURN(
      const PosesAtNormalizedCurveParameters base_t_target_path,
      cartesian_curve.SampleUniformly(normalized_sampling_step,
                                      force_odd_number_of_samples));
  INTR_ASSIGN_OR_RETURN(
      std::vector<eigenmath::VectorNd> path_ik_solutions,
      PathIkWithErrorHandling(
          manipulator_kinematics, hint_joint_configuration,
          base_t_target_path.poses, tip_t_target, kFrequencyHz, joint_limits,
          validate_first_ik_solution_against_hint_joint_configuration));

  // If `joint_space_uniform_sampling_steps_rad` computed by the
  // `resampling_steps_function` is set, we resample the path in joint space
  // accordingly.
  INTR_ASSIGN_OR_RETURN(const std::optional<std::vector<double>>
                            joint_space_uniform_sampling_steps_rad,
                        resampling_steps_function(path_ik_solutions));
  if (joint_space_uniform_sampling_steps_rad.has_value()) {
    // The `ApproximateJointUniformSampling()` implementation
    // ensures that the returned `curve_parameters_for_uniform_joint_sampling`'s
    // size is odd.
    INTR_ASSIGN_OR_RETURN(
        const std::vector<double> curve_parameters_for_uniform_joint_sampling,
        ApproximateJointUniformSampling(
            base_t_target_path.normalized_curve_parameters, path_ik_solutions,
            *joint_space_uniform_sampling_steps_rad));

    // We increase the density of the curve parameters by a factor of
    // `kUpsamplingFactor` to be able to take larger steps in joint space
    // sampling, while still being able to robustly rely on `FinePathIk`.
    const int kUpsamplingFactor = 10;
    INTR_ASSIGN_OR_RETURN(
        const UpsampledCurveParameters upsampled_curve_parameters,
        CreateUpsampledCurveParameters(
            curve_parameters_for_uniform_joint_sampling, kUpsamplingFactor));
    INTR_ASSIGN_OR_RETURN(
        const std::vector<Pose3d>
            dense_base_t_target_path_for_uniform_joint_sampling,
        cartesian_curve.Sample(
            upsampled_curve_parameters.dense_curve_parameters));
    INTR_ASSIGN_OR_RETURN(
        const std::vector<eigenmath::VectorNd> dense_path_ik_solutions,
        PathIkWithErrorHandling(
            manipulator_kinematics, hint_joint_configuration,
            dense_base_t_target_path_for_uniform_joint_sampling, tip_t_target,
            kFrequencyHz, joint_limits,
            validate_first_ik_solution_against_hint_joint_configuration));
    INTR_ASSIGN_OR_RETURN(
        path_ik_solutions,
        FilterJointConfigurations(
            dense_path_ik_solutions,
            upsampled_curve_parameters
                .sparse_curve_parameters_indices_in_dense_vector));
  }

  if (force_odd_number_of_samples && path_ik_solutions.size() % 2 == 0) {
    return absl::InternalError(
        "The number of Path IK solutions is even. This should not happen.");
  }

  return path_ik_solutions;
}

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
    const std::optional<JointLimitsXd> joint_limits,
    bool force_odd_number_of_samples,
    const bool validate_first_ik_solution_against_hint_joint_configuration) {
  std::optional<JointLimits> joint_limits_fixed_size = std::nullopt;
  if (joint_limits.has_value()) {
    INTR_ASSIGN_OR_RETURN(joint_limits_fixed_size,
                          ToJointLimits(joint_limits.value()));
  }

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<icon::ManipulatorKinematics> kinematics,
                        kinematics_proxy.GetManipulatorKinematics());

  return PathIkOnCartesianCurveAndResampleForUniformJointSampling(
      cartesian_curve, translational_path_sampling_distance_m,
      rotational_path_sampling_distance_rad, kinematics.get(),
      hint_joint_configuration, tip_t_target, resampling_steps_function,
      joint_limits_fixed_size, force_odd_number_of_samples,
      validate_first_ik_solution_against_hint_joint_configuration);
}

}  // namespace intrinsic
