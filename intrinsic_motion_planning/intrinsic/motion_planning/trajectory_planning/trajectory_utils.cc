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

#include "intrinsic/motion_planning/trajectory_planning/trajectory_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/check_cartesian_limits.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/dynamic_limits_check_mode.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/path_sample.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

absl::StatusOr<ZeroCrossingCount> CountTimeSeriesZeroCrossings(
    absl::Span<const eigenmath::VectorNd> time_series,
    double boundary_zero_threshold) {
  if (time_series.empty()) {
    return absl::FailedPreconditionError(
        "Cannot count zero crossings for empty `time_series`.");
  }
  if (time_series.front().size() == 0) {
    return absl::FailedPreconditionError(
        "The `time_series` vector cannot have elements of size 0.");
  }
  for (int i = 1; i < time_series.size(); ++i) {
    if (time_series.front().size() != time_series[i].size()) {
      return absl::FailedPreconditionError(
          absl::StrCat("The elements of `time_series` must have the same size. "
                       "The expected size is ",
                       time_series.front().size(), " but the size of element ",
                       i, " is ", time_series[i].size(), "."));
    }
  }
  if (boundary_zero_threshold < 0.0) {
    return absl::FailedPreconditionError(
        "The boundary_zero_threshold must be >= 0.0.");
  }

  const int ndof = time_series.front().size();
  ZeroCrossingCount result;
  result.num_zeros_inner = Eigen::VectorXi::Zero(ndof);
  result.num_zeros_including_boundaries = Eigen::VectorXi::Zero(ndof);

  std::vector<std::optional<double>> last_nonzero_value(ndof, std::nullopt);

  for (int i = 0; i < ndof; i++) {
    if (time_series.front()(i) != 0.0)
      last_nonzero_value[i] = time_series.front()(i);
  }

  for (const auto& state : time_series) {
    const auto& current_velocity = state;
    for (int j = 0; j < ndof; j++) {
      if (current_velocity(j) == 0.0) continue;

      if (last_nonzero_value[j].has_value()) {
        // Signs must have changed in case the product between the last non-zero
        // value and the current value is negative.
        if (current_velocity(j) * last_nonzero_value.at(j).value() < 0.0) {
          result.num_zeros_inner(j)++;
        }
      }
      last_nonzero_value.at(j) = current_velocity(j);
    }
  }

  // Examine the boundaries of the trajectory to compute
  // 'num_zeros_including_boundaries'. The boundary values are interpreted as
  // separate zeros if their magnitude is below 'zero_threshold'.
  result.num_zeros_including_boundaries = result.num_zeros_inner;
  for (int i = 0; i < ndof; i++) {
    if (std::abs(time_series.front()(i)) < boundary_zero_threshold) {
      result.num_zeros_including_boundaries(i)++;
    }
    // Zeros at the end of the interval are only counted as zeros if the
    // trajectory has at least 2 entries.
    if (std::abs(time_series.back()(i)) < boundary_zero_threshold &&
        time_series.size() > 1) {
      result.num_zeros_including_boundaries(i)++;
    }
  }
  return result;
}

absl::StatusOr<JointTrajectoryPVA> TimeScale(
    const JointTrajectoryPVA& trajectory, const double time_scaling_factor) {
  if (time_scaling_factor <= 0)
    return absl::InvalidArgumentError("time_scaling_factor must be > 0.");

  std::vector<absl::Duration> scaled_time_stamps{
      trajectory.time_stamps().begin(), trajectory.time_stamps().end()};
  // Time stamps get scaled linearly.
  std::transform(scaled_time_stamps.begin(), scaled_time_stamps.end(),
                 scaled_time_stamps.begin(),
                 [time_scaling_factor](absl::Duration& c) {
                   return c * time_scaling_factor;
                 });

  std::vector<JointStatePVA> scaled_states{trajectory.data().begin(),
                                           trajectory.data().end()};
  for (auto& scaled_state : scaled_states) {
    // Velocities get scaled linearly.
    scaled_state.velocity /= time_scaling_factor;
    // Accelerations get scaled quadratically.
    scaled_state.acceleration /= ::intrinsic::IPow(time_scaling_factor, 2);
  }

  return JointTrajectoryPVA::Create(
      std::move(scaled_states), std::move(scaled_time_stamps),
      trajectory.joint_dynamic_limits_check_mode(),
      trajectory.interpolation_type());
}

// Returns a time-scaled `trajectory` which adheres to velocity and acceleration
// `limits`. Returns original trajectory if no limit violation detected.
absl::StatusOr<JointTrajectoryPVA> DownscaleIfViolatingLimits(
    const JointTrajectoryPVA& trajectory, const JointLimits& limits) {
  double time_scaling_factor = 1.0;
  for (const auto& point : trajectory.data()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto limit_check_result,
                                  IsWithinLimits(point, limits));
    if (!limit_check_result.a_ok) {
      const eigenmath::VectorNd acc_limit_overshoot_ratio =
          point.acceleration.cwiseAbs().array() /
          limits.max_acceleration.array();
      // Time scaling factor is direct proportional to the square-root of the
      // acceleration overshoot.
      time_scaling_factor = std::max(
          time_scaling_factor,
          std::sqrt(*std::max_element(acc_limit_overshoot_ratio.begin(),
                                      acc_limit_overshoot_ratio.end())));
    }
    if (!limit_check_result.v_ok) {
      const eigenmath::VectorNd vel_limit_overshoot_ratio =
          point.velocity.cwiseAbs().array() / limits.max_velocity.array();
      // Time scaling factor is direct proportional to the velocity overshoot.
      time_scaling_factor =
          std::max(time_scaling_factor,
                   *std::max_element(vel_limit_overshoot_ratio.begin(),
                                     vel_limit_overshoot_ratio.end()));
    }
  }
  DVLOG(1) << "Computed time scaling factor " << time_scaling_factor
           << " trajectory limit violation.";

  return TimeScale(trajectory, time_scaling_factor);
}

absl::Status IsTrajectoryWithinCartesianPositionLimits(
    const kinematics::Chain& chain, const CartesianLimits& cartesian_limits,
    const JointTrajectoryPVA& trajectory) {
  kinematics::State state(&chain);
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const JointStatePVA& joint_state = trajectory.data().at(i);

    INTRINSIC_RT_RETURN_IF_ERROR(state.SetDofPositions(joint_state.position));
    INTRINSIC_RT_ASSIGN_OR_RETURN(Pose3d tip_pose,
                                  state.GetTransform(chain.GetTipId()));

    if (!IsWithinLimits(tip_pose, cartesian_limits)) {
      return absl::InvalidArgumentError(
          absl::StrCat("Provided joint trajectory violates Cartesian position ",
                       "limits at index ", i));
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<JointTrajectoryPVA> ZeroMotionTrajectory(
    const eigenmath::VectorNd& zero_motion_joint_configuration) {
  JointStatePVA joint_state;
  INTRINSIC_RT_RETURN_IF_ERROR(
      joint_state.SetSize(zero_motion_joint_configuration.size()));
  joint_state.position = zero_motion_joint_configuration;
  joint_state.velocity.setZero();
  joint_state.acceleration.setZero();
  return JointTrajectoryPVA::Create(
      {joint_state, joint_state}, {absl::ZeroDuration(), absl::Milliseconds(1)},
      DynamicLimitsCheckMode::kCheckJointAcceleration,
      JointTrajectoryInterpolationType::kCubicPolynomial);
}

absl::StatusOr<JointTrajectoryPVA> ZeroMotionTrajectory(
    const eigenmath::VectorXd& zero_motion_joint_configuration) {
  if (zero_motion_joint_configuration.size() >
      eigenmath::VectorNd::MaxSizeAtCompileTime) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Can not construct joint trajectory, because max "
        "size of current configuration exceeds maximum allowed "
        "size. Got configuration of size ",
        zero_motion_joint_configuration.size(), ", but max size is ",
        eigenmath::VectorNd::MaxSizeAtCompileTime, "."));
  }
  const eigenmath::VectorNd current_joint_config_nd =
      zero_motion_joint_configuration;
  return ZeroMotionTrajectory(current_joint_config_nd);
}

absl::StatusOr<std::vector<topp::PathSample>> ZeroMotionPath(
    const eigenmath::VectorNd& zero_motion_joint_configuration) {
  topp::PathSample path_sample;
  INTR_RETURN_IF_ERROR(
      path_sample.SetSize(zero_motion_joint_configuration.size()));
  path_sample.q = zero_motion_joint_configuration;
  path_sample.qp = eigenmath::VectorXd::Zero(path_sample.q.size());
  path_sample.qpp = eigenmath::VectorXd::Zero(path_sample.q.size());
  path_sample.qppp = eigenmath::VectorXd::Zero(path_sample.q.size());

  std::vector<topp::PathSample> zero_motion_path;
  path_sample.s = 0.0;
  zero_motion_path.push_back(path_sample);
  // This is only valid because push_back() creates a copy of `path_sample`.
  path_sample.s = 1.0;
  zero_motion_path.push_back(path_sample);
  return zero_motion_path;
}

}  // namespace intrinsic
