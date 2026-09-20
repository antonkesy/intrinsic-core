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

#include "intrinsic/motion_planning/path_planning/path_planning_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

using eigenmath::VectorXd;

namespace {

double GetSmallestSingularValueFromJacobian(
    const eigenmath::Matrix6Nd& jacobian) {
  Eigen::JacobiSVD<eigenmath::Matrix6Nd> svd(jacobian);
  return svd.singularValues().minCoeff();
}

}  // namespace

absl::StatusOr<double> ComputeViolationRatio(
    const PointPath& path, const PointValidator& point_validator,
    const EdgeValidator& edge_validator) {
  if (path.empty()) return 0.;  // no violation for an empty path.
  int invalid_point_count = 0;
  int invalid_edge_count = 0;
  for (int i = 0; i < path.size(); i++) {
    INTR_ASSIGN_OR_RETURN(bool valid_pi, point_validator(path[i]));
    if (!valid_pi) invalid_point_count++;
    if (i < path.size() - 1) {
      INTR_ASSIGN_OR_RETURN(bool valid_ei,
                            edge_validator(path[i], path[i + 1]));
      if (!valid_ei) invalid_edge_count++;
    }
  }
  return static_cast<double>(invalid_point_count + invalid_edge_count) /
         (path.size() + path.size() - 1.);
}

absl::StatusOr<double> ComputeViolationLengthRatio(
    const PointPath& path, const EdgeValidator& edge_validator) {
  if (path.size() < 2) return 0.;  // no violation for a path of 0 or 1 points.
  double invalid_length = 0.;
  double total_length = 0.;
  for (int i = 0; i < path.size() - 1; i++) {
    double d = (path[i + 1] - path[i]).norm();
    total_length += d;
    INTR_ASSIGN_OR_RETURN(bool valid_ei, edge_validator(path[i], path[i + 1]));
    if (!valid_ei) invalid_length += d;
  }

  // Pre-caution for 0-length path.
  if (total_length > 0.) return invalid_length / total_length;

  return 0.;
}

absl::StatusOr<JointLimitsXd> GetSamplingLimitsForPlanningProblem(
    const PointPath& path, const JointLimitsXd& limits) {
  JointLimitsXd planning_limits = limits;

  // Find the max and min values of the path for each joint
  VectorXd min_path_configs = VectorXd::Constant(
      planning_limits.size(), std::numeric_limits<double>::max());
  VectorXd max_path_configs = VectorXd::Constant(
      planning_limits.size(), std::numeric_limits<double>::lowest());

  for (const VectorXd& config : path) {
    if (config.size() != planning_limits.size()) {
      return absl::InvalidArgumentError(
          "Provided path contains configurations of size different from the "
          "joint limits.");
    }
    for (int i = 0; i < min_path_configs.size(); ++i) {
      min_path_configs[i] = std::min(min_path_configs[i], config[i]);
      max_path_configs[i] = std::max(max_path_configs[i], config[i]);
    }
  }

  // Adjust limits for infinite joints to restrict the search space around the
  // provide path.
  for (int i = 0; i < planning_limits.size(); ++i) {
    // TODO(kmuelling): The value of 3*pi was chosen without much data to
    // base its selection. With more data, we should find the right value here
    // that keeps the path planning fast and the ability to effectively use the
    // infinite joint to wind multiple times around to avoid obstacles.
    // TODO(kmuelling): We should pick more reasonable definition of too high
    // instead of relying on float min/max as a guide. Maybe always clamping to
    // `min_path_configs[i] - 3 * M_PI` or 6 * M_PI.
    if (std::isinf(limits.min_position[i]) ||
        limits.min_position[i] <= std::numeric_limits<float>::lowest()) {
      planning_limits.min_position[i] = min_path_configs[i] - 3 * M_PI;
    }
    if (std::isinf(limits.max_position[i]) ||
        limits.max_position[i] >= std::numeric_limits<float>::max()) {
      planning_limits.max_position[i] = max_path_configs[i] + 3 * M_PI;
    }
  }

  return planning_limits;
}

icon::RealtimeStatusOr<double> GetJacobianSmallestSingularValue(
    const icon::ManipulatorKinematics* manipulator_kinematics,
    const eigenmath::VectorNd& joint_configuration) {
  if (manipulator_kinematics == nullptr) {
    return icon::InvalidArgumentError(
        "The manipulator kinematics cannot be a nullptr.");
  }
  const size_t ndofs =
      manipulator_kinematics->GetKinematicsModel().GetNumberDegreesOfFreedom();
  if (joint_configuration.size() != ndofs) {
    return icon::InvalidArgumentError(
        "Joint configuration size should match the number of dofs of the "
        "manipulator kinematics.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const eigenmath::Matrix6Nd jacobian,
      manipulator_kinematics->ComputeChainJacobian(joint_configuration));
  return GetSmallestSingularValueFromJacobian(jacobian);
}

icon::RealtimeStatusOr<double> GetJacobianSmallestSingularValue(
    const kinematics::Chain& chain,
    const eigenmath::VectorNd& joint_configuration) {
  if (joint_configuration.size() != chain.GetNumberDegreesOfFreedom()) {
    return icon::InvalidArgumentError(
        "Joint_configuration size should match the number of dofs of the "
        "kinematics chain.");
  }
  kinematics::State state(&chain);
  INTRINSIC_RT_RETURN_IF_ERROR(
      state.SetDofPositions(joint_configuration, /*check_limits=*/false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const eigenmath::Matrix6Nd jacobian,
                                state.ComputeJacobian(chain.GetTipId()));
  return GetSmallestSingularValueFromJacobian(jacobian);
}
double ComputePointPathLength(const PointPath& path) {
  double path_length = 0.0;
  for (int ii = 0; ii + 1 < path.size(); ++ii) {
    path_length += (path[ii + 1] - path[ii]).norm();
  }
  return path_length;
}

}  // namespace intrinsic
