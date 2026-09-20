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

#include "intrinsic/kinematics/ik_solvers/ik_solver_utils.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"

namespace intrinsic {
namespace kinematics {

namespace {

using Vector2Nd =
    eigenmath::VectorNdWithMaxSize<2 * eigenmath::MAX_EIGEN_VECTOR_SIZE>;

std::vector<int> FloorToVectorInt(const eigenmath::VectorXd& values,
                                  double scale) {
  std::vector<int> result(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    result[i] = floor(values[i] * scale);
  }
  return result;
}

}  // namespace

std::seed_seq GetSeedSeqFromValues(const eigenmath::VectorXd& values) {
  std::vector<int> seed_array = FloorToVectorInt(values, 1000);
  return std::seed_seq(seed_array.begin(), seed_array.end());
}

double ComputeManipulability(const eigenmath::Matrix6Nd& jacobian) {
  eigenmath::Matrix6d jjt = jacobian * jacobian.transpose();
  double determinant_jjt = jjt.determinant();
  // In theory determinant_jjt >= 0, but in some cases (e.g. at some joint
  // limits) determinant_jjt can be a very small negative number, leading to
  // sqrt(determinant_jjt) to be NaN (numerical issues), so we zero out those
  // cases.
  if (determinant_jjt < 0) {
    determinant_jjt = 0.0;
  }
  return std::sqrt(determinant_jjt);
}

icon::RealtimeStatusOr<double> ComputeJointLimitDistance(
    const JointLimits& limits, const eigenmath::VectorNd& joint_angles,
    bool is_using_softmin, double softmin_alpha, double default_return_value) {
  if (softmin_alpha >= 0) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "softmin_alpha must be < 0, but instead get: ", softmin_alpha));
  }
  for (int j = 0; j < limits.size(); j++) {
    if (limits.max_position(j) == -std::numeric_limits<double>::infinity()) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "limits.max_position(", j, ") cannot be -infinity!"));
    }
    if (limits.min_position(j) == std::numeric_limits<double>::infinity()) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "limits.min_position(", j, ") cannot be infinity!"));
    }
  }
  // distances to joint limits:
  eigenmath::MatrixNd distance_matrix(2, joint_angles.rows());
  distance_matrix.row(0) =
      (limits.max_position - joint_angles).cwiseAbs().transpose();
  distance_matrix.row(1) =
      (joint_angles - limits.min_position).cwiseAbs().transpose();

  if (!is_using_softmin) {
    return distance_matrix.minCoeff();
  }

  Vector2Nd all_distance_vector = Vector2Nd::Zero(2 * joint_angles.rows());
  int num_relevant_values = 0;
  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < joint_angles.rows(); col++) {
      // Only include distances which are not infinity.
      if (!std::isinf(distance_matrix(row, col))) {
        all_distance_vector(num_relevant_values) = distance_matrix(row, col);
        num_relevant_values++;
      }
    }
  }

  if (num_relevant_values == 0) {
    return default_return_value;
  }

  Vector2Nd relevant_distance_vector =
      all_distance_vector.topRows(num_relevant_values);
  eigenmath::MatrixNd exp_alpha_d =
      (softmin_alpha * relevant_distance_vector).array().exp();
  double min_joint_limit_distance =
      ((relevant_distance_vector.cwiseProduct(exp_alpha_d).sum()) /
       (exp_alpha_d.sum()));

  return min_joint_limit_distance;
}

icon::RealtimeStatusOr<double> ComputeJointLimDistModManipulability(
    const eigenmath::Matrix6Nd& jacobian, const JointLimits& limits,
    const eigenmath::VectorNd& joint_angles, bool is_using_softmin,
    double softmin_alpha) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto joint_limit_distance,
      ComputeJointLimitDistance(limits, joint_angles, is_using_softmin,
                                softmin_alpha));
  return (joint_limit_distance * ComputeManipulability(jacobian));
}

}  // namespace kinematics
}  // namespace intrinsic
