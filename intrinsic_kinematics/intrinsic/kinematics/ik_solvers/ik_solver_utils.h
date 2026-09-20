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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_IK_SOLVER_UTILS_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_IK_SOLVER_UTILS_H_

#include <cmath>
#include <cstddef>
#include <random>

#include "absl/log/check.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/util/macros.h"

namespace intrinsic {
namespace kinematics {

// Null space task function.
class NullSpaceTask {
 public:
  // Construct the NullSpaceTask.
  explicit NullSpaceTask(const JointLimits& dof_limits,
                         double soft_limit_fraction = 0.1)
      : soft_limit_fraction_([&]() {
          CHECK((soft_limit_fraction >= 0) && (soft_limit_fraction < 0.5));
          return soft_limit_fraction;
        }()),
        joint_ranges_([&]() {
          CHECK_GE(
              (dof_limits.max_position - dof_limits.min_position).minCoeff(),
              0);
          return dof_limits.max_position - dof_limits.min_position;
        }()) {
    soft_lower_limits_ =
        dof_limits.min_position + soft_limit_fraction * joint_ranges_;
    soft_upper_limits_ =
        dof_limits.max_position - soft_limit_fraction * joint_ranges_;
  }

  // Returns the desired null space velocity, given the current robot
  // configuration. Allows the use of a NullSpaceTask object directly as a
  // kinematics::NullSpaceTaskFunction.
  eigenmath::VectorNd operator()(const eigenmath::VectorNd& joint_angles) {
    constexpr double kStayCloseToZeroGain = 0.1;
    constexpr double kStayAwayFromLimitsGain = 10.0;

    // Stay close to zero
    eigenmath::VectorNd null_space_velocity =
        -kStayCloseToZeroGain * joint_angles;
    // Stay away from joint limits
    for (eigenmath::VectorNd::Index i = 0; i < joint_angles.size(); ++i) {
      if (joint_angles[i] > soft_upper_limits_[i]) {
        null_space_velocity[i] += kStayAwayFromLimitsGain *
                                  (soft_upper_limits_[i] - joint_angles[i]) /
                                  joint_ranges_[i];
      }
      if (joint_angles[i] < soft_lower_limits_[i]) {
        null_space_velocity[i] += kStayAwayFromLimitsGain *
                                  (soft_lower_limits_[i] - joint_angles[i]) /
                                  joint_ranges_[i];
      }
    }
    return null_space_velocity;
  }

 private:
  double soft_limit_fraction_;
  eigenmath::VectorNd soft_lower_limits_;
  eigenmath::VectorNd soft_upper_limits_;
  eigenmath::VectorNd joint_ranges_;
};

std::seed_seq GetSeedSeqFromValues(const eigenmath::VectorXd& values);

// Computes the manipulability index.
double ComputeManipulability(const eigenmath::Matrix6Nd& jacobian);

// Computes the (minimum) joint limit distance.
// If is_using_softmin == false, then compute the (regular) minimum value.
// If is_using_softmin == true, then compute the smooth/soft-minimum
// with the selected softmin_alpha value < 0
// (in case we want to compute a derivative of it;
// the closer softmin_alpha to zero, the smoother it is, and
// as softmin_alpha goes to -infinity, the closer it approximates
// the regular minimum function).
// If the computed distance is infinity --which means that there is no joint
// limit--, the function will return `default_return_value` instead.
// A few notes on the selection of the `default_return_value`:
// The default value of `default_return_value` is 1.0 which is a good selection
// when the result of this function is to be multiplied with another term (e.g.
// the result of ComputeManipulability()) in an additive optimization cost term.
// When used as a stand-alone additive optimization cost term, it makes sense to
// set `default_return_value` to 0.0 or any constant value that will not affect
// the optimization. Finally, `default_return_value` can also be set to a large
// positive value which specifies the maximum possible value that can be
// returned by this function.
icon::RealtimeStatusOr<double> ComputeJointLimitDistance(
    const JointLimits& limits, const eigenmath::VectorNd& joint_angles,
    bool is_using_softmin = false, double softmin_alpha = -10.0,
    double default_return_value = 1.0);

// Computes the joint limit distance-modulated manipulability index.
// Please see the definition of is_using_softmin and softmin_alpha
// in ComputeJointLimitDistance() function.
icon::RealtimeStatusOr<double> ComputeJointLimDistModManipulability(
    const eigenmath::Matrix6Nd& jacobian, const JointLimits& limits,
    const eigenmath::VectorNd& joint_angles, bool is_using_softmin = false,
    double softmin_alpha = -10.0);

// Get a random joint configuration from a uniform distribution within the
// joint limits defined by the given kinematic chain. The Generator is expected
// to be a absl::BitGen.
template <typename Generator>
eigenmath::VectorXd GetUniformRandomConfiguration(const Chain& kinematic_chain,
                                                  Generator& gen) {
  const JointLimits dof_limits = kinematic_chain.GetDofSystemLimits();
  eigenmath::VectorXd upper_limits = dof_limits.max_position;
  eigenmath::VectorXd lower_limits = dof_limits.min_position;
  // Replace -inf/inf with concrete values.
  for (int i = 0; i < kinematic_chain.GetNumberDegreesOfFreedom(); i++) {
    if (std::isinf(lower_limits[i])) {
      lower_limits(i) = -M_PI;
    }
    if (std::isinf(upper_limits[i])) {
      upper_limits(i) = M_PI;
    }
  }

  ASSIGN_OR_DIE(
      eigenmath::VectorXd random_q,
      eigenmath::GetUniformRandomVectorXd(lower_limits, upper_limits, gen));
  return random_q;
}

// Get a random joint configuration from a uniform distribution within the
// joint limits and within a given range from a given configuration. The
// Generator is expected to be a absl::BitGen.
template <typename Generator>
eigenmath::VectorXd SampleRandomDeltaJointConfiguration(
    const eigenmath::VectorXd& current, double max_delta,
    const Chain& kinematic_chain, Generator& gen) {
  auto dof_limits = kinematic_chain.GetDofSystemLimits();
  eigenmath::VectorXd upper_limits = dof_limits.max_position;
  eigenmath::VectorXd lower_limits = dof_limits.min_position;

  // Replace -inf/inf with concrete values.
  for (int i = 0; i < kinematic_chain.GetNumberDegreesOfFreedom(); i++) {
    if (std::isinf(lower_limits[i])) {
      lower_limits(i) = -M_PI;
    }
    if (std::isinf(upper_limits[i])) {
      upper_limits(i) = M_PI;
    }
  }

  for (std::size_t i = 0; i < kinematic_chain.GetNumberDegreesOfFreedom();
       ++i) {
    double lower_bound = current[i] - max_delta / 2;
    if (lower_bound > upper_limits[i]) {
      upper_limits[i] = lower_bound;
    }

    double upper_bound = current[i] + max_delta / 2;
    if (upper_bound < upper_limits[i]) {
      upper_limits[i] = upper_bound;
    }
  }
  ASSIGN_OR_DIE(
      eigenmath::VectorXd random_q,
      eigenmath::GetUniformRandomVectorXd(lower_limits, upper_limits, gen));
  return random_q;
}

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_IK_SOLVER_UTILS_H_
