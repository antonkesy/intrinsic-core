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

#include "intrinsic/kinematics/ik_solvers/kinematic_chain/damped_least_squares/kinematic_chain_damped_least_squares_ik_solver.h"

#include <cstdint>
#include <random>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/damped_least_squares/ik_damped_least_squares.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/kinematics/joint_wrapping.h"
#include "intrinsic/kinematics/state_generation.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/time/clock_steady.h"
#include "intrinsic/util/time/elapsed_timer.h"

namespace intrinsic {
namespace kinematics {

using eigenmath::VectorXd;

KinematicChainDampedLeastSquaresIKSolver::
    KinematicChainDampedLeastSquaresIKSolver(const Chain* kinematic_chain)
    : KinematicChainIKSolver(kinematic_chain),
      seed_q_(VectorXd::Zero(kinematic_chain->GetNumberDegreesOfFreedom())),
      epsilon_(0.0005),
      timeout_(absl::Seconds(5)),
      max_iterations_(100),
      ik_solver_(
          *kinematic_chain,
          IKDampedLeastSquaresParameters{
              static_cast<int32_t>(max_iterations_), epsilon_, kOrientTolerance,
              kPosClampDist, kOrientClampDist, kDampingCoefficient}) {}

void KinematicChainDampedLeastSquaresIKSolver::setSeed(const VectorXd& seed_q) {
  CHECK_EQ(seed_q.rows(),
           static_cast<int>(kinematic_chain_->GetNumberDegreesOfFreedom()))
      << absl::StrFormat(
             "Wrong number of joints in the input joint vector. Expected %d, "
             "got %zd "
             "joints",
             kinematic_chain_->GetNumberDegreesOfFreedom(), seed_q.rows());
  INTRINSIC_RT_ASSIGN_OR_DIE(
      bool is_within_limits,
      IsWithinLimitsAfterWrapping(*kinematic_chain_, seed_q_,
                                  kinematic_chain_->GetDofSystemLimits()))
  CHECK(is_within_limits) << "Seed must be within joint limits";
  seed_q_ = seed_q;
}

IKResult KinematicChainDampedLeastSquaresIKSolver::compute(
    const Pose3d& base_pose_tip, const IKValidator& validator) const {
  std::default_random_engine engine(0);
  return compute(base_pose_tip, validator, &engine);
}

IKResult KinematicChainDampedLeastSquaresIKSolver::compute(
    const Pose3d& base_pose_tip, const IKValidator& validator,
    std::default_random_engine* engine) const {
  IKResult result;
  VectorXd q = seed_q_;
  bool valid = KinematicChainDampedLeastSquaresIKSolver::IK(
      *kinematic_chain_, base_pose_tip, ik_solver_, timeout_, validator, engine,
      &q);
  if (valid) {
    result.reserve(1);
    result.push_back(q);
  }
  return result;
}

bool KinematicChainDampedLeastSquaresIKSolver::IK(
    const Chain& kinematics, const Pose3d& base_pose_tip,
    const IKDampedLeastSquaresSolver<::Eigen::Dynamic>& dls_ik_solver,
    const absl::Duration timeout, const IKValidator& validator,
    std::default_random_engine* engine, VectorXd* q) {
  CHECK(q != nullptr);

  IKDampedLeastSquaresSolver<::Eigen::Dynamic>::Request request =
      dls_ik_solver.createRequest();
  request.root_pose_link_target = base_pose_tip;

  request.q_start = *q;

  const JointLimits dof_limits = kinematics.GetDofSystemLimits();

  ClockSteady clock_steady;
  ElapsedTimer timer(&clock_steady);
  while (timer.Elapsed() < timeout) {
    dls_ik_solver.solve(&request);

    if (request.status) {
      VectorXd solution = request.q;
      if (!validator || validator(solution)) {
        *q = solution;
        return true;
      }
    }

    // If not successful, try with another random seed.
    request.q_start = GetUniformRandomConfiguration(dof_limits, *engine);
  }
  INTRINSIC_RT_LOG(WARNING) << "IK Timed out";

  return false;
}

}  // namespace kinematics
}  // namespace intrinsic
