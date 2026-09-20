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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_SOLVER_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_SOLVER_H_

#include <cstddef>
#include <random>

#include "absl/log/check.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/damped_least_squares/ik_damped_least_squares.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/units.h"

namespace intrinsic {
namespace kinematics {

/** A IK Solver using the IKDampedLeastSquaresSolver */
class KinematicChainDampedLeastSquaresIKSolver : public KinematicChainIKSolver {
 public:
  /** Constructor.
   *  @param kinematic_chain a pointer to a kinematic chain object
   */
  explicit KinematicChainDampedLeastSquaresIKSolver(
      const Chain* kinematic_chain);

  /** Sets a timeout.
   *  The compute call will return with failure if the timeout is reached
   *  @param d the timeout (in seconds)
   */
  void setTimeout(const absl::Duration d) { timeout_ = d; }

  void setSeed(const eigenmath::VectorXd& seed_q) override;

  /** Computes IK for a pose.
   *  @param base_pose_tip the desired tip link pose wrt the kinematic chain
   * base link
   *  @param validator an IKValidator object
   *  @returns IK result, a list of a configuration poses.
   *           Contains single element on success, empty if it failed.
   */
  IKResult compute(const Pose3d& base_pose_tip,
                   const IKValidator& validator) const override;

  IKResult compute(const Pose3d& base_pose_tip, const IKValidator& validator,
                   std::default_random_engine* engine) const;

  /** Compute IK.
   *  Looks for a solution to the IK problem given a maximum number of
   * iterations, a timeout and a desired tolerance. Picks seeds randomly until a
   * solution is found or the exit conditions are met. Thread-safe, uses an FK
   * solver internally.
   *  @param kinematics a kinematic chain object
   *  @param base_pose_tip the desired end-effector pose wrt the chain base link
   *  @param dls_ik_solver a pointer to an IKDampedLeastSquaresSolver
   *  @param timeout the maximum time to spend looking for a solution
   *  @param validator an IKValidator, empty if no validation required
   *  @param[in,out] q as input, a initial seed to use. As output, the IK result
   *  @returns true if a solution has been found, false if could not find a
   * solution
   */
  static bool IK(
      const Chain& kinematics, const Pose3d& base_pose_tip,
      const IKDampedLeastSquaresSolver<::Eigen::Dynamic>& dls_ik_solver,
      absl::Duration timeout, const IKValidator& validator,
      std::default_random_engine* engine, eigenmath::VectorXd* q);

  ~KinematicChainDampedLeastSquaresIKSolver() override = default;

 protected:
  eigenmath::VectorXd seed_q_;
  double epsilon_;
  absl::Duration timeout_;
  std::size_t max_iterations_;

  static constexpr double kDampingCoefficient = 0.06;
  static constexpr double kOrientTolerance = 0.01;
  static constexpr double kPosClampDist = 0.35;
  static constexpr double kOrientClampDist = DegToRad(20.0);

  IKDampedLeastSquaresSolver<::Eigen::Dynamic> ik_solver_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_SOLVER_H_
