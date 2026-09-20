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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_NEWTON_RAPHSON_IK_SOLVER_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_NEWTON_RAPHSON_IK_SOLVER_H_

#include <cstddef>
#include <random>

#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

/** A Newton-Raphson implementation for solving IK */
class KinematicChainNewtonRaphsonIKSolver : public KinematicChainIKSolver {
 public:
  /** Constructor.
   *  @param kinematic_chain a pointer to the kinematic chain object
   */
  explicit KinematicChainNewtonRaphsonIKSolver(const Chain* kinematic_chain);

  void setSeed(const eigenmath::VectorXd& seed_q) override;

  /** Computes IK for a pose.
   *  @param base_pose_tip the desired tip link pose wrt the kinematic chain
   * base link
   *  @param validator an IKValidator object
   *  @return IK result, a list of a configuration poses.
   *           Contains single element on success, empty if it failed.
   */
  IKResult compute(const Pose3d& base_pose_tip,
                   const IKValidator& validator) const override;

  ~KinematicChainNewtonRaphsonIKSolver() override = default;

  /** Static IK call.
   *  Looks for a solution to the IK problem given a seed joint configuration, a
   * maximum number of iterations and a desired tolerance. Thread-safe.
   *  @param kinematics a kinematic chain object
   *  @param base_pose_tip the desired end-effector pose wrt the chain base link
   *  @param max_iterations the maximum number of iterations to perform
   *  @param epsilon the maximum error allowed (euclidean norm of the pose error
   * vector)
   *  @param validator an IKValidator, empty if no validation required
   *  @param[in,out] seed_q as input, a initial seed to use. As output, the IK
   * result
   *  @return true if a solution has been found, false if could not find a
   * solution
   */
  static bool IK(const Chain& kinematics, const Pose3d& base_pose_tip,
                 const ElementId& tip_id, std::size_t max_iterations,
                 double epsilon, const IKValidator& validator,
                 eigenmath::VectorXd* seed_q);

  // Same as above, but using fixed maximum size vectors and absl::FunctionRef
  // to ensure no memory allocations.
  // Note that this makes `validator` a required parameter!
  static bool IK(const Chain& kinematics, const Pose3d& base_pose_tip,
                 const ElementId& tip_id, std::size_t max_iterations,
                 double epsilon,
                 absl::FunctionRef<bool(const eigenmath::VectorNd&)> validator,
                 eigenmath::VectorNd& seed_q);

 protected:
  eigenmath::VectorXd seed_q_;
  double epsilon_;
  std::size_t max_iterations_;
  ElementId tip_id_ = kInvalidElementId;
};

/** A specialized KinematicChainNewtonRaphsonIKSolver with random seeding.
 *  This solver will generate random seeds and call the
 * KinematicChainNewtonRaphsonIKSolver until a solution is found or a timeout or
 * max. number of attempts. Seeds are generated in a deterministic way,
 * therefore IK solutions will be provided in a deterministic order as well. A
 * validation function can be provided by the user to discard those solutions
 * that doesn't meet certain criteria.
 */
class KinematicChainRandomSeedIKSolver
    : public KinematicChainNewtonRaphsonIKSolver {
 public:
  /** Constructor.
   *  @param kinematic_chain a pointer to the kinematic chain object
   */
  explicit KinematicChainRandomSeedIKSolver(const Chain* kinematic_chain)
      : KinematicChainNewtonRaphsonIKSolver(kinematic_chain),
        timeout_(absl::Seconds(5)),
        max_num_ik_attempts_(0) {}

  /** Sets a timeout.
   *  The compute call will return with failure if the timeout is reached.
   *  @param d the timeout
   */
  void setTimeout(absl::Duration d) { timeout_ = d; }

  /** Computes IK for a pose.
   *  @param base_pose_tip the desired tip link pose wrt the kinematic chain
   * base link
   *  @param validator an IKValidator object
   *  @return IK result as a list of configuration poses, possibly empty
   */
  IKResult compute(const Pose3d& base_pose_tip,
                   const IKValidator& validator) const override;

  /** Computes IK for a pose.
   *  @param base_pose_tip the desired tip link pose wrt the kinematic chain
   * base link
   *  @param validator an IKValidator object
   *  @param engine used to generate random numbers
   *  @return IK result as a list of configuration poses, possibly empty
   */
  IKResult compute(const Pose3d& base_pose_tip, const IKValidator& validator,
                   std::default_random_engine* engine) const;

  /** Static IK call.
   *  Will compute IK using an initial seed if provided. If no seed provided or
   * IK not found from the initial seed, it will pick random seeds until a
   * solution is found or a timeout is reached. Thread-safe.
   *  @param kinematics a kinematic chain object
   *  @param base_pose_tip the desired end-effector pose wrt the chain base link
   *  @param timeout the maximum time to spend looking for a solution
   *  @param max_iterations the maxixum number of iterations to perform per IK
   * call
   *  @param epsilon the maximum error allowed (euclidean norm of the pose error
   * vector)
   *  @param validator an IKValidator, empty if no validation required
   *  @param max_num_ik_attempts the maximum number of IK attempts
   *  @param engine used to generate random numbers
   *  @param[in,out] q as input, an initial seed to use (if valid). As output,
   * the IK result
   *  @return true if a solution has been found, false if could not find a
   * solution
   */
  static bool IK(const Chain& kinematics, const Pose3d& base_pose_tip,
                 const ElementId& tip_id, absl::Duration timeout,
                 std::size_t max_iterations, double epsilon,
                 const IKValidator& validator, std::size_t max_num_ik_attempts,
                 std::default_random_engine* engine, eigenmath::VectorXd* q);

  ~KinematicChainRandomSeedIKSolver() override = default;

 protected:
  absl::Duration timeout_;
  std::size_t max_num_ik_attempts_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_NEWTON_RAPHSON_IK_SOLVER_H_
