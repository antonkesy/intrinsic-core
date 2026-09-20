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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_IK_SOLVER_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_IK_SOLVER_H_

#include <functional>
#include <vector>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

/** Inverse Kinematics(IK) result
 *
 * list of configuration poses
 */
using IKResult = std::vector<eigenmath::VectorXd>;

/** The signature for an function to be used as an IK validator */
using IKValidator = std::function<bool(const eigenmath::VectorXd&)>;

/** Abstract class for Inverse Kinematics solvers.
 *  All IK solvers need to implement the compute() method
 */
class KinematicChainIKSolver {
 public:
  /** Constructor.
   *  @param kinematic_chain a pointer to the kinematic chain object
   */
  explicit KinematicChainIKSolver(const Chain* kinematic_chain)
      : kinematic_chain_(kinematic_chain) {}

  /** Sets the initial seed.
   *  @param seed_q a vector with joint values to be used as a seed to the IK
   * solver
   */
  virtual void setSeed(const eigenmath::VectorXd& seed_q) = 0;

  /** Compute IK for a pose.
   *  @param base_pose_tip the desired tip link pose wrt the kinematic chain
   * base link
   *  @param validator an IKValidator object
   *  @return IK result as a list of configuration poses, possibly empty
   */
  virtual IKResult compute(const Pose3d& base_pose_tip,
                           const IKValidator& validator) const = 0;

  virtual ~KinematicChainIKSolver() = default;

 protected:
  const Chain* kinematic_chain_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_IK_SOLVER_H_
