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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_QP_IK_SOLVER_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_QP_IK_SOLVER_H_

#include <cstddef>
#include <tuple>

#include "absl/log/check.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

/** An implementation for solving IK using Quadratic Programming (QP)
 */
class KinematicChainQPIKSolver : public KinematicChainIKSolver {
 public:
  explicit KinematicChainQPIKSolver(const Chain* kinematic_chain);

  ~KinematicChainQPIKSolver() override = default;

  void setSeed(const eigenmath::VectorXd& seed_q) override;

  void setTimeout(const absl::Duration d) { timeout_ = d; }

  IKResult compute(const Pose3d& base_pose_tip,
                   const IKValidator& validator) const override;

 protected:
  std::tuple<eigenmath::MatrixXd, eigenmath::VectorXd, double>
  GetObjectiveMatrixAndVector(const Pose3d& base_pose_tip_desired,
                              const eigenmath::VectorXd& q, double lm_damping,
                              double dt) const;

  bool IK(const Chain& kinematics, const Pose3d& base_pose_tip,
          const IKValidator& validator, eigenmath::VectorXd* seed_q) const;

 protected:
  eigenmath::VectorXd seed_q_;
  double epsilon_;
  std::size_t max_iterations_;
  absl::Duration timeout_;
  JointLimits limits_;
  ElementId tip_id_ = kInvalidElementId;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_KINEMATIC_CHAIN_QP_IK_SOLVER_H_
