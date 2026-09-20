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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_UR_IK_SOLVER_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_UR_IK_SOLVER_H_

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

class URIKSolver {
 public:
  // The maximum number of solutions that can be found.
  static constexpr int kSolutionBufferSize = 8;

  // The degrees-of-freedom of the robot arm.
  static constexpr size_t kNbDof = 6;

  URIKSolver() = default;

  absl::Status Init(const Chain* model);

  // Solve for all possible solutions for base_t_tip. The number of solutions
  // found is returned. The valid solutions will be placed at the beginning of
  // solutions. The remaining solutions will be invalid.
  int Solve(
      const Pose3d& desired_base_t_tip, const JointStateP& prev_joint_state,
      std::array<eigenmath::VectorNd, kSolutionBufferSize>* solutions) const;

  // Returns the unique branch for a given joint state. Refer to
  // go/branch_labeling_background for theoretical background. Three bits are
  // used:
  //   bit 0: wrist in front of / behind shoulder
  //   bit 1: elbow up / down
  //   bit 2: wrist up / down
  size_t GetBranch(const JointStateP& joint_state) const;

  // Returns true if the passed model defines kinematics that can be
  // solved by the solver. However, note that there are kinematic descriptions
  // of UR arms that the class doesn't handle and will therefore be rejected by
  // this function.
  //
  // This function is more strict than necessary to make parameter extraction
  // easier (certain link lengths, and plane offsets for example).
  static bool ValidateURKinematics(const Chain& model);

 protected:
  const Chain* model_ = nullptr;
  const Joint* joints_[kNbDof];

  // Canonical link lengths required for computing kinematic branch.
  double l2_, l3_, l4_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_UR_IK_SOLVER_H_
