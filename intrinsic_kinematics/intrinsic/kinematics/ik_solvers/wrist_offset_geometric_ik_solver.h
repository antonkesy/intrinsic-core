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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_WRIST_OFFSET_GEOMETRIC_IK_SOLVER_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_WRIST_OFFSET_GEOMETRIC_IK_SOLVER_H_

#include <array>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "gtest/gtest_prod.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/dh_params.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// Inverse kinematic solver for serial robot with a wrist offset.
// In terms of DH parameters, these robots have fabs(joint 4 alpha) = fabs(joint
// 5 alpha). Put another way, the line from joint 3 to 4 is parallel but offset
// from the line from joint 5 to the tip.
//
// Currently failing to find some solutions that are at singularity. The trade
// off chosen is to return solution with guarantee accuracy or no solution.
//
// This solver is meant to be real-time safe, but it's currently not.
// TODO(jeanfrancoisd, efernan): Fix this solver to make real-time safe.
class WristOffsetGeometricIKSolver {
 public:
  // The maximum number of solutions that can be found.
  static constexpr int kSolutionBufferSize = 40;

  WristOffsetGeometricIKSolver() = default;

  absl::Status Init(const Chain& chain);

  // Solve for all possible solutions for base_t_tip. The number of solutions
  // found is returned. The valid solutions will be placed at the beginning of
  // solutions. The remaining solutions will be invalid.
  int Solve(const Pose3d& base_t_tip,
            std::array<eigenmath::Vector6dAligned, kSolutionBufferSize>*
                solutions) const;

  struct Parameters {
    // Denavit-Hartenberg parameters.
    DhParams dh[7];

    // Derived parameters
    double dh_d5_x;
    double dh_d5_z;
    double r01;
    double r24;
    double cos_wrist_angle;
  };

  static absl::StatusOr<Parameters> ExtractParameters(const Chain& chain);

 protected:
  // Constructor for testing that allows parameters to be set directly.
  explicit WristOffsetGeometricIKSolver(const Parameters& parameters);

  Parameters parameters_;

  bool is_initialized_ = false;

  // Friend declarations for testing.
  FRIEND_TEST(WristOffsetGeometricIKSolverTest, CalibrationPosesTest);
  FRIEND_TEST(WristOffsetGeometricIKSolverTest, FKIKTest);
  FRIEND_TEST(WristOffsetGeometricIKSolverTest, RandomFKIKTest);
  FRIEND_TEST(WristOffsetGeometricIKSolverTest, LegacyIKFKTest_NJ4_110_22i);
  FRIEND_TEST(WristOffsetGeometricIKSolverTest, LegacyIKFKTest_NJ4_170_25i);
  FRIEND_TEST(WristOffsetGeometricIKSolverTest, LegacyIKTest_NJ4_170_25i);
  friend int BenchmarkSolve(const Parameters& params, const Pose3d& base_t_tip);
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_WRIST_OFFSET_GEOMETRIC_IK_SOLVER_H_
