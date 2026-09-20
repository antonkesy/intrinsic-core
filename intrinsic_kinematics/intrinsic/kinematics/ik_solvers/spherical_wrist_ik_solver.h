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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_SPHERICAL_WRIST_IK_SOLVER_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_SPHERICAL_WRIST_IK_SOLVER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic {
namespace kinematics {

using SphericalWristIKSolutionSet = std::array<eigenmath::VectorNd, 8>;

// Enumeration of flags set during computation
enum SphericalWristIKResultFlags : uint64_t {
  // Nothing
  RESULT_BLANK = 1 << 0,
  // Everything went OK
  RESULT_OK = 1 << 1,
  // No result found
  RESULT_NONE = 1 << 2,
  // Invalid input
  RESULT_INPUT_INVALID = 1 << 3,
  // q[0] indeterminate (infinite no. of solutions)
  RESULT_POS_Q0_INDETERMINITE = 1 << 4,
  // Only sum of q[3] and q[5] determinated from input (infinite no. of
  // solutions)
  RESULT_POS_Q35_INDETERMINATE = 1 << 5,
  // Position unreachable
  RESULT_POS_UNREACHABLE = 1 << 6,
  // Wrist above axis 0
  RESULT_VEL_Q0_SINGULARITY = 1 << 7,
  // Wrist velocity orthogonal to manipulator plane infeasible because of Q0
  // singularity
  RESULT_VEL_Q0_INFEASIBLE = 1 << 8,
  // Singular configuration for velocity IK, arm stretched
  RESULT_VEL_STRETCH_SINGULARITY = 1 << 9,
  // Singular configuration for velocity IK, arm stretched and
  // given input not in feasible subspace of velocity directions
  RESULT_VEL_STRETCH_INFEASIBLE = 1 << 10,
  // Singular configuration for velocity IK, wrist singularity
  RESULT_VEL_WRIST_SINGULARITY = 1 << 11,
  // Singular configuration for velocity IK, wrist singularity
  // given input not in feasible subspace of velocity directions
  RESULT_VEL_WRIST_INFEASIBLE = 1 << 12,
  // Singular configuration for acceleration IK, arm stretched
  RESULT_ACC_STRETCH_SINGULARITY = 1 << 13,
  // Singular configuration for acceleration IK, arm stretched and
  // given input not in feasible subspace of velocity directions
  RESULT_ACC_STRETCH_INFEASIBLE = 1 << 14,
  // Singular configuration for acceleration IK, wrist singularity
  RESULT_ACC_WRIST_SINGULARITY = 1 << 15,
  // Singular configuration for acceleration IK, wrist singularity
  // given input not in feasible subspace of velocity directions
  RESULT_ACC_WRIST_INFEASIBLE = 1 << 16,
  // Solution is outside the joint limits.
  RESULT_JOINT_LIMIT_EXCEEDED = 1 << 17,
};

struct SphericalWristIKParameters {
  // Relevant parameters extracted from the kinematic model.
  // Reference paper:
  // [1] "Brandstötter, Mathias & Angerer, Arthur & Hofbaur, Michael. (2014).
  //     An Analytical Solution of the Inverse Kinematics Problem of
  //     Industrial Serial Manipulators with an Ortho-parallel Basis and a
  //     Spherical Wrist."
  double basel1z;  // c1 in the paper [1], Fig. 1(a)
  double l12x;     // a1 in the paper [1], Fig. 1(a)
  double l23z;     // c2 in the paper [1], Fig. 1(a)
  double l34z;     // a2 in the paper [1], Fig. 1(a)
  double l34y;     // b  in the paper [1], Fig. 1(b)
  double l45x;     // c3 in the paper [1], Fig. 1(a)
  double l56x;     // c4 in the paper [1], Fig. 1(a)
  // Derived parameters
  double lhypot35;
  double alpha35_p_pi_2;
  double one_over_l23z_m_lhypot35;
  double l23z2_m_lhypot352;

  Pose3d base_offset;

  // Position offset for each joint to match default configuration
  eigenmath::VectorNd joint_offsets;

  // Direction difference (1/-1) for each joint to match default configuration
  eigenmath::VectorNd joint_axis_directions_diff;

  // TODO(b/375462963): Need parameterization improvement, to also take into
  // account the `final_dof_joint_t_flange` pose offset.
};

absl::StatusOr<SphericalWristIKParameters> ExtractParameters(
    const ModelInterface& model) INTRINSIC_NON_REALTIME_ONLY;

std::string ToString(SphericalWristIKResultFlags flags);

// A closed-form IK solver for spherical wrist manipulators.
//
// Assumption: the z-axis of base_t_tip that we are solving for is collinear
// with the wrist and pointing outward.
class SphericalWristIKSolver {
 public:
  static constexpr size_t kNbJoints = 6;
  static constexpr int kMaxNumberSolutions = 8;

  explicit SphericalWristIKSolver(const Chain* chain);

  absl::Status Init() INTRINSIC_NON_REALTIME_ONLY;

  // Returns a unique int for each possible configuration of the arm.
  // Three bits are used:
  //   bit 0: wrist up / down
  //   bit 1: elbow up / down
  //   bit 2: wrist in front of / behind shoulder
  size_t GetConfiguration(const eigenmath::VectorNd& q) const;

  int Solve(const Pose3d& base_t_tip,
            SphericalWristIKSolutionSet* solution_qs) const;

  // Solve for all possible solutions for base_t_tip.
  // The number of solutions returned will be the true solution count.
  // If empty, no solution could be found.
  FixedVector<eigenmath::VectorNd, kMaxNumberSolutions> Solve(
      const Pose3d& base_t_tip) const;

  // Returns the solution which is closest to the hint joint state.
  bool Solve(const Pose3d& base_t_tip, eigenmath::VectorNd* solution_q);

  // Returns the unique branch for a given joint state.
  // Similar to GetConfiguration but using a different computation based on
  // analytical factors obtained from the forward-kinematic Jacobian. Refer to
  // go/branch_labeling_background for theoretical background. Three bits are
  // used:
  //   bit 0: wrist in front of / behind shoulder
  //   bit 1: elbow up / down
  //   bit 2: wrist up / down
  // Note that the order of bits is reversed compared to GetConfiguration.
  // TODO(http://b/250494113)
  size_t GetBranch(const eigenmath::VectorNd& q) const;

  // Returns the smallest distance to any of the three singular positions.
  // The value is normalized in the interval [0, 1] following the convention:
  //    0.0: at singularity,
  //    1.0: furthest possible from singularity
  double GetMinDistanceToSingularity(const eigenmath::VectorNd& q) const;

  const eigenmath::VectorNd& joint_offsets() const {
    return parameters_.joint_offsets;
  }
  const eigenmath::VectorNd& joint_axis_directions_diff() const {
    return parameters_.joint_axis_directions_diff;
  }

  const SphericalWristIKParameters& parameters() const { return parameters_; }

  // Returns the (ground-truth) maximum arm length of the robot
  // manipulator, computed by the Spherical Wrist Inverse Kinematics (IK)
  // solver's extracted kinematic parameters (e.g. those extracted from the
  // robot manipulator's URDF file). The maximum arm length is defined as the
  // maximum distance between the base and the tip of the robot manipulator.
  double GetMaximumArmLength() const;

  void SetNearestJointStateHint(const eigenmath::VectorNd& q) { nearby_q_ = q; }

  // Sorts joint states within `qs` according to distance to `nearby_q` after
  // unwrapping to the closest solution. Closest solution is first element.
  // Important: if two solutions are equidistant to `nearby_q`, solutions on the
  // same kinematic branch as `nearby_q` are listed first.
  icon::RealtimeStatus WrapAndSort(const ModelInterface& model,
                                   const JointLimits& dof_limits,
                                   const eigenmath::VectorNd& nearby_q,
                                   absl::Span<eigenmath::VectorNd> qs);

 protected:
  SphericalWristIKParameters parameters_;

  const Chain* chain_;

  eigenmath::VectorNd nearby_q_;

  void GetDisplacementToSingularity(
      const eigenmath::VectorNd& q_orig,
      double* displacement_to_wrist_singularity,
      double* displacement_to_elbow_singularity,
      double* displacement_to_overhead_singularity) const;

  bool is_initialized_ = false;

  // Extracting the dof limits from the model is somewhat expensive so we keep
  // it here.
  intrinsic::JointLimits dof_limits_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_SPHERICAL_WRIST_IK_SOLVER_H_
