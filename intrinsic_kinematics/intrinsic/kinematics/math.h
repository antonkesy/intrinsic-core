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

#ifndef INTRINSIC_KINEMATICS_MATH_H_
#define INTRINSIC_KINEMATICS_MATH_H_

#include <limits>

#include "absl/log/log.h"
#include "intrinsic/eigenmath/manifolds.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

/** Matrix pseudoinverse.
 *  @note this function is real-time safe when using static Eigen types
 *  @tparam N the number of degrees of freedom, or Dynamic for dynamically
 * allocated types
 *  @param M an input matrix for which the inverse is required
 *  @param threshold percentage over the maximum singular value used to cancel
 * small singular values
 *  @return the pseudoinverse of M
 */
template <int ROWS = ::Eigen::Dynamic, int COLS = ::Eigen::Dynamic>
eigenmath::Matrixd<COLS, ROWS> PseudoInverse(
    const eigenmath::Matrixd<ROWS, COLS>& M, double threshold) {
  Eigen::JacobiSVD<eigenmath::Matrixd<ROWS, COLS>> svdOfM(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& U = svdOfM.matrixU();
  const auto& V = svdOfM.matrixV();
  const auto& S = svdOfM.singularValues();

  auto Sinv = S;
  double maxsv = S.maxCoeff();
  for (unsigned int i = 0; i < S.rows(); ++i) {
    if (fabs(S(i)) > maxsv * threshold) {
      Sinv(i) = 1.0 / S(i);
    } else {
      Sinv(i) = 0.0;
    }
  }

  eigenmath::Matrixd<COLS, ROWS> sigmat;
  sigmat.resize(M.cols(), M.rows());
  sigmat.fill(0.0);
  sigmat.diagonal() = Sinv;
  return V * sigmat * U.transpose();
}

/** Damped least-squares matrix inverse.
 *  @note this function is real-time safe when using static Eigen types
 *  @tparam N the number of degrees of freedom, or Dynamic for dynamically
 * allocated types
 *  @param M an input matrix for which the inverse is required
 *  @param lambda the desired damping coefficient (1 >> d > epsilon).
 *  @return the DLS inverse of M
 */
template <int ROWS = ::Eigen::Dynamic, int COLS = ::Eigen::Dynamic>
eigenmath::Matrixd<COLS, ROWS> DlsInverse(
    const eigenmath::Matrixd<ROWS, COLS>& M, double lambda) {
  constexpr double eps = std::numeric_limits<double>::epsilon();
  if (lambda <= eps) {
    INTRINSIC_RT_LOG(WARNING)
        << "Damping coefficient should be greater than epsilon. lambda="
        << lambda << " epsilon=" << eps << ". Using lambda=epsilon.";
    lambda = eps;
  }

  eigenmath::Matrixd<COLS, ROWS> result =
      M.transpose() *
      (M * M.transpose() +
       (lambda * lambda) *
           eigenmath::Matrixd<ROWS, ROWS>::Identity(M.rows(), M.rows()))
          .llt()
          .solve(eigenmath::Matrixd<ROWS, ROWS>::Identity(M.rows(), M.rows()));
  return result;
}

/** Returns a 6x6 matrix that rotates velocities between two frames.
 *  @note this function is real-time safe
 *  @param a_pose_b a transform between two frames
 *  @return a 6x6 matrix that rotates velocities from b to a, with these
 * velocities given as [v w]^T, being v the translational velocity, and w the
 * rotational one.
 */
eigenmath::Matrix6d VelocityRotationMatrix(const Pose3d& a_pose_b);

/** Computes a pose vector from an homogeneous transformation matrix.
 *  @note this function is real-time safe
 *  The pose resulting vector is in the form [t utheta]^T, i.e. translation +
 * axis/angle.
 *  @param M a transform between two frames
 *  @return a 6x1 vector containing the translation and rotation (axis/angle)
 * of M
 */
inline eigenmath::Vector6d TransformToPoseVector(const Pose3d& M) {
  return intrinsic::eigenmath::logRiemann(M);
}

/**
 * Evaluates the pose error from root_pose_estimate to root_pose_target,
 * expressed in the root frame.
 * @param[in]  root_pose_estimate The 'current' pose
 * @param[in]  root_pose_target   The 'desired' pose
 * @return pose_error The 6-DOF pose error (desired - current) expressed in the
 * root frame.
 */
template <typename Scalar, int OptionsEstimate, int OptionsTarget>
eigenmath::Vector6d EvaluatePoseError(
    const Pose3<Scalar, OptionsEstimate>& root_pose_estimate,
    const Pose3<Scalar, OptionsTarget>& root_pose_target) {
  // Compute the pose representing the transformation
  // between the target and the estimate
  eigenmath::Vector6d pose_error =
      intrinsic::eigenmath::logRiemann(root_pose_estimate, root_pose_target);

  // Pose_error is currently in estimate link frame - it just needs to be
  // rotated, but not translated, back into origin frame
  const eigenmath::Quaternion<Scalar, Eigen::AutoAlign>& root_R_estimate =
      root_pose_estimate.quaternion();
  pose_error.head<3>() = root_R_estimate * pose_error.head<3>();
  pose_error.tail<3>() = root_R_estimate * pose_error.tail<3>();
  return pose_error;
}

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_MATH_H_
