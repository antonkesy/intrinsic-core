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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_VELOCITY_IK_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_VELOCITY_IK_H_

#include "absl/log/check.h"
#include "absl/strings/str_format.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/math.h"

namespace intrinsic {
namespace kinematics {

/**
 * The signature of a function to be used as a null-space task for the IK
 * velocity solver. The function will take as input a vector with the current
 * joint angles, and needs to provide a velocity in joint space as output. That
 * velocity will be projected later into the robot nullspace.
 * @tparam N the number of degrees of freedom, or Dynamic for dynamically
 * allocated types
 */
template <int N = ::Eigen::Dynamic>
using NullSpaceTaskFunction =
    std::function<eigenmath::Vectord<N>(const eigenmath::Vectord<N>&)>;

/**
 * Empty nullspace task.
 * @note real-time safe when using static Eigen types
 * @tparam N the number of degrees of freedom, or Dynamic for dynamically
 * allocated types
 * @param q the input joint configuration
 * @returns a zero-vector with the same size as q
 */
template <typename VectorType>
VectorType EmptyNullSpaceTaskFunction(const VectorType& q) {
  // won't allocate memory for fixed-size vectors
  VectorType result(q.rows());
  result.fill(0.0);
  return result;
}

/** Compute velocity IK, i.e. map from Cartesian velocities to joint velocities.
 *  This method implements the following equation:
 *      \f$ \dot{q} = J^+ \dot{x} + (I - J^+J) \dot{q_{st}} \f$
 *  @note real-time safe when using static Eigen types and real-time safe
 * inversion and nullspace functions
 *  @tparam N the number of degrees of freedom, or Dynamic for dynamically
 * allocated types
 *  @tparam InversionFunctionType a type implementing the function call operator
 * with the following signature: `eigenmath::Matrixd<N,6>(const
 * eigenmath::Matrixd<6,N> &)`
 *  @tparam NullSpaceTaskFunctionType a type implementing the function call
 * operator with the following signature: `eigenmath::Vectord<N>(const
 * eigenmath::Vectord<N> &)`
 *  @param q the current joint configuration
 *  @param J the jacobian at joint configuration q
 *  @param inversion_function a function to be used for Jacobian inversion
 *  @param nullspace_task_function a function to be used as the null-space task
 *  @param xdot the desired velocity in Cartesian space
 *  @returns the joint velocity (qdot) required to achieve xdot
 */
template <int N, typename InversionFunctionType,
          typename NullSpaceTaskFunctionType>
eigenmath::Vectord<N> velocityIK(
    const eigenmath::Vectord<N>& q, const eigenmath::Matrixd<6, N>& J,
    const InversionFunctionType& inversion_function,
    const NullSpaceTaskFunctionType& nullspace_task_function,
    const eigenmath::Vector6d& xdot) {
  CHECK_EQ(q.rows(), J.cols())
      << "Jacobian / joint position vector size mismatch";

  const eigenmath::Matrixd<N, 6> Jinv = inversion_function(J);
  CHECK_EQ(J.cols(), Jinv.rows()) << "Invalid Jacobian inverse";

  // Early return if no nullspace function provided
  const eigenmath::Vectord<N> ns_velocity = nullspace_task_function(q);
  if (ns_velocity.isZero()) {
    return Jinv * xdot;
  }

  eigenmath::Matrixd<N> I(J.cols(), J.cols());
  I.setIdentity();
  CHECK_EQ(q.rows(), ns_velocity.rows())
      << "Nullspace velocity needs one element per joint";
  return Jinv * xdot + (I - Jinv * J) * ns_velocity;
}

/** Compute velocity IK using the Jacobian pseudoinverse.
 *  @note real-time safe when using static Eigen types and a real-time safe
 * nullspace function.
 *  @tparam N the number of degrees of freedom, or Dynamic for dynamically
 * allocated types
 *  @tparam NullSpaceTaskFunctionType a type implementing the function call
 * operator with the following signature: `eigenmath::Vectord<N>(const
 * eigenmath::Vectord<N> &)`
 *  @param q the current joint configuration.
 *  @param J the jacobian at joint configuration q
 *  @param singular_value_threshold percentage over the maximum singular value
 * used to cancel small singular values (\sa pseudoInverse)
 *  @param nullspace_task_function a function to be used as the null-space task
 *  @param xdot the required displacement in Cartesian space
 *  @returns the joint velocity (qdot) required to achieve xdot
 */
template <int N, typename NullSpaceTaskFunctionType>
eigenmath::Vectord<N> pseudoInverseVelocityIK(
    const eigenmath::Vectord<N>& q, const eigenmath::Matrixd<6, N>& J,
    double singular_value_threshold,
    const NullSpaceTaskFunctionType& nullspace_task_function,
    const eigenmath::Vector6d& xdot) {
  return velocityIK<N>(q, J,
                       std::bind(PseudoInverse<6, N>, std::placeholders::_1,
                                 singular_value_threshold),
                       nullspace_task_function, xdot);
}

/** Compute velocity IK using the Jacobian Damped Least Squares (DLS) inverse.
 *  @note real-time safe when using static Eigen types and a real-time safe
 * nullspace function.
 *  @tparam N the number of degrees of freedom, or Dynamic for dynamically
 * allocated types
 *  @tparam NullSpaceTaskFunctionType a type implementing the function call
 * operator with the following signature: `eigenmath::Vectord<N>(const
 * eigenmath::Vectord<N> &)`
 *  @param q the current joint configuration
 *  @param J the jacobian at joint configuration q
 *  @param lambda the desired damping coefficient (\sa dlsInverse)
 *  @param nullspace_task_function a function to be used as the null-space task
 *  @param xdot the required displacement in Cartesian space
 *  @returns the joint velocity (qdot) required to achieve xdot
 */
template <int N, typename NullSpaceTaskFunctionType>
eigenmath::Vectord<N> dlsVelocityIK(
    const eigenmath::Vectord<N>& q, const eigenmath::Matrixd<6, N>& J,
    double lambda, const NullSpaceTaskFunctionType& nullspace_task_function,
    const eigenmath::Vector6d& xdot) {
  return velocityIK<N>(
      q, J, std::bind(DlsInverse<6, N>, std::placeholders::_1, lambda),
      nullspace_task_function, xdot);
}

// Compute velocity IK by solving
//           1/2*qdot^T*qdot - z^T*qdot -> min!
//           s.t. xdot-J*qdot = 0,
// where z is used to produce a desired nullspace velocity.
// [see, e.g,. Nakamura "Advanced Robotics: Redundancy and Optimization,"
//  1991.]
// Uses additional damping to be robust to singularities.
// This function is substantially the same as dlsVelocityIK, but a little
// faster as it avoids computing pseudo inverses and nullspace projection
// matrices.
// Note: real-time safe when using static Eigen types and a real-time safe
//   nullspace function.
// N is the number of degrees of freedom, or Eigen::Dynamic for dynamically
//   allocated types
// NullSpaceTaskFunctionType is a type implementing the function call operator
// with the following signature: `eigenmath::Vectord<N>(const
// eigenmath::Vectord<N> &)` q is the current joint configuration J is the
// Jacobian at joint configuration q damping is the desired damping coefficient
// nullspace_task_function a function to be used as an additional desired
//   nullspace veloity (e.g,. the negated gradient of a nullspace cost
//   function).
// xdot the required displacement in Cartesian space
//
// NOTE: You should never need to manually provide EigenOptions, MaxVectorSize
// and MaxMatrixSize. The compiler can infer them from the Vector/Matrix types
// you pass to the template.
// We use Eigen types here directly (rather than the eigenmath:: typedefs) to
// allow calling this with both eigenmath::VectorXd and eigenmath::VectorNd.
//
// Returns the joint velocity (qdot) required to achieve xdot.
template <int N, typename NullSpaceTaskFunctionType, int EigenOptions,
          int MaxVectorSize, int MaxMatrixSize>
Eigen::Matrix<double, N, 1, EigenOptions, MaxVectorSize, 1> dQPVelocityIK(
    const Eigen::Matrix<double, N, 1, EigenOptions, MaxVectorSize, 1>& q,
    const Eigen::Matrix<double, 6, N, EigenOptions, 6, MaxMatrixSize>& J,
    double damping, const NullSpaceTaskFunctionType& nullspace_task_function,
    const eigenmath::Vector6d& xdot) {
  CHECK_EQ(q.rows(), J.cols())
      << "Jacobian / joint position vector size mismatch";
  CHECK_GT(damping, 0.0) << absl::StrFormat("damping is %e but must be > 0.0.",
                                            damping);
  // Lagrangian: L = 1/2*qdot^2-z^T*qdot + lambda^T*(xdot-J*qdot)
  // to solve: 1) qdot-z-J^T*lambda = 0
  //           2) xdot -J*qdot = 0
  // Solve 1) for qdot, substitute in 2):
  //           3) J*J^T*lambda+J*z -xdot = 0.
  // Solving 3) for lambda and substituting in 1) gives qdot.
  // Here, J*J^T is regularized to be robust close to singularities.
  const Eigen::Matrix<double, N, 1, EigenOptions, MaxVectorSize, 1> z =
      nullspace_task_function(q);
  CHECK_EQ(q.rows(), z.rows())
      << "Nullspace velocity needs one element per joint";
  const eigenmath::Matrix6d JJt_regularized =
      J * J.transpose() + eigenmath::Matrix6d::Identity() * (damping * damping);
  // Solve for lambda using LLT, which always works as damping > 0.
  if (z.isZero()) {
    const eigenmath::Vector6d lambda = JJt_regularized.llt().solve(xdot);
    return J.transpose() * lambda;
  }
  const eigenmath::Vector6d lambda = JJt_regularized.llt().solve(xdot - J * z);
  return J.transpose() * lambda + z;
}
/**
 * Enumeration of strategies to handle joint limits during incremental IK.
 */
enum class JointLimitStrategy : int {
  SCALE_ENTIRE_VECTOR =
      0,  // The entire update vector will be scaled to respect joint limits.
  CLIP_EACH_JOINT =
      1  // Each joint is clipped independently to respect joint limits.
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_VELOCITY_IK_H_
