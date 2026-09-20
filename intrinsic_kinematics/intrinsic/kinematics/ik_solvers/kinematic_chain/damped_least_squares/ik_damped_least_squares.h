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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_DAMPED_LEAST_SQUARES_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_DAMPED_LEAST_SQUARES_H_

#include "absl/log/check.h"
#include "absl/time/time.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/damped_least_squares/ik_base.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/util/time/clock_steady.h"
#include "intrinsic/util/time/elapsed_timer.h"

namespace intrinsic {
namespace kinematics {

struct IKDampedLeastSquaresParameters : public IKParametersBase {
  IKDampedLeastSquaresParameters() = default;
  IKDampedLeastSquaresParameters(int32_t max_iterations_in,
                                 double position_error_tolerance_in,
                                 double orientation_error_tolerance_in,
                                 double position_clamp_distance_in,
                                 double orientation_clamp_distance_in,
                                 double damping_coefficient_in)
      : IKParametersBase(max_iterations_in, position_error_tolerance_in,
                         orientation_error_tolerance_in,
                         position_clamp_distance_in,
                         orientation_clamp_distance_in),
        damping_coefficient(damping_coefficient_in) {}

  /**
   * A non-zero damping coefficient for the solver.
   */
  double damping_coefficient = 0.08;
};

/**
 * The IKDampedLeastSquaresSolver uses the Damped Least Squares method, via
 * Eigen's JacobiSVD solver, to iteratively solve for an inverse kinematics
 * solution in a way that has better stability near singularities.
 * The solver is configured at construction with the damping coefficient,
 * error tolerances, maximum
 * iterations, and max step distance. A "request" struct is used to ask the
 * solver for a solution - this allows for the allocated space required during
 * a solution to be pre-allocated, so that the solution call itself is
 * real-time safe.
 * @tparam N The number of DOFs to solve for. A value of -1 (Eigen::Dynamic) can
 * be used when the number of DOFs is not known at compile time.
 */
template <int N = Eigen::Dynamic>
class IKDampedLeastSquaresSolver
    : public IKSolverBase<IKDampedLeastSquaresParameters, N> {
 public:
  // The following qualification is required to access members and functions of
  // the base class.
  using Base = IKSolverBase<IKDampedLeastSquaresParameters, N>;
  using Parameters = typename Base::Parameters;

  /**
   * The Request struct pre-allocates all the space required for a solution
   * to be calculated for a given target pose and starting joint
   * configuration. The usage is to create a Request (usually by calling
   * IKDampedLeastSquaresSolver::createRequest(), and then setting
   * the transform target root_pose_link_target and the starting joint vector
   * q_start.  Then call IKDampedLeastSquaresSolver::solve(request), and
   * the resulting request will contain a status (true/false) whether the
   * solution was found, and additional data about the quality of the
   * solution.
   */
  struct Request : public Base::RequestBase {
   private:
    friend class IKDampedLeastSquaresSolver;
    Request(size_t link_count, size_t joint_count, size_t dof_count)
        : Base::RequestBase(link_count, joint_count, dof_count),
          scratch(dof_count) {}

    /**
     * The scratch struct contains allocated space needed during the
     * computation of the solution, but not interesting to the user.
     */
    struct Scratch {
      explicit Scratch(size_t dof_count)
          : clamped_pose_error(6),
            J(6, dof_count),
            Jt(dof_count, 6),
            J_Jt_plus_Lambda_I(6, 6),
            dls_solution(6) {}
      eigenmath::Vector6d clamped_pose_error;
      eigenmath::Matrixd<6, N> J;              // NOLINT
      eigenmath::Matrixd<N, 6> Jt;             // NOLINT
      eigenmath::Matrix6d J_Jt_plus_Lambda_I;  // NOLINT
      eigenmath::Vector6d dls_solution;
    } scratch;
  };

  /**
   * To construct the IKDampedLeastSquaresSolver, an model is required,
   * as is an externally created forward position kernel and jacobian
   * kernel. The IK Solver will retain these by reference, and so will
   * require them to exist during its lifetime.
   * The Solver also explicitly requires the tip_id of the element being
   * solved for.
   * @param model  A const reference to an externally managed ModelInterface.
   * @param params  The configuration parameters
   * @param tip_id  The id of the element to solve for. If
   * kInvalidElementId, the tip will be used. Assuming a single tip.
   */
  IKDampedLeastSquaresSolver(const ModelInterface& model,
                             const Parameters& params,
                             const ElementId& tip_id = kInvalidElementId);

  /**
   * Creates a request object suitable for this solver.
   */
  Request createRequest() const {
    return Request{Base::link_count_, Base::joint_count_, Base::dof_count_};
  }

  /**
   * Solves a request, putting the resulting solution information into
   * the request object's output fields.
   * This function is safe to call in real-time.
   */
  void solve(Request* request) const;
};

// MEMBER FUNCTION IMPLEMENTATIONS

template <int N>
IKDampedLeastSquaresSolver<N>::IKDampedLeastSquaresSolver(
    const ModelInterface& model, const Parameters& params,
    const ElementId& tip_id)
    : IKSolverBase<IKDampedLeastSquaresParameters, N>(model, params, tip_id) {}

template <int N>
void IKDampedLeastSquaresSolver<N>::solve(Request* request) const {
  CHECK(request != nullptr);
  CHECK_EQ(request->q_start.size(), request->q.size())
      << "Inconsistent number of DOFs in IK Damped Least Squares";
  CHECK_EQ(request->q.size(), Base::model_->GetNumberDegreesOfFreedom())
      << "Inconsistent number of DOFs in IK Damped Least Squares";

  // Init status
  request->status = false;
  request->duration_milliseconds = 0.0;
  request->iterations = 0;

  // Initialize
  request->q = request->q_start;
  Base::enforceDofLimits(&request->q);

  // Create a timer, which starts it running.
  ClockSteady clock_steady;
  ElapsedTimer timer(&clock_steady);

  // Loop over pose refinements until we reach an acceptable error
  // or run out of iterations.
  do {
    Base::evaluatePoseError(request->q, request->root_pose_link_target,
                            &request->pose_error);

    // Test the pose error against our success tolerances, and if it
    // is good enough, return. Otherwise, clamp the pose error against
    // the clamp distances and prepare for pose improvement.
    if (Base::testAndClampPoseError(request->pose_error,
                                    &request->scratch.clamped_pose_error)) {
      request->duration_milliseconds =
          absl::ToDoubleMilliseconds(timer.Elapsed());
      request->status = true;
      return;
    }

    Base::computeJacobian(request->q, &request->scratch.J);

    request->scratch.Jt = request->scratch.J.transpose();

    request->scratch.J_Jt_plus_Lambda_I =
        request->scratch.J * request->scratch.Jt;
    request->scratch.J_Jt_plus_Lambda_I.diagonal() +=
        request->scratch.J_Jt_plus_Lambda_I.diagonal().Constant(
            Base::parameters_.damping_coefficient);

    request->scratch.dls_solution =
        request->scratch.J_Jt_plus_Lambda_I.llt().solve(
            request->scratch.clamped_pose_error);

    request->q += request->scratch.Jt * request->scratch.dls_solution;

    Base::enforceDofLimits(&request->q);

    ++request->iterations;
  } while (request->iterations < Base::parameters_.max_iterations);

  // If we get here, the solver ran out of iterations.
  // Set the status to false, but also include the timing information.
  request->duration_milliseconds = absl::ToDoubleMilliseconds(timer.Elapsed());
  request->status = false;
}

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_DAMPED_LEAST_SQUARES_H_
