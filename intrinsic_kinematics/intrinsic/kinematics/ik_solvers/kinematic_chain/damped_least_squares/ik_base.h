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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_BASE_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_BASE_H_

#include "absl/log/check.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/joint_wrapping.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/math.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/units.h"
#include "intrinsic/util/eigen.h"

namespace intrinsic {
namespace kinematics {

/**
 * Configuration parameters for constructing an IKSolver
 */
struct IKParametersBase {
  IKParametersBase() = default;
  IKParametersBase(int32_t max_iterations_in,
                   double position_error_tolerance_in,
                   double orientation_error_tolerance_in,
                   double position_clamp_distance_in,
                   double orientation_clamp_distance_in)
      : max_iterations(max_iterations_in),
        position_error_tolerance(position_error_tolerance_in),
        orientation_error_tolerance(orientation_error_tolerance_in),
        position_clamp_distance(position_clamp_distance_in),
        orientation_clamp_distance(orientation_clamp_distance_in) {}

  /**
   * The maximum number of iterations in the solver.
   */
  int32_t max_iterations = 2048;

  /**
   * The squared norm of positional difference which must be achieved
   *     between the target position and the solved position before a
   *     solution is accepted.
   */
  double position_error_tolerance = 0.001;

  /**
   * The squared norm of orientational difference which must be achieved
   *     (using axis-angle formulation) between the target orientation and
   *     the solved orientation before a solution is accepted.
   */
  double orientation_error_tolerance = DegToRad(1.0);

  /**
   * The distance greater than which positional pose errors will be
   *     clamped, before being used to calculate deltas.
   *     This is normally set to about half the average link length.
   */
  double position_clamp_distance = 0.35;

  /**
   * The distance greater than which orientational pose errors will be
   *     clamped, before being used to calculate deltas.
   */
  double orientation_clamp_distance = DegToRad(20.0);
};

/**
 * The IKSolverBase provides common functionality for IK Solvers.
 * @tparam N The number of DOFs to solve for. A value of -1 (Eigen::Dynamic) can
 * be used when the number of DOFs is not known at compile time.
 */
template <typename DerivedParameters, int N = Eigen::Dynamic>
class IKSolverBase {
 public:
  using Parameters = DerivedParameters;

  using VectorNd = eigenmath::Vectord<N>;
  using Matrix6Nd = eigenmath::Matrixd<6, N>;
  using MatrixN6d = eigenmath::Matrixd<N, 6>;

  /**
   * The RequestBase struct pre-allocates all the common space required for a
   * solution to be calculated for a given target pose and starting joint
   * configuration. It is intended as a base class for Request types in
   * derived IK Solvers.
   */
  struct RequestBase {
   protected:
    /**
     * @brief construct a RequestBase explicitly
     * @param link_count  number of links
     * @param joint_count  number of joints
     * @param dof_count  number of dofs
     */
    RequestBase(size_t link_count, size_t joint_count, size_t dof_count)
        : q_start(dof_count),
          q(dof_count),
          status(false),
          duration_milliseconds(0.0),
          iterations(0) {}

   public:
    //----------------------------------------------------------------------
    // SET THESE
    //----------------------------------------------------------------------
    /**
     * root_pose_link_target is the desired coordinate frame of the
     * link that the solver is configured for.
     */
    Pose3d root_pose_link_target;

    /**
     * q_start is the starting joint configuration from which the
     * solution will be searched for.
     */
    eigenmath::VectorNd q_start;

    //----------------------------------------------------------------------
    // GET BACK EVERYTHING BELOW
    //----------------------------------------------------------------------
    /**
     * q is the solved joint vector, if a solution was able to be found.
     */
    eigenmath::VectorNd q;

    /**
     * pose_error is the transformation, as a pose vector, of the
     * predicted pose to the target pose.
     */
    eigenmath::Vector6d pose_error;

    /**
     * status indicates whether a solution was found.
     */
    bool status;

    /**
     * duration_milliseconds is how long, in real time, the solver
     * took to search for the solution.
     */
    double duration_milliseconds;

    /**
     * iterations is the number of iterations the solver took to find
     * the solution.
     */
    int32_t iterations;
  };

  /**
   * An ik solver for the provided id of the element to solve for. If
   * kInvalidElementId, the tip will be used. Assuming a single tip.
   * @param parameters  configuration parameters
   */
  IKSolverBase(const ModelInterface& model, const Parameters& params,
               const ElementId& tip_id = kInvalidElementId);

 protected:
  void enforceDofLimits(eigenmath::VectorNd* q) const;
  void evaluatePoseError(const eigenmath::VectorNd& q,
                         const Pose3d& root_pose_link_target,
                         eigenmath::Vector6d* pose_error) const;

  /**
   * Tests to see whether the magnitude of the position and
   * orientation errors are less than the tolerances. Also clamps
   * both errors to the clamp distance and returns the clamped
   * result via reference.
   */
  bool testAndClampPoseError(const eigenmath::Vector6d& pose_error,
                             eigenmath::Vector6d* clamped_pose_error) const;
  void computeJacobian(const eigenmath::VectorNd& dof_values,
                       IKSolverBase::Matrix6Nd* J) const;

  const ModelInterface* model_;
  Parameters parameters_;
  ElementId tip_id_;

  size_t link_count_;
  size_t joint_count_;
  size_t dof_count_;
};

// HELPER FUNCTIONS

/**
 * @param vec6 A kinematic pose with orientation as an axis-angle
 * @return the translation component of the pose, as a modifiable reference
 */
template <typename Scalar, int Options>
auto translation(eigenmath::Vector6<Scalar, Options>* vec6)
    -> decltype(vec6->template head<3>()) {
  return vec6->template head<3>();
}

/**
 * @param vec6 A const kinematic pose with orientation as an axis-angle
 * @return the translation component of the pose, as a const reference
 */
template <typename Scalar, int Options>
auto translation(const eigenmath::Vector6<Scalar, Options>& vec6)
    -> decltype(vec6.template head<3>()) {
  return vec6.template head<3>();
}

/**
 * @param vec6 A kinematic pose with orientation as an axis-angle
 * @return the orientation component of the pose, as a modifiable reference
 */
template <typename Scalar, int Options>
auto rotation(eigenmath::Vector6<Scalar, Options>* vec6)
    -> decltype(vec6->template tail<3>()) {
  return vec6->template tail<3>();
}

/**
 * @param vec6 A const kinematic pose with orientation as an axis-angle
 * @return the orientation component of the pose, as a const reference
 */
template <typename Scalar, int Options>
auto rotation(const eigenmath::Vector6<Scalar, Options>& vec6)
    -> decltype(vec6.template tail<3>()) {
  return vec6.template tail<3>();
}

// MEMBER FUNCTION IMPLEMENTATIONS

template <typename DerivedParameters, int N>
IKSolverBase<DerivedParameters, N>::IKSolverBase(const ModelInterface& model,
                                                 const Parameters& params,
                                                 const ElementId& tip_id)
    : model_(&model), parameters_(params), tip_id_(tip_id) {
  // If a link index of -1 is requested, use the maximum link index,
  // which we assume to be the end effector.
  if (tip_id_ == kInvalidElementId) {
    auto tips = model_->GetTipIds();
    CHECK_EQ(tips.size(), 1)
        << "There should be a single tip. Got: " << tips.size();
    tip_id_ = tips.front();
  }

  CHECK(model_->GetElement(tip_id_).ok());

  link_count_ = model_->GetAllLinkIds().size();
  joint_count_ = model_->GetAllJointIds().size();
  dof_count_ = model_->GetNumberDegreesOfFreedom();
}

template <typename DerivedParameters, int N>
void IKSolverBase<DerivedParameters, N>::enforceDofLimits(
    eigenmath::VectorNd* q) const {
  CHECK_EQ(q->size(), dof_count_)
      << "Inconsistent number of DOFs in IK Jacobian Transpose";

  auto limits = model_->GetDofSystemLimits();

  eigenmath::VectorNd nearby_q =
      0.5 * (limits.max_position + limits.min_position);
  eigenmath::VectorNd q_tmp = *q;  // Temporary change of type
  INTRINSIC_RT_ASSIGN_OR_DIE(bool wrap_success,
                             Wrap(*model_, limits, nearby_q, &q_tmp));
  if (!wrap_success) {
    VLOG(2) << "Couldn't wrap q=" << toString(q_tmp)
            << ", lower=" << toString(limits.min_position)
            << ", upper=" << toString(limits.max_position);
  }
  *q = q_tmp;
}

template <typename DerivedParameters, int N>
void IKSolverBase<DerivedParameters, N>::evaluatePoseError(
    const eigenmath::VectorNd& q, const Pose3d& root_pose_link_target,
    eigenmath::Vector6d* pose_error) const {
  State kinematics_state(model_);
  // It is possible that we want to measure the error for something outside the
  // limits.
  CHECK_EQ(kinematics_state.SetDofPositions(q, /*check_limits=*/false),
           icon::OkStatus());
  INTRINSIC_RT_ASSIGN_OR_DIE(auto base_t_tip,
                             kinematics_state.GetTransform(tip_id_));

  // Evaluate the pose error expressed in root frame
  *pose_error = EvaluatePoseError(base_t_tip, root_pose_link_target);
}

template <typename DerivedParameters, int N>
bool IKSolverBase<DerivedParameters, N>::testAndClampPoseError(
    const eigenmath::Vector6d& pose_error,
    eigenmath::Vector6d* clamped_pose_error) const {
  // Clamping the error distance used to calculate our next step
  // helps remove oscillations when the target positions are too far.
  // The position and orientation errors are calculated independently,
  // and clamped independently.
  eigenmath::Vector3d pos_error = translation(pose_error);
  double dpos = pos_error.norm();
  if (dpos > parameters_.position_clamp_distance) {
    pos_error *= parameters_.position_clamp_distance / dpos;
  }
  translation(clamped_pose_error) = pos_error;

  eigenmath::Vector3d orient_error = rotation(pose_error);
  double dorient = orient_error.norm();
  if (dorient > parameters_.orientation_clamp_distance) {
    orient_error *= parameters_.orientation_clamp_distance / dorient;
  }
  rotation(clamped_pose_error) = orient_error;

  return (dpos < parameters_.position_error_tolerance) &&
         (dorient < parameters_.orientation_error_tolerance);
}

template <typename DerivedParameters, int N>
void IKSolverBase<DerivedParameters, N>::computeJacobian(
    const eigenmath::VectorNd& dof_values, IKSolverBase::Matrix6Nd* J) const {
  State state(model_);
  // For some numerical methods, we don't want to be bounded to the dof limits.
  CHECK_EQ(state.SetDofPositions(dof_values, /*check_limits=*/false),
           intrinsic::icon::OkStatus());
  INTRINSIC_RT_ASSIGN_OR_DIE(*J, state.ComputeJacobian());
}

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_KINEMATIC_CHAIN_DAMPED_LEAST_SQUARES_IK_BASE_H_
