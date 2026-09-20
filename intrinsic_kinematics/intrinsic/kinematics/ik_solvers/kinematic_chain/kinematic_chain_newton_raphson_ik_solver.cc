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

#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_newton_raphson_ik_solver.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <random>

#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/velocity_ik.h"
#include "intrinsic/kinematics/joint_wrapping.h"
#include "intrinsic/kinematics/math.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/state_generation.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/time/clock_steady.h"
#include "intrinsic/util/time/elapsed_timer.h"

namespace intrinsic {
namespace kinematics {

using eigenmath::Matrix3d;
using eigenmath::Matrix6d;
using eigenmath::Vector6d;
using eigenmath::VectorNd;
using eigenmath::VectorXd;

KinematicChainNewtonRaphsonIKSolver::KinematicChainNewtonRaphsonIKSolver(
    const Chain* kinematic_chain)
    : KinematicChainIKSolver(kinematic_chain),
      seed_q_(VectorXd::Zero(kinematic_chain->GetNumberDegreesOfFreedom())),
      epsilon_(0.0001),
      max_iterations_(100) {
  tip_id_ = kinematic_chain->GetTipId();
}

void KinematicChainNewtonRaphsonIKSolver::setSeed(const VectorXd& seed_q) {
  CHECK_EQ(seed_q.rows(),
           static_cast<int>(kinematic_chain_->GetNumberDegreesOfFreedom()))
      << absl::StrFormat(
             "Wrong number of joints in the input joint vector. Expected %d, "
             "got %zd "
             "joints",
             kinematic_chain_->GetNumberDegreesOfFreedom(), seed_q.rows());
  seed_q_ = seed_q;
  INTRINSIC_RT_ASSIGN_OR_DIE(
      bool is_within_limits,
      IsWithinLimits(JointStateP(seed_q_),
                     kinematic_chain_->GetDofSystemLimits()));
  CHECK(is_within_limits) << "Seed must be within joint limits";
}

IKResult KinematicChainNewtonRaphsonIKSolver::compute(
    const Pose3d& base_pose_tip, const IKValidator& validator) const {
  // TODO(b/427930379): add support for dependent joints.
  INTRINSIC_RT_ASSIGN_OR_DIE(bool has_dependent_joints,
                             kinematic_chain_->HasDependentJoints());
  if (has_dependent_joints) {
    INTRINSIC_RT_LOG(ERROR)
        << "IK solver not supported for robots with dependent "
           "joints. Falling back to no IK solution.";
    // The default IKResult has status = NO_SOLUTION and num_solutions equals
    // zero.
    return {};
  }

  VectorXd inout_q = seed_q_;
  bool ik_found = KinematicChainNewtonRaphsonIKSolver::IK(
      *kinematic_chain_, base_pose_tip, tip_id_, max_iterations_, epsilon_,
      validator, &inout_q);
  if (ik_found) {
    return {inout_q};
  }
  return {};
}

bool KinematicChainNewtonRaphsonIKSolver::IK(
    const Chain& kinematics, const Pose3d& base_pose_tip,
    const ElementId& tip_id, std::size_t max_iterations, double epsilon,
    const IKValidator& validator, VectorXd* seed_q) {
  CHECK_EQ(seed_q->rows(),
           static_cast<int>(kinematics.GetNumberDegreesOfFreedom()))
      << absl::StrFormat(
             "Wrong number of joints in the input joint vector. Expected %d, "
             "got %zd "
             "joints",
             kinematics.GetNumberDegreesOfFreedom(), seed_q->rows());
  CHECK_GT(epsilon, 0.0) << "Epsilon must be positive and non-zero";

  double distance_to_target = std::numeric_limits<double>::max();

  std::size_t iterations = 0;
  State state(&kinematics);
  while (distance_to_target > epsilon && max_iterations > iterations++) {
    // The numerical convergence is drastically slowed down when we restrict to
    // the limits.
    CHECK_EQ(state.SetDofPositions(*seed_q, /*check_limits=*/false),
             intrinsic::icon::OkStatus());

    // Get current tip (ctip) pose wrt base
    INTRINSIC_RT_ASSIGN_OR_DIE(Pose3d base_pose_ctip,
                               state.GetTransform(tip_id));

    // Compute a cartesian velocity going from base_pose_ctip towards
    // base_pose_tip
    Vector6d ctip_V_tip =
        TransformToPoseVector(base_pose_ctip.inverse() * base_pose_tip);

    // Transform the velocity to the base frame
    Matrix6d base_W_ctip = Matrix6d::Zero();
    Matrix3d R_base_ctip = base_pose_ctip.rotationMatrix();
    base_W_ctip.block<3, 3>(0, 0) = R_base_ctip;
    base_W_ctip.block<3, 3>(3, 3) = R_base_ctip;
    ctip_V_tip = base_W_ctip * ctip_V_tip;
    distance_to_target = ctip_V_tip.norm();

    // Perform gradient descent.
    constexpr double kDampingCoefficient = 1e-06;
    INTRINSIC_RT_ASSIGN_OR_DIE(eigenmath::Matrix6Xd J, state.ComputeJacobian());
    VectorXd deltaq =
        dQPVelocityIK<>(*seed_q, J, kDampingCoefficient,
                        EmptyNullSpaceTaskFunction<VectorXd>, ctip_V_tip);

    // Setting adaptive gain, increases as we get closer to the goal
    double alpha = std::max(0.5, 1 - distance_to_target / 0.5);
    *seed_q = *seed_q + alpha * deltaq;
  }

  INTRINSIC_RT_ASSIGN_OR_DIE(
      bool is_within_limits,
      IsWithinLimitsAfterWrapping(kinematics, *seed_q,
                                  kinematics.GetDofSystemLimits()))

  return (distance_to_target <= epsilon && is_within_limits &&
          (!validator || validator(*seed_q)));
}

bool KinematicChainNewtonRaphsonIKSolver::IK(
    const Chain& kinematics, const Pose3d& base_pose_tip,
    const ElementId& tip_id, std::size_t max_iterations, double epsilon,
    absl::FunctionRef<bool(const eigenmath::VectorNd&)> validator,
    VectorNd& seed_q) {
  if (seed_q.rows() !=
      static_cast<int>(kinematics.GetNumberDegreesOfFreedom())) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Wrong number of joints in the input joint vector. Expected "
        << kinematics.GetNumberDegreesOfFreedom() << ", got " << seed_q.rows();
    return false;
  }
  if (epsilon <= 0.0) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Epsilon must be positive and non-zero";
    return false;
  }

  double distance_to_target = std::numeric_limits<double>::max();

  std::size_t iterations = 0;
  State state(&kinematics);
  while (distance_to_target > epsilon && max_iterations > iterations++) {
    // The numerical convergence is drastically slowed down when we restrict
    // to the limits.
    if (icon::RealtimeStatus s =
            state.SetDofPositions(seed_q, /*check_limits=*/false);
        !s.ok()) {
      INTRINSIC_RT_LOG_THROTTLED(ERROR)
          << "Failed to set dof positions: " << s.message();
      return false;
    }

    // Get current tip (ctip) pose wrt base
    icon::RealtimeStatusOr<Pose3d> base_pose_ctip = state.GetTransform(tip_id);
    if (!base_pose_ctip.ok()) {
      INTRINSIC_RT_LOG_THROTTLED(ERROR)
          << "Failed to get base_t_tip transform: "
          << base_pose_ctip.status().message();
      return false;
    }

    // Compute a cartesian velocity going from base_pose_ctip towards
    // base_pose_tip
    Vector6d ctip_V_tip =
        TransformToPoseVector(base_pose_ctip.value().inverse() * base_pose_tip);

    // Transform the velocity to the base frame
    Matrix6d base_W_ctip = Matrix6d::Zero();
    Matrix3d R_base_ctip = base_pose_ctip.value().rotationMatrix();
    base_W_ctip.block<3, 3>(0, 0) = R_base_ctip;
    base_W_ctip.block<3, 3>(3, 3) = R_base_ctip;
    ctip_V_tip = base_W_ctip * ctip_V_tip;
    distance_to_target = ctip_V_tip.norm();

    // Perform gradient descent.
    constexpr double kDampingCoefficient = 1e-06;
    icon::RealtimeStatusOr<eigenmath::Matrix6Nd> J = state.ComputeJacobian();
    if (!J.ok()) {
      INTRINSIC_RT_LOG_THROTTLED(ERROR)
          << "Failed to compute Jacobian: " << J.status().message();
      return false;
    }
    VectorNd deltaq =
        dQPVelocityIK<>(seed_q, *J, kDampingCoefficient,
                        EmptyNullSpaceTaskFunction<VectorNd>, ctip_V_tip);

    // Setting adaptive gain, increases as we get closer to the goal
    double alpha = std::max(0.5, 1 - distance_to_target / 0.5);
    seed_q += alpha * deltaq;
  }

  icon::RealtimeStatusOr<bool> is_within_limits = IsWithinLimitsAfterWrapping(
      kinematics, seed_q, kinematics.GetDofSystemLimits());
  if (!is_within_limits.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Failed to check IK solution for joint limits: "
        << is_within_limits.status().message();
    return false;
  }

  return (distance_to_target <= epsilon && *is_within_limits &&
          validator(seed_q));
}

IKResult KinematicChainRandomSeedIKSolver::compute(
    const Pose3d& base_pose_tip, const IKValidator& validator) const {
  std::default_random_engine engine(0);
  return compute(base_pose_tip, validator, &engine);
}

IKResult KinematicChainRandomSeedIKSolver::compute(
    const Pose3d& base_pose_tip, const IKValidator& validator,
    std::default_random_engine* engine) const {
  VectorXd inout_q = seed_q_;

  bool ik_found = KinematicChainRandomSeedIKSolver::IK(
      *kinematic_chain_, base_pose_tip, tip_id_, timeout_, max_iterations_,
      epsilon_, validator, max_num_ik_attempts_, engine, &inout_q);
  if (ik_found) {
    return {inout_q};
  }
  return {};
}

bool KinematicChainRandomSeedIKSolver::IK(
    const Chain& kinematics, const Pose3d& base_pose_tip,
    const ElementId& tip_id, absl::Duration timeout, std::size_t max_iterations,
    double epsilon, const IKValidator& validator,
    std::size_t max_num_ik_attempts, std::default_random_engine* engine,
    VectorXd* q) {
  int ndofs = static_cast<int>(kinematics.GetNumberDegreesOfFreedom());
  CHECK(q->rows() == 0 || q->rows() == ndofs) << absl::StrFormat(
      "Output vector must be either empty or a seed with the "
      "correct number of values. Expected zero of %d, got %zd "
      "joints",
      ndofs, q->rows());

  ClockSteady clock_steady;
  ElapsedTimer timer(&clock_steady);

  const JointLimits dof_limits = kinematics.GetDofSystemLimits();

  // Use an initial seed if it is provided as input. Random seed if not.
  VectorXd random_q = *q;
  if (random_q.rows() != ndofs) {
    random_q = GetUniformRandomConfiguration(dof_limits, *engine);
  }

  // Keep picking random seeds until found or the timeout or max attempts
  // are reached.
  std::size_t ik_attempt_count = 0;
  while (timer.Elapsed() < timeout &&
         (max_num_ik_attempts == 0 || ik_attempt_count < max_num_ik_attempts)) {
    if (KinematicChainNewtonRaphsonIKSolver::IK(kinematics, base_pose_tip,
                                                tip_id, max_iterations, epsilon,
                                                validator, &random_q)) {
      *q = random_q;
      return true;
    }
    random_q = GetUniformRandomConfiguration(dof_limits, *engine);
    ik_attempt_count++;
  }

  if (ik_attempt_count >= max_num_ik_attempts) {
    INTRINSIC_RT_LOG_THROTTLED(WARNING) << "IK reached max attempts.";
  } else {
    INTRINSIC_RT_LOG_THROTTLED(WARNING) << "IK timed out.";
  }

  return false;
}

}  // namespace kinematics
}  // namespace intrinsic
