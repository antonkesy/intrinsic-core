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

#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_qp_ik_solver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <tuple>
#include <utility>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/random/random.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik_solvers/ik_solver_utils.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/kinematics/joint_wrapping.h"
#include "intrinsic/kinematics/math.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/time/clock_steady.h"
#include "intrinsic/util/time/elapsed_timer.h"
#include "osqp++.h"

namespace intrinsic {
namespace kinematics {

using eigenmath::MatrixXd;
using eigenmath::Vector6d;
using eigenmath::VectorXd;

KinematicChainQPIKSolver::KinematicChainQPIKSolver(const Chain* kinematic_chain)
    : KinematicChainIKSolver(kinematic_chain),
      seed_q_(VectorXd::Zero(kinematic_chain->GetNumberDegreesOfFreedom())),
      epsilon_(0.0001),
      max_iterations_(100) {
  tip_id_ = kinematic_chain->GetTipId();
}

void KinematicChainQPIKSolver::setSeed(const VectorXd& seed_q) {
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
      IsWithinLimitsAfterWrapping(*kinematic_chain_, seed_q_,
                                  kinematic_chain_->GetDofSystemLimits()))
  CHECK(is_within_limits) << "Seed must be within joint limits";
}

IKResult KinematicChainQPIKSolver::compute(const Pose3d& base_pose_tip,
                                           const IKValidator& validator) const {
  // TODO(b/427930380): add support for dependent joints.
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
  bool ik_found = IK(*kinematic_chain_, base_pose_tip, validator, &inout_q);
  if (ik_found) {
    return {inout_q};
  } else {
    INTRINSIC_RT_LOG(WARNING) << "No valid IK solution found.";
  }
  return {};
}

std::tuple<MatrixXd, VectorXd, double>
KinematicChainQPIKSolver::GetObjectiveMatrixAndVector(
    const Pose3d& base_pose_tip_desired, const VectorXd& q, double lm_damping,
    double dt) const {
  State state(kinematic_chain_);
  CHECK_EQ(state.SetDofPositions(q), intrinsic::icon::OkStatus());
  INTRINSIC_RT_ASSIGN_OR_DIE(const Pose3d& base_t_tip_current,
                             state.GetTransform(tip_id_));
  INTRINSIC_RT_ASSIGN_OR_DIE(eigenmath::Matrix6Nd J, state.ComputeJacobian());

  Vector6d pose_error =
      EvaluatePoseError(base_t_tip_current, base_pose_tip_desired);
  VLOG(2) << "pose_error: " << pose_error.transpose();
  double distance_to_target = (pose_error).norm();

  Vector6d r = pose_error / dt;

  double mu = lm_damping * std::max(1e-3, r.norm());

  MatrixXd P = J.transpose() * J + mu * MatrixXd::Identity(q.size(), q.size());
  VectorXd v = -r.transpose() * J;

  VLOG(2) << "P=\n" << P;
  VLOG(2) << "v=" << v.transpose();

  return std::make_tuple(P, v, distance_to_target);
}

// Implementation follows the description found at:
// https://scaron.info/teaching/inverse-kinematics.html
//
// Another related paper is:
// "Solvability-Unconcerned Inverse Kinematics by the Levenberg–Marquardt
// Method" by Tomomichi Sugihara DOI: 10.1109/TRO.2011.2148230
bool KinematicChainQPIKSolver::IK(const Chain& kinematics,
                                  const Pose3d& base_pose_tip,
                                  const IKValidator& validator,
                                  VectorXd* seed_q) const {
  CHECK_EQ(seed_q->size(),
           static_cast<int>(kinematics.GetNumberDegreesOfFreedom()))
      << absl::StrFormat(
             "Wrong number of joints in the input joint vector. Expected %d, "
             "got %zd "
             "joints",
             kinematics.GetNumberDegreesOfFreedom(), seed_q->size());
  CHECK_GT(epsilon_, 0.0) << "Epsilon must be positive and non-zero";

  size_t num_variables = seed_q->size();
  // The constraints are the joint limits on all joints.
  size_t num_constraints = num_variables;

  const JointLimits dof_limits = kinematics.GetDofSystemLimits();
  VLOG(2) << "q_lower=" << dof_limits.min_position.transpose();
  VLOG(2) << "q_upper=" << dof_limits.max_position.transpose();

  // Parameters to be set with meaningful values
  constexpr double improvement_stop = 1e-4;
  constexpr double starting_dt = 5e-2;
  constexpr size_t max_steps = 1e5;  // Should be set as a function of dt

  // A Levenberg-Marquardt damping factor. Seems to help performance. Might want
  // to explore turning it on or off as a function of the quality of the seed.
  constexpr double lm_damping = 1e-5;

  osqp::OsqpSolver solver;
  osqp::OsqpSettings settings;
  settings.verbose = false;
  // Disabling adaptive_rho makes QP solver deterministic. However it is
  // slowing down convergence.
  settings.adaptive_rho = true;

  osqp::OsqpInstance instance;

  absl::BitGen gen = GetRandomBitGenerator();

  auto solve_step = [&](VectorXd q, bool is_initialized,
                        double dt) -> absl::StatusOr<VectorXd> {
    auto objectives =
        GetObjectiveMatrixAndVector(base_pose_tip, q, lm_damping, dt);

    instance.objective_matrix = std::get<0>(objectives).sparseView();
    instance.objective_vector = std::get<1>(objectives).sparseView();
    const double distance_to_target = std::get<2>(objectives);

    // Since we are solving for a velocity vector, we need to bound the problem
    // so that the resulting position change should respect the limits.
    // Adding some slack to the bounds helps to solve the near joint limits.
    constexpr double kBoundSlack = 1.10;
    // We limits the bound so that the QP will solve from a fraction of the gap
    // between current and desired state.
    const double kBoundLimit = 1 / dt;
    instance.lower_bounds =
        VectorXd::Constant(num_variables, -kBoundLimit)
            .cwiseMax((kBoundSlack * dof_limits.min_position - q) / dt);
    instance.upper_bounds =
        VectorXd::Constant(num_variables, kBoundLimit)
            .cwiseMin((kBoundSlack * dof_limits.max_position - q) / dt);

    VLOG(2) << "lower bounds: " << instance.lower_bounds.transpose();
    VLOG(2) << "upper bounds: " << instance.upper_bounds.transpose();

    if (!is_initialized) {
      instance.constraint_matrix =
          MatrixXd::Identity(num_variables, num_variables).sparseView();
      CHECK_OK(solver.Init(instance, settings));
    } else {
      CHECK_OK(solver.SetObjectiveVector(instance.objective_vector));
      instance.objective_matrix = std::get<0>(objectives).sparseView();
      CHECK_OK(solver.UpdateObjectiveMatrix(instance.objective_matrix));
      // It has been observed that using a warm start from a previous solve
      // slowed down the solve. So we here reset the state of the solver after
      // updating the objective matrix. More investigation should be performed
      // to clearly understand the effect of the warm start state on the
      // performance of the solve.
      CHECK_OK(solver.SetWarmStart(VectorXd::Zero(num_variables),
                                   VectorXd::Zero(num_constraints)));

      CHECK_OK(solver.SetBounds(instance.lower_bounds, instance.upper_bounds));
    }

    osqp::OsqpExitCode exit_code = solver.Solve();

    if (exit_code != osqp::OsqpExitCode::kOptimal &&
        exit_code != osqp::OsqpExitCode::kOptimalInaccurate) {
      return absl::InternalError(absl::StrCat(
          "Failed to solve the IK problem: ", osqp::ToString(exit_code)));
    }

    VectorXd qd = solver.primal_solution();
    VectorXd delta_q = qd * dt;

    VLOG(2) << "dist: " << distance_to_target << ", dt: " << dt;
    VLOG(2) << "dq: " << delta_q.transpose();

    q += delta_q;

    // Clip new q to position limits
    q = q.cwiseMax(dof_limits.min_position);
    q = q.cwiseMin(dof_limits.max_position);

    return q;
  };

  ClockSteady clock_steady;
  ElapsedTimer timer(&clock_steady);

  VectorXd q_current = *seed_q;
  VLOG(1) << "Starting to solve with seed: " << toString(q_current);

  size_t iterations = 0;

  State state_current(kinematic_chain_);

  // Limit the number of time the problem is reinitialized with a random seed.
  while (max_iterations_ > iterations++) {
    size_t steps_count = 0;
    double dt = starting_dt;
    bool solution_found = false;
    double prev_improvement = -1;

    double prev_target_error = std::numeric_limits<double>::max();
    double target_error = std::numeric_limits<double>::max();

    while (true) {
      auto step_status = solve_step(q_current, steps_count > 1, dt);

      if (!step_status.ok()) {
        LOG(ERROR) << "Failed to solve QP step: " << step_status.status();
        break;
      }
      q_current = step_status.value();

      CHECK_EQ(state_current.SetDofPositions(q_current),
               intrinsic::icon::OkStatus());
      INTRINSIC_RT_ASSIGN_OR_DIE(const Pose3d& base_t_tip_current,
                                 state_current.GetTransform(tip_id_));

      target_error =
          EvaluatePoseError(base_pose_tip, base_t_tip_current).norm();

      const JointLimits dof_limits = state_current.GetDofLimits();

      VLOG(2) << "q_lower=" << dof_limits.min_position.transpose();
      VLOG(2) << "q=" << q_current.transpose();
      VLOG(2) << "q_upper=" << dof_limits.max_position.transpose();

      double improvement =
          (target_error - prev_target_error) / prev_target_error;
      prev_target_error = target_error;

      VLOG(2) << "improvement: " << improvement
              << ", prev_improvement: " << prev_improvement << ", dt=" << dt;
      if (fabs(improvement) < improvement_stop) {
        VLOG(1) << "Stop improving, error=" << target_error
                << ", improvement=" << improvement;
        solution_found = target_error < epsilon_;
        break;
      }

      if (signbit(improvement) != signbit(prev_improvement) &&
          steps_count > 0 && !signbit(improvement)) {
        dt /= 2.0;
        VLOG(2) << "Improvement sign changed, new dt=" << dt;
        if (dt < 1e-8) {
          VLOG(1) << "Stop improving, oscillating.";
          // Usually the case for infeasible solution.
          break;
        }
      }
      prev_improvement = improvement;

      if (target_error < epsilon_) {
        solution_found = true;
        break;
      }

      if (timer.Elapsed() > timeout_) {
        VLOG(1) << "Failed to find a solution: timed out after "
                << absl::ToInt64Milliseconds(timer.Elapsed()) << "ms";
        return false;
      }

      if (max_steps <= steps_count++) {
        VLOG(1) << "Failed to find a solution: max steps reached.";
        break;
      }
    }

    if (!(validator ? validator(q_current) : true)) {
      VLOG(1) << "Failed to find a solution: out of limits or invalid.";
      solution_found = false;
    }

    INTRINSIC_RT_ASSIGN_OR_DIE(
        bool is_within_limits,
        IsWithinLimitsAfterWrapping(*kinematic_chain_, q_current,
                                    kinematic_chain_->GetDofSystemLimits()))

    if (!is_within_limits) {
      VLOG(1) << "Failed to find a solution: out of limits after unwind: "
              << toString(q_current);
      solution_found = false;
    }

    if (solution_found) {
      VLOG(1) << "Solution found in " << steps_count << " steps after "
              << iterations << " iterations";
      VLOG(2) << "q=" << q_current.transpose();
      break;
    }

    q_current = GetUniformRandomConfiguration(kinematics, gen);
    VLOG(1) << "No solution found, trying again with new seed: "
            << toString(q_current);
  }

  if (iterations >= max_iterations_ + 1) {
    VLOG(1)
        << "Failed to find a solution: max number of seeded attempt reached.";
    return false;
  }

  *seed_q = q_current;
  return true;
}

}  // namespace kinematics
}  // namespace intrinsic
