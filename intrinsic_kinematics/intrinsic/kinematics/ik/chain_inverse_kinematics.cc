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

#include "intrinsic/kinematics/ik/chain_inverse_kinematics.h"

#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/ik_solvers/ik_solver_utils.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/damped_least_squares/kinematic_chain_damped_least_squares_ik_solver.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_newton_raphson_ik_solver.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_qp_ik_solver.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/util/status/status_macros.h"

// TODO(intrinsic-opx): Clean up to inject this setting through the blue
// stack via intrinsic::PartDescription and KinematicsConfigs.(b/145278077)
ABSL_FLAG(double, chain_ik_random_seed_solver_timeout_s, 0,
          "Optional timeout in seconds for the Newton-Raphson IK solver.");
ABSL_FLAG(std::string, chain_ik_solver, "NewtonRaphson",
          "The IK Solver to use ('NewtonRaphson', 'RT', 'QP')");

namespace intrinsic {
namespace kinematics {

namespace {

constexpr double kDefaultIkTimeoutSeconds = 5.0;

absl::Duration GetTimeout() {
  const double kTimeoutFlagSeconds =
      ::absl::GetFlag(FLAGS_chain_ik_random_seed_solver_timeout_s);
  if (kTimeoutFlagSeconds > 0) {
    return absl::Seconds(kTimeoutFlagSeconds);
  }
  return absl::Seconds(kDefaultIkTimeoutSeconds);
}

}  // namespace

absl::StatusOr<std::unique_ptr<ChainInverseKinematics>>
ChainInverseKinematics::Create(
    Chain chain, const InverseKinematicsInterface::Options& options) {
  auto solver = std::make_unique<ChainInverseKinematics>(std::move(chain));
  INTR_RETURN_IF_ERROR(solver->Init());
  return std::move(solver);
}

absl::Status ChainInverseKinematics::Init() {
  if (::absl::GetFlag(FLAGS_chain_ik_solver) == "NewtonRaphson") {
    auto ik_solver = std::make_unique<
        intrinsic::kinematics::KinematicChainRandomSeedIKSolver>(&chain());
    ik_solver->setTimeout(GetTimeout());
    ik_solver_ = std::move(ik_solver);
  } else if (::absl::GetFlag(FLAGS_chain_ik_solver) == "RT") {
    auto ik_solver = std::make_unique<
        intrinsic::kinematics::KinematicChainDampedLeastSquaresIKSolver>(
        &chain());
    ik_solver->setTimeout(GetTimeout());
    ik_solver_ = std::move(ik_solver);
  } else if (::absl::GetFlag(FLAGS_chain_ik_solver) == "QP") {
    // The unit test which exercises this codepath is flaky and therefore is
    // removed to fix b/231503842 and b/231503944.
    // TODO(b/231645705): Revive `chain_inverse_kinematics_qp_test` once the
    // Modular QP IK has landed (b/217739859).
    LOG(WARNING)
        << "The codepath with flag --chain_ik_solver=QP is flaky "
           "(b/231503842 and b/231503944; please see also b/231645705).";
    auto ik_solver =
        std::make_unique<intrinsic::kinematics::KinematicChainQPIKSolver>(
            &chain());
    ik_solver->setTimeout(GetTimeout());
    ik_solver_ = std::move(ik_solver);
  } else {
    return absl::InvalidArgumentError(
        absl::StrFormat("%s ik solver specified an unknown internal solver: %s",
                        GetName(), ::absl::GetFlag(FLAGS_chain_ik_solver)));
  }
  return absl::OkStatus();
}

int ChainInverseKinematics::GetNumDof() const {
  // The ChainKinematicsSolver supports any number of degrees of freedom.
  // Therefore, we return -1 here. In order to get the actual number of degrees
  // of freedom of the underlying chain, users are expected to query the chain
  // directly.
  return -1;
}

absl::StatusOr<InverseKinematicsInterface::IKResult>
ChainInverseKinematics::ComputeIK(const JointStateP& hint_joint_state,
                                  const Pose3d& desired_base_t_tip,
                                  const JointLimits& limits,
                                  absl::Span<JointStateP> solutions) const {
  if (!ik_solver_) {
    return absl::InternalError(
        "The solver has not been properly initialized. Did you call Init?");
  }
  const int num_requested_solutions = solutions.size();
  if (num_requested_solutions < 1) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "The solutions span needs to have size 1 or bigger, but it is: %d",
        num_requested_solutions));
  }

  if (hint_joint_state.size() != chain().GetNumberDegreesOfFreedom()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "The provided hint vector is of size %d but the chain has %d DOFs",
        hint_joint_state.size(), chain().GetNumberDegreesOfFreedom()));
  }

  INTRINSIC_ASSERT_NON_REALTIME();

  ik_solver_->setSeed(hint_joint_state.position);

  int num_solutions = 0;
  intrinsic::kinematics::IKValidator validator =
      [solutions, &num_solutions,
       limits](const intrinsic::eigenmath::VectorXd& q_test) {
        // Any solutions we have already seen we treat as invalid.
        for (const auto& value : solutions.first(num_solutions)) {
          if (q_test.isApprox(value.position)) {
            return false;
          }
        }

        // Solution should be within joint limits.
        JointStateP joint_state;
        joint_state.position = q_test;
        INTRINSIC_RT_ASSIGN_OR_DIE(auto limits_ok,
                                   IsWithinLimits(joint_state, limits));
        return limits_ok == true;
      };

  // We use a deterministic random number generator in order to ensure that the
  // solutions we return are consistent across calls and builds (see
  // b/162074965).
  std::default_random_engine gen(0);

  while (num_solutions < num_requested_solutions) {
    std::vector<intrinsic::eigenmath::VectorXd> solver_solutions =
        ik_solver_->compute(desired_base_t_tip, validator);

    // If no solution is found, it is very likely that the target pose is out of
    // reach.
    if (solver_solutions.empty()) {
      break;
    }

    // Copy valid solutions.
    for (int k = 0;
         k < solver_solutions.size() && num_solutions < num_requested_solutions;
         ++k) {
      JointStateP joint_state;
      joint_state.position = solver_solutions[k];

      INTRINSIC_RT_ASSIGN_OR_DIE(auto limits_ok,
                                 IsWithinLimits(joint_state, limits));
      if (!limits_ok) {
        continue;
      }

      solutions[num_solutions] = joint_state;
      num_solutions++;
    }

    if (num_solutions >= num_requested_solutions) {
      break;
    }

    // Obtain a solution from another seed.
    ik_solver_->setSeed(
        intrinsic::kinematics::GetUniformRandomConfiguration(chain(), gen));
  }

  const InverseKinematicsInterface::IKResult result = {
      .status = (num_solutions == 0 ? IKResult::NO_SOLUTION : IKResult::OK),
      .number_of_solutions = num_solutions};

  return result;
}

}  // namespace kinematics
}  // namespace intrinsic
