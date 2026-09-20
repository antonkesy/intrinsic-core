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

#include "intrinsic/icon/control/algorithms/speed_override_factor_limits_utils.h"

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"

namespace intrinsic::icon {

namespace {

// A tiny regularization value.
const double kMinusculeRegularizer = 1.0e-13;

// Computes the scalar lower and upper limits for scalar variable `x` in an
// N-dimensional equation of the form:
//   lhs <= coeff * x <= rhs
// where `lhs` (left-hand side), `rhs` (right-hand side) and `coeff` are
// N-dimensional vectors. It returns the most constraining limit for `x` from
// all the N-dimensional limits. The vectors `lhs`, `rhs` and `coeff` should be
// of the same size.
template <typename DerivedLhs, typename DerivedRhs, typename DerivedCoeff>
RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair> ComputeLimitPair(
    const Eigen::MatrixBase<DerivedLhs>& lhs,
    const Eigen::MatrixBase<DerivedRhs>& rhs,
    const Eigen::MatrixBase<DerivedCoeff>& coeff) {
  const int ndofs = lhs.size();
  if (ndofs != rhs.size() || ndofs != coeff.size()) {
    return InvalidArgumentError(
        "The vectors `lhs`, `rhs` and `coeff` must have the same size.");
  }

  SpeedOverrideFactorLimits::LimitPair limit_pair;
  for (int dof_id = 0; dof_id < ndofs; ++dof_id) {
    // If the coefficient is zero, we skip the computation of the limit, as it
    // will be plus or minus infinity, which are the least constraining limits
    // and the `limit_pair` is already initialized to plus or minus infinity.
    const double coeff_val = coeff[dof_id];
    if (AlmostEquals(coeff_val, 0.0)) {
      continue;
    }
    const double regularized_coeff =
        (coeff_val >= 0.0) ? std::max(coeff_val, kMinusculeRegularizer)
                           : std::min(coeff_val, -kMinusculeRegularizer);

    const double lower_limit =
        ((coeff_val > 0.0) ? lhs[dof_id] : rhs[dof_id]) / regularized_coeff;
    const double upper_limit =
        ((coeff_val > 0.0) ? rhs[dof_id] : lhs[dof_id]) / regularized_coeff;
    limit_pair.lower = std::max(limit_pair.lower, lower_limit);
    limit_pair.upper = std::min(limit_pair.upper, upper_limit);
  }
  return limit_pair;
}

}  // namespace

RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair>
ComputeSpeedOverrideFactorLimits(const JointStatePVA& state,
                                 const JointLimits& joint_limits,
                                 bool clamp_sof_to_zero_one_range) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      SpeedOverrideFactorLimits::LimitPair limit_pair,
      ComputeLimitPair(-joint_limits.max_velocity, joint_limits.max_velocity,
                       state.velocity));

  if (clamp_sof_to_zero_one_range) {
    limit_pair.lower = std::max(limit_pair.lower, kMinimumSpeedOverrideFactor);
    limit_pair.upper = std::min(limit_pair.upper, kMaximumSpeedOverrideFactor);
  }
  return limit_pair;
}

RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair>
ComputeTimeDerivativeOfSpeedOverrideFactorLimitsForAccLimitedTraj(
    const JointStatePVA& state, const JointLimits& joint_limits,
    const double speed_override_factor) {
  const eigenmath::VectorNd& qpp = state.acceleration;
  const double sof2 = ::intrinsic::IPow(speed_override_factor, 2);
  const auto qpp_sof2 = qpp * sof2;

  return ComputeLimitPair(-joint_limits.max_acceleration - qpp_sof2,
                          joint_limits.max_acceleration - qpp_sof2,
                          state.velocity);
}

RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair>
ComputeTimeDerivativeOfSpeedOverrideFactorLimitsForTorqueLimitedTraj(
    const JointStatePVA& state, const JointLimits& joint_limits,
    const double speed_override_factor, icon::RigidBodyInterface& dynamics) {
  // In the first place, we evaluate the components of the equations of motion
  // at the current joint configuration `q` and speed override parameterized
  // joint velocity `qd`.
  const eigenmath::VectorNd& q = state.position;
  const eigenmath::VectorNd& qp = state.velocity;
  const eigenmath::VectorNd qd = qp * speed_override_factor;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const eigenmath::VectorNd generalized_gravity_vector,
      dynamics.ComputeGeneralizedGravityVector(q));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const eigenmath::VectorNd coriolis_vector,
                                dynamics.ComputeCoriolisVector(q, qd));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const eigenmath::MatrixNd joint_space_inertia_matrix,
      dynamics.ComputeJointSpaceInertiaMatrix(q));

  // The equations of motion as a function of the speed override factor (sof)
  // and its time derivative (dsof/dt) take the following form:
  // torque = M(q) * (qpp * sof^2 + qp * dsof/dt) + C(q,qd) * sof^2 + G(q)
  // and the torque is box limited as in -t_max <= torque <= t_max.
  // With this in mind, we can group terms that do not contain dsof/dt, as in:
  // t_bar = (M(q) * qpp + C(q,qd)) * sof^2 + G(q).
  const eigenmath::VectorNd& qpp = state.acceleration;
  const double sof2 = ::intrinsic::IPow(speed_override_factor, 2);
  const eigenmath::VectorNd t_bar =
      (joint_space_inertia_matrix * qpp + coriolis_vector) * sof2 +
      generalized_gravity_vector;
  // The terms that contain dsof/dt, can also be grouped as in M(q) * qp,
  // including a minuscule regularization term to avoid divisions by zero.
  const eigenmath::VectorNd jsim_qp = joint_space_inertia_matrix * qp;

  return ComputeLimitPair(-joint_limits.max_torque - t_bar,
                          joint_limits.max_torque - t_bar, jsim_qp);
}

RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair>
ComputeSecondTimeDerivativeOfSpeedOverrideFactorLimits(
    const JointStatePVAJ& state, const JointLimits& joint_limits,
    const double speed_override_factor,
    const double time_derivative_speed_override_factor) {
  const eigenmath::VectorNd& qpp = state.acceleration;
  const eigenmath::VectorNd& qppp = state.jerk;
  const double& sof = speed_override_factor;
  const double& dsof_dt = time_derivative_speed_override_factor;
  const double sof3 = ::intrinsic::IPow(speed_override_factor, 3);
  const auto j_offset = qppp * sof3 + 3.0 * qpp * dsof_dt * sof;

  return ComputeLimitPair(-joint_limits.max_jerk - j_offset,
                          joint_limits.max_jerk - j_offset, state.velocity);
}

}  // namespace intrinsic::icon
