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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_UTILS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_UTILS_H_

#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

// Valid speed override factor minimum and maximum range values.
static constexpr double kMinimumSpeedOverrideFactor = 0.0;
static constexpr double kMaximumSpeedOverrideFactor = 1.0;

// Computes scalar lower and upper limits (min_sof, max_sof) of the speed
// override factor `sof`. These scalar limits (min_sof, max_sof) are implied by
// the N-dimensional lower and upper limits of the joint velocities, by
// selecting the most constraining limit.
// The joint velocity `qd` should be in the range [-max_vel, max_vel] as in:
//   -max_vel      <=     qd     <= max_vel
// Parameterizing the trajectory in terms of the speed override factor `sof`,
// leads to `qd` = `qp` * `sof`, where `qp` is the vector of joint velocities
// from the current `state`. One can interpret, `qp` as a planned joint velocity
// vector and `qd` as the actual velocity vector due to the speed override
// factor `sof`.
//   -max_vel <= qp * sof <= max_vel
// Then, the limits for the speed override factor are given by:
//   (-max_vel / qp)[dof_id] <= sof <= (max_vel / qp)[dof_id] for all dof_id.
// Note that from the last inequalities, this function returns only the most
// constraining limit for `sof` from all the DOFs.
RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair>
ComputeSpeedOverrideFactorLimits(const JointStatePVA& state,
                                 const JointLimits& joint_limits,
                                 bool clamp_sof_to_zero_one_range);

// Computes the scalar lower and upper limits of the time derivative of the
// speed override factor (dsof_dt) for an acceleration limited trajectory. These
// scalar limits on dsof_dt (min_dsof_dt, max_dsof_dt) are implied by the
// N-dimensional lower and upper limits for joint accelerations, by
// selecting the most constraining limit.
// The joint acceleration `qdd` should be in the range [-max_acc, max_acc]:
//   -max_acc <= qdd <= max_acc
// Parameterizing the trajectory in terms of the speed override factor `sof`,
// leads to `qdd` = qpp * sof^2 + qp * dsof/dt, where `qp`, `qpp` are the
// vectors of joint velocities, accelerations from the current `state`. One can
// interpret, `qpp` as a planned joint acceleration vector and `qdd` as the
// actual acceleration vector due to the speed override factor dynamics `sof`
// and its time derivative `dsof/dt`.
//   -max_acc <= qpp * sof^2 + qp * dsof/dt <= max_acc
//   -max_acc - qpp * sof^2 <= qp * dsof/dt <= max_acc - qpp * sof^2
// Then, the limits for the speed override factor first time derivative are:
//  ((-max_acc - qpp * sof^2) / qp)[dof_id] <=
//       dsof/dt <= ((max_acc - qpp * sof^2) / qp)[dof_id] for all dof_id.
// Note that from the last inequalities, this function returns only the most
// constraining limit for `dsof_dt` from all the DOFs.
RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair>
ComputeTimeDerivativeOfSpeedOverrideFactorLimitsForAccLimitedTraj(
    const JointStatePVA& state, const JointLimits& joint_limits,
    const double speed_override_factor);

// Computes the scalar lower and upper limits of the first time derivative of
// the speed override factor with respect to time (dsof/dt) based on a dynamics
// model. These limits (min_dsof_dt, max_dsof_dt) for the time derivative of the
// speed override factor are implied by the lower and upper limits of the joint
// torque `trq` (-max_trq, max_trq).
//   -max_trq <= trq <= max_trq
// The joint torque `trq` is in general given by:
//    trq = M(q) * qdd + C(q,qd) + G(q)
// where M(q) is the joint space inertia matrix, C(q,qd) is the Coriolis vector
// and G(q) is the generalized gravity vector. q, qd, qdd are the joint
// position, velocity and acceleration vectors. q, qp, qpp are the planned joint
// position, velocity and acceleration vectors from the current `state`, which
// are being reparameterized based on the speed override factor. In terms of the
// dynamics of the speed override factor, it can be rewritten as:
//    trq = M(q) * (qpp * sof^2 + qp * dsof/dt) + C(q,qp * sof) + G(q)
//    trq = (M(q) * qpp + C(q,qd)) * sof^2 + G(q) + M(q) * qp * dsof/dt
// which for simplicity can be formulated as:
//    trq = t_bar + jsim_qp * dsof/dt
// The limits for `dsof/dt` are then given by:
//   -max_trq <= t_bar + jsim_qp * dsof/dt <= max_trq
//   ((-max_trq - t_bar) / jsim_qp)[dof_id] <=
//       dsof/dt <= ((max_trq - t_bar) / jsim_qp)[dof_id] for all dof_id.
// Note that from the last inequalities, this function returns only the most
// constraining limit for `dsof/dt` from all the DOFs.
RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair>
ComputeTimeDerivativeOfSpeedOverrideFactorLimitsForTorqueLimitedTraj(
    const JointStatePVA& state, const JointLimits& joint_limits,
    const double speed_override_factor, icon::RigidBodyInterface& dynamics);

// Computes the scalar lower and upper limits of the second time derivative of
// the speed override factor with respect to time (d2sof/dt2). These limits
// (min_d2sof_dt2, max_d2sof_dt2) for the second time derivative of the speed
// override factor are implied by the lower and upper limits of the joint jerk
// `qddd` (-max_jerk, max_jerk).
//   -max_jerk <= qddd <= max_jerk
// The actual jerk `qddd` in terms of the speed override factor derivatives
// (sof, dsof/dt, d2sof/dt2) and planned joint velocities, accelerations and
// jerks (qp, qpp, qppp from the current `state`) can be formulated as follows:
//   qddd = qppp * sof^3 + 3 * qpp * dsof/dt * sof + qp * d2sof/dt2
// where for simplicity, we can rewrite the first term as:
//   qddd = j_offset + qp * d2sof/dt2
// where j_offset = qppp * sof^3 + 3 * qpp * dsof/dt * sof.
// The limits for the second time derivative of the speed override factor are
// then given by:
//   -max_jerk <= j_offset + qp * d2sof/dt2 <= max_jerk
//   -max_jerk - j_offset <= qp * d2sof/dt2 <= max_jerk - j_offset
//   ((-max_jerk - j_offset)/qp)[dof_id] <=
//       d2sof/dt2 <= ((max_jerk - j_offset)/qp)[dof_id] for all dof_id.
// Note that from the last inequalities, this function returns only the most
// constraining limit for `d2sof/dt2` from all the DOFs.
RealtimeStatusOr<SpeedOverrideFactorLimits::LimitPair>
ComputeSecondTimeDerivativeOfSpeedOverrideFactorLimits(
    const JointStatePVAJ& state, const JointLimits& joint_limits,
    const double speed_override_factor,
    const double time_derivative_speed_override_factor);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_UTILS_H_
