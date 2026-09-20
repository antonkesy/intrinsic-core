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

#include "intrinsic/math/spline/polynomial_spline.h"

#include <algorithm>
#include <array>
#include <cstddef>

#include "absl/functional/function_ref.h"
#include "intrinsic/eigenmath/rotation_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/state_rn.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace {

// Several structs holding the translation and rotation states as needed for
// this spline implementation. The rotation state contains the angular velocity
// and the angular acceleration vectors expressed in the local (tangent) frame
// of the quaternion.
struct LocalPolynomialStatePVA {
  StateRnPVA translation3d_state;
  // The QuaternionState struct is specialized as it uses angular velocity and
  // acceleration in the local (tangent) frame of the quaternion.
  QuaternionStateQVA quat_state_with_local_derivatives;
};

struct LocalPolynomialStatePV {
  StateRnPV translation3d_state;
  // The QuaternionState struct is specialized as it uses angular velocity and
  // acceleration in the local (tangent) frame of the quaternion.
  QuaternionStateQV quat_state_with_local_derivatives;
};

struct LocalPolynomialStateP {
  StateRnP translation3d_state;
  // The QuaternionState struct is specialized as it uses angular velocity and
  // acceleration in the local (tangent) frame of the quaternion.
  QuaternionStateQ quat_state_with_local_derivatives;
};

struct LocalPolynomialStateV {
  StateRnV translation3d_state;
  // The QuaternionState struct is specialized as it uses angular velocity and
  // acceleration in the local (tangent) frame of the quaternion.
  QuaternionStateV quat_state_with_local_derivatives;
};

// Returns the `time_eval_seconds` value clamped to the minimum and maximum
// limits. These minimum and maximum time limits are given by `0` and
// `time_horizon_seconds` with an allowed numerical precision limit of
// `kEpsilonTimeDurationSeconds`. Returns an InvalidArgumentError, if the input
// value is beyond these limits.
icon::RealtimeStatusOr<double> ClampTimeEvaluationSeconds(
    double time_eval_seconds, double time_horizon_seconds) {
  const double min_time =
      -polynomial_spline_internal::kEpsilonTimeDurationSeconds;
  const double max_time =
      time_horizon_seconds +
      polynomial_spline_internal::kEpsilonTimeDurationSeconds;
  if ((time_eval_seconds < min_time) || (time_eval_seconds > max_time)) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "time_eval_seconds must be in the range [", min_time, ",", max_time,
        "], but got ", time_eval_seconds, "."));
  }
  return std::clamp(time_eval_seconds, 0.0, time_horizon_seconds);
}

// Returns the array of size `MaxPower` composed of the powers of the input `t`.
template <size_t MaxPower>
std::array<double, MaxPower> ComputePowersOf(double t) {
  std::array<double, MaxPower> powers{{1}};
  for (size_t i = 1; i < MaxPower; ++i) {
    powers[i] = powers[i - 1] * t;
  }
  return powers;
}

// Returns the interpolated StateRnPVA defined by the polynomial given by the
// input coefficients `c` at the desired evaluation time `time_eval_seconds`.
template <size_t NumCoeffs>
icon::RealtimeStatusOr<StateRnPVA> InterpolatePVAState(
    const std::array<eigenmath::VectorNd, NumCoeffs>& coefficients,
    double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // powers of t: t^i; here t = time_eval_seconds, considering the polynomial
  // start time as the 0 seconds.
  std::array<double, NumCoeffs> powers_of_t =
      ComputePowersOf<NumCoeffs>(time_eval_seconds);

  StateRnPVA interpolated_state;
  eigenmath::VectorNd delta_position =
      eigenmath::VectorNd::Zero(coefficients[0].rows());
  interpolated_state.velocity =
      eigenmath::VectorNd::Zero(coefficients[1].rows());
  interpolated_state.acceleration =
      eigenmath::VectorNd::Zero(coefficients[2].rows());
  for (int i = 1; i < NumCoeffs; ++i) {
    delta_position += (coefficients[i] * powers_of_t[i]);
    interpolated_state.velocity +=
        static_cast<double>(i) * coefficients[i] * powers_of_t[i - 1];
    if (i >= 2) {
      interpolated_state.acceleration += static_cast<double>(i * (i - 1)) *
                                         coefficients[i] * powers_of_t[i - 2];
    }
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(interpolated_state.position,
                                euler_integration_step_compute_function(
                                    coefficients[0], delta_position, 1.0));
  return interpolated_state;
}

// Returns the interpolated StateRnPVAJ defined by the polynomial given by the
// input coefficients `c` at the desired evaluation time `time_eval_seconds`.
template <size_t NumCoeffs>
icon::RealtimeStatusOr<StateRnPVAJ> InterpolatePVAJState(
    const std::array<eigenmath::VectorNd, NumCoeffs>& coefficients,
    double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // powers of t: t^i; here t = time_eval_seconds, considering the polynomial
  // start time as the 0 seconds.
  std::array<double, NumCoeffs> powers_of_t =
      ComputePowersOf<NumCoeffs>(time_eval_seconds);

  StateRnPVAJ interpolated_state;
  eigenmath::VectorNd delta_position =
      eigenmath::VectorNd::Zero(coefficients[0].rows());
  interpolated_state.velocity =
      eigenmath::VectorNd::Zero(coefficients[1].rows());
  interpolated_state.acceleration =
      eigenmath::VectorNd::Zero(coefficients[2].rows());
  interpolated_state.jerk = eigenmath::VectorNd::Zero(coefficients[3].rows());
  for (int i = 1; i < NumCoeffs; ++i) {
    delta_position += (coefficients[i] * powers_of_t[i]);
    interpolated_state.velocity +=
        static_cast<double>(i) * coefficients[i] * powers_of_t[i - 1];
    if (i >= 2) {
      interpolated_state.acceleration += static_cast<double>(i * (i - 1)) *
                                         coefficients[i] * powers_of_t[i - 2];
    }
    if (i >= 3) {
      interpolated_state.jerk += static_cast<double>(i * (i - 1) * (i - 2)) *
                                 coefficients[i] * powers_of_t[i - 3];
    }
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(interpolated_state.position,
                                euler_integration_step_compute_function(
                                    coefficients[0], delta_position, 1.0));
  return interpolated_state;
}

QuaternionStateQVA FromStateRnToQuaternionState(const StateRnPVA& state) {
  eigenmath::AngleAxisd angle_axis =
      eigenmath::AngleTimesAxisToAngleAxis<double>(state.position);
  QuaternionStateQVA quat_state;
  quat_state.quaternion = eigenmath::Quaterniond(angle_axis);
  quat_state.angular_velocity = state.velocity;
  quat_state.angular_acceleration = state.acceleration;
  return quat_state;
}

StateRnPVA FromQuaternionStateToStateRn(const QuaternionStateQVA& quat_state) {
  eigenmath::AngleAxisd angle_axis =  // NOLINT: <modernize-use-auto>
      eigenmath::AngleAxisd(quat_state.quaternion);
  StateRnPVA state;
  state.position = eigenmath::AngleAxisToAngleTimesAxis<double>(angle_axis);
  state.velocity = quat_state.angular_velocity;
  state.acceleration = quat_state.angular_acceleration;
  return state;
}

StateRnPV FromQuaternionStateToStateRn(const QuaternionStateQV& quat_state) {
  eigenmath::AngleAxisd angle_axis =  // NOLINT: <modernize-use-auto>
      eigenmath::AngleAxisd(quat_state.quaternion);
  StateRnPV state;
  state.position = eigenmath::AngleAxisToAngleTimesAxis<double>(angle_axis);
  state.velocity = quat_state.angular_velocity;
  return state;
}

StateRnP FromQuaternionStateToStateRn(const QuaternionStateQ& quat_state) {
  eigenmath::AngleAxisd angle_axis =  // NOLINT: <modernize-use-auto>
      eigenmath::AngleAxisd(quat_state.quaternion);
  StateRnP state;
  state.position = eigenmath::AngleAxisToAngleTimesAxis<double>(angle_axis);
  return state;
}

StateRnV FromQuaternionStateToStateRn(const QuaternionStateV& quat_state) {
  StateRnV state;
  state.velocity = quat_state.angular_velocity;
  return state;
}

// `cart_state` is split into a translation component `translation3d_state`
// and a rotation component `quat_state` in the `lp_state` struct. In order
// to apply splines to orientations, `quat_state` needs to have its angular
// velocity and acceleration represented in the local (tangent) frame of the
// quaternion, which requires a rotation matrix `R` that is computed from the
// `reference_state`.
icon::RealtimeStatusOr<LocalPolynomialStatePVA> CartStateToLocalCoordinates(
    const CartStatePVA& reference_state, const CartStatePVA& cart_state) {
  LocalPolynomialStatePVA lp_state;
  INTRINSIC_RT_RETURN_IF_ERROR(lp_state.translation3d_state.SetSize(3));

  // The translational part doesn't change.
  lp_state.translation3d_state.position = cart_state.pose.translation();
  lp_state.translation3d_state.velocity = cart_state.velocity.head<3>();
  lp_state.translation3d_state.acceleration = cart_state.acceleration.head<3>();
  // The rotation required to rotate angular velocity and acceleration to the
  // local frame (representing the tangent space) of the quaternion to perform
  // integration on the manifold.
  const eigenmath::Matrix3d R = reference_state.pose.rotationMatrix();
  lp_state.quat_state_with_local_derivatives.quaternion =
      cart_state.pose.quaternion();
  lp_state.quat_state_with_local_derivatives.angular_velocity =
      R.transpose() * cart_state.velocity.tail<3>();
  lp_state.quat_state_with_local_derivatives.angular_acceleration =
      R.transpose() * cart_state.acceleration.tail<3>();

  return lp_state;
}

icon::RealtimeStatusOr<LocalPolynomialStatePV> CartStateToLocalCoordinates(
    const CartStatePVA& reference_state, const CartStatePV& cart_state) {
  LocalPolynomialStatePV lp_state;
  INTRINSIC_RT_RETURN_IF_ERROR(lp_state.translation3d_state.SetSize(3));

  // The translational part doesn't change.
  lp_state.translation3d_state.position = cart_state.pose.translation();
  lp_state.translation3d_state.velocity = cart_state.velocity.head<3>();
  // The rotation required to rotate angular velocity and acceleration to the
  // local frame (representing the tangent space) of the quaternion to perform
  // integration on the manifold.
  const eigenmath::Matrix3d R = reference_state.pose.rotationMatrix();
  lp_state.quat_state_with_local_derivatives.quaternion =
      cart_state.pose.quaternion();
  lp_state.quat_state_with_local_derivatives.angular_velocity =
      R.transpose() * cart_state.velocity.tail<3>();

  return lp_state;
}

icon::RealtimeStatusOr<LocalPolynomialStateV> CartStateToLocalCoordinates(
    const CartStatePVA& reference_state, const CartStateV& cart_state) {
  LocalPolynomialStateV lp_state;
  INTRINSIC_RT_RETURN_IF_ERROR(lp_state.translation3d_state.SetSize(3));

  // The translational part doesn't change.
  lp_state.translation3d_state.velocity = cart_state.velocity.head<3>();
  // The rotation required to rotate angular velocity and acceleration to the
  // local frame (representing the tangent space) of the quaternion to perform
  // integration on the manifold.
  const eigenmath::Matrix3d R = reference_state.pose.rotationMatrix();
  lp_state.quat_state_with_local_derivatives.angular_velocity =
      R.transpose() * cart_state.velocity.tail<3>();

  return lp_state;
}

icon::RealtimeStatusOr<LocalPolynomialStateP> CartStateToLocalCoordinates(
    const CartStatePVA& reference_state, const CartStateP& cart_state) {
  LocalPolynomialStateP lp_state;
  INTRINSIC_RT_RETURN_IF_ERROR(lp_state.translation3d_state.SetSize(3));

  // The translational part doesn't change.
  lp_state.translation3d_state.position = cart_state.pose.translation();
  // The rotational part also doesn't change.
  lp_state.quat_state_with_local_derivatives.quaternion =
      cart_state.pose.quaternion();

  return lp_state;
}

// `translation3d_state` and `quat_state` of the `lp_state` struct are converted
// back to the global representation `cart_state`. As angular velocity and
// acceleration of `quat_state` are represented in the local (tangent) frame of
// the quaternion, they need to be rotated back to the global frame using the
// rotation matrix `R` computed from the `reference_state`.
CartStatePVA LocalCoordinatesToCartState(
    const CartStatePVA& reference_state,
    const LocalPolynomialStatePVA& lp_state) {
  CartStatePVA cart_state;

  // The translational part doesn't change.
  cart_state.pose.translation() = lp_state.translation3d_state.position;
  cart_state.velocity.head<3>() = lp_state.translation3d_state.velocity;
  cart_state.acceleration.head<3>() = lp_state.translation3d_state.acceleration;
  // Rotation requires to rotate angular velocity and acceleration from the
  // local frame of the quaternion back to the global frame.
  const eigenmath::Matrix3d R = reference_state.pose.rotationMatrix();
  cart_state.pose.setQuaternion(
      lp_state.quat_state_with_local_derivatives.quaternion);
  cart_state.velocity.tail<3>() =
      R * lp_state.quat_state_with_local_derivatives.angular_velocity;
  cart_state.acceleration.tail<3>() =
      R * lp_state.quat_state_with_local_derivatives.angular_acceleration;

  return cart_state;
}

// Returns the coefficients to interpolate between the `start_state` and
// `target_state` in a time interval given by `time_horizon_seconds`. The
// function
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`
// measures the minimum geodesic distance between `start_state` and
// `target_state` positions. This implementation constructs the quadratic
// polynomial coefficients such that they preserve position continuity and keep
// the acceleration of the first input constant throughout the interval.
// Velocity information is irrelevant here and disregarded.
icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 3>>
ComputeQuadraticPolynomialCoefficients(
    const StateRnPA& start_state, const StateRnP& target_state,
    double time_horizon_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function) {
  // Short names for brevity:
  const auto& x = start_state.position;
  const auto& g = target_state.position;
  const auto& xdd = start_state.acceleration;

  eigenmath::VectorNd tangent_vector_minimizing_geodesic_distance_to_target;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      tangent_vector_minimizing_geodesic_distance_to_target,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function(x, g));

  // Computes the power basis constants:
  const double tau = time_horizon_seconds;
  std::array<eigenmath::VectorNd, 3> coefficients;
  coefficients[0] = x;
  coefficients[1] =
      tangent_vector_minimizing_geodesic_distance_to_target / tau -
      xdd / 2.0 * tau;
  coefficients[2] = xdd / 2.0;

  return coefficients;
}

// Returns the coefficients to interpolate between the `start_state` and
// `target_state` in a time interval given by `time_horizon_seconds`. This
// implementation constructs the cubic polynomial coefficients with position,
// velocity, and acceleration constraints at the start_state and
// only a position constraint at the target_state.
icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 4>>
ComputeCubicSplineCoefficientsWithOnlyTargetPosition(
    const StateRnPVA& start_state, const StateRnP& target_state,
    double time_horizon_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function) {
  // Powers of tau: tau^i; tau is the short name of time_horizon_seconds,
  // which is the time from start to reach the target.
  std::array<double, 4> powers_of_tau =
      ComputePowersOf<4>(time_horizon_seconds);

  // Short names for brevity:
  const eigenmath::VectorNd& x = start_state.position;
  const eigenmath::VectorNd& xd = start_state.velocity;
  const eigenmath::VectorNd& xdd = start_state.acceleration;
  const eigenmath::VectorNd& g = target_state.position;

  eigenmath::VectorNd tangent_vector_minimizing_geodesic_distance_to_target;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      tangent_vector_minimizing_geodesic_distance_to_target,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function(x, g));

  // Computes the power basis constants:
  // (Note: coefficients were auto-generated with Mathematica and verified)
  std::array<eigenmath::VectorNd, 4> coefficients;
  coefficients[0] = x;
  coefficients[1] = xd;
  coefficients[2] = 0.5 * xdd;
  coefficients[3] =
      -0.5 *
      (-2.0 * tangent_vector_minimizing_geodesic_distance_to_target +
       2.0 * powers_of_tau[1] * xd + powers_of_tau[2] * xdd) /
      powers_of_tau[3];

  return coefficients;
}

// Returns the coefficients to interpolate between the `start_state` and
// `target_state` in a time interval given by `time_horizon_seconds`. This
// implementation constructs the cubic polynomial coefficients with position,
// velocity, and acceleration constraints at the start_state and
// only a velocity constraint at the target_state.
icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 4>>
ComputeCubicSplineCoefficientsWithOnlyTargetVelocity(
    const StateRnPVA& start_state, const StateRnV& target_state,
    double time_horizon_seconds) {
  // Powers of tau: tau^i; tau is the short name of time_horizon_seconds,
  // which is the time from start to reach the target.
  std::array<double, 4> powers_of_tau =
      ComputePowersOf<4>(time_horizon_seconds);

  // Short names for brevity:
  const eigenmath::VectorNd& x = start_state.position;
  const eigenmath::VectorNd& xd = start_state.velocity;
  const eigenmath::VectorNd& xdd = start_state.acceleration;
  const eigenmath::VectorNd& gd = target_state.velocity;

  // Computes the power basis constants:
  // (Note: coefficients were auto-generated with Mathematica and verified)
  std::array<eigenmath::VectorNd, 4> coefficients;
  coefficients[0] = x;
  coefficients[1] = xd;
  coefficients[2] = 0.5 * xdd;
  coefficients[3] =
      -1.0 / 3.0 * (-gd + xd + powers_of_tau[1] * xdd) / powers_of_tau[2];

  return coefficients;
}

// Returns the coefficients to interpolate between the `start_state` and
// `target_state` in a time interval given by `time_horizon_seconds`. The
// function
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`
// measures the minimum geodesic distance between `start_state` and
// `target_state` positions. This implementation constructs the cubic polynomial
// coefficients such that they preserve position and velocity continuity.
icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 4>>
ComputeCubicSplineCoefficientsWithVelocityContinuity(
    const StateRnPV& start_state, const StateRnPV& target_state,
    double time_horizon_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function) {
  // Powers of tau: tau^i; tau is the short name of time_horizon_seconds,
  // which is the time from start to reach the target.
  std::array<double, 4> powers_of_tau =
      ComputePowersOf<4>(time_horizon_seconds);

  // Short names for brevity:
  const auto& x = start_state.position;
  const auto& xd = start_state.velocity;
  const auto& g = target_state.position;
  const auto& gd = target_state.velocity;

  eigenmath::VectorNd tangent_vector_minimizing_geodesic_distance_to_target;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      tangent_vector_minimizing_geodesic_distance_to_target,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function(x, g));

  // Computes the power basis constants:
  std::array<eigenmath::VectorNd, 4> coefficients;
  coefficients[0] = x;
  coefficients[1] = xd;
  coefficients[2] = 3.0 *
                        (tangent_vector_minimizing_geodesic_distance_to_target -
                         xd * powers_of_tau[1]) /
                        powers_of_tau[2] -
                    (gd - xd) / powers_of_tau[1];
  coefficients[3] =
      ((gd - xd) / powers_of_tau[1] -
       2.0 *
           (tangent_vector_minimizing_geodesic_distance_to_target -
            xd * powers_of_tau[1]) /
           powers_of_tau[2]) /
      powers_of_tau[1];

  return coefficients;
}

// Returns the coefficients to interpolate between the `start_state` and
// `target_state` in a time interval given by `time_horizon_seconds`. The
// function
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`
// measures the minimum geodesic distance between `start_state` and
// `target_state` positions. This implementation constructs the cubic polynomial
// coefficients such that they preserve position and acceleration continuity.
// Velocity information is irrelevant here and disregarded.
icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 4>>
ComputeCubicSplineCoefficientsWithAccelerationContinuity(
    const StateRnPA& start_state, const StateRnPA& target_state,
    double time_horizon_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function) {
  // Short names for brevity:
  const auto& x = start_state.position;
  const auto& g = target_state.position;
  const auto& xdd = start_state.acceleration;
  const auto& gdd = target_state.acceleration;

  eigenmath::VectorNd tangent_vector_minimizing_geodesic_distance_to_target;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      tangent_vector_minimizing_geodesic_distance_to_target,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function(x, g));

  // Computes the power basis constants:
  const double tau = time_horizon_seconds;
  std::array<eigenmath::VectorNd, 4> coefficients;
  coefficients[0] = x;
  coefficients[1] =
      tangent_vector_minimizing_geodesic_distance_to_target / tau -
      (xdd * tau / 2.0 + (gdd - xdd) * tau / 6.0);
  coefficients[2] = xdd / 2.0;
  coefficients[3] = (gdd - xdd) / (6.0 * tau);

  return coefficients;
}

icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 5>>
ComputeQuarticSplineCoefficients(
    const StateRnPVA& start_state, const StateRnPV& target_state,
    double time_horizon_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function) {
  // Powers of tau: tau^i; tau is the short name of time_horizon_seconds,
  // which is the time from start to reach the target.
  std::array<double, 5> powers_of_tau =
      ComputePowersOf<5>(time_horizon_seconds);

  // Short names for brevity. Note that gddd, the jerk at the goal, is set to
  // zero as a boundary condition.
  const eigenmath::VectorNd& x = start_state.position;
  const eigenmath::VectorNd& xd = start_state.velocity;
  const eigenmath::VectorNd& xdd = start_state.acceleration;
  const eigenmath::VectorNd& g = target_state.position;
  const eigenmath::VectorNd& gd = target_state.velocity;

  eigenmath::VectorNd tangent_vector_minimizing_geodesic_distance_to_target;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      tangent_vector_minimizing_geodesic_distance_to_target,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function(x, g));

  // Computes the power basis constants (solution was auto-generated with
  // Mathematica and verified):
  std::array<eigenmath::VectorNd, 5> coefficients;
  coefficients[0] = x;
  coefficients[1] = xd;
  coefficients[2] = 0.5 * xdd;
  coefficients[3] =
      -((-4.0 * tangent_vector_minimizing_geodesic_distance_to_target +
         gd * powers_of_tau[1]) +
        3.0 * powers_of_tau[1] * xd + powers_of_tau[2] * xdd) /
      powers_of_tau[3];
  coefficients[4] =
      -0.5 *
      (6.0 * tangent_vector_minimizing_geodesic_distance_to_target -
       2.0 * gd * powers_of_tau[1] - 4.0 * powers_of_tau[1] * xd -
       powers_of_tau[2] * xdd) /
      powers_of_tau[4];

  return coefficients;
}

icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 6>>
ComputeQuinticSplineCoefficients(
    const StateRnPVA& start_state, const StateRnPVA& target_state,
    double time_horizon_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function) {
  // Powers of tau: tau^i; tau is the short name of time_horizon_seconds,
  // which is the time from start to reach the target.
  std::array<double, 6> powers_of_tau =
      ComputePowersOf<6>(time_horizon_seconds);

  // Short names for brevity:
  const auto& x = start_state.position;
  const auto& xd = start_state.velocity;
  const auto& xdd = start_state.acceleration;
  const auto& g = target_state.position;
  const auto& gd = target_state.velocity;
  const auto& gdd = target_state.acceleration;

  eigenmath::VectorNd tangent_vector_minimizing_geodesic_distance_to_target;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      tangent_vector_minimizing_geodesic_distance_to_target,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function(x, g));

  // Computes the power basis constants:
  std::array<eigenmath::VectorNd, 6> coefficients;
  coefficients[0] = x;
  coefficients[1] = xd;
  coefficients[2] = 0.5 * xdd;
  coefficients[3] = 0.5 * (gdd - (3.0 * xdd)) / powers_of_tau[1] -
                    ((4.0 * gd) + (6.0 * xd)) / powers_of_tau[2] +
                    10.0 *
                        tangent_vector_minimizing_geodesic_distance_to_target /
                        powers_of_tau[3];
  coefficients[4] = -0.5 * ((2 * gdd) - (3 * xdd)) / powers_of_tau[2] +
                    (7.0 * gd + 8.0 * xd) / powers_of_tau[3] -
                    15.0 *
                        tangent_vector_minimizing_geodesic_distance_to_target /
                        powers_of_tau[4];
  coefficients[5] =
      0.5 * ((gdd - xdd) / powers_of_tau[3]) -
      3.0 * ((gd + xd) / powers_of_tau[4]) +
      6.0 * (tangent_vector_minimizing_geodesic_distance_to_target /
             powers_of_tau[5]);

  return coefficients;
}

icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 8>>
ComputeSepticSplineCoefficients(
    const StateRnPVAJ& start_state, const StateRnPVAJ& target_state,
    double time_horizon_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function) {
  // powers of tau: tau^i; tau is the short name of time_horizon_seconds,
  // which is the time from start to reach the target.
  std::array<double, 8> powers_of_tau =
      ComputePowersOf<8>(time_horizon_seconds);

  // short names for brevity:
  const auto& x = start_state.position;
  const auto& xd = start_state.velocity;
  const auto& xdd = start_state.acceleration;
  const auto& xddd = start_state.jerk;
  const auto& g = target_state.position;
  const auto& gd = target_state.velocity;
  const auto& gdd = target_state.acceleration;
  const auto& gddd = target_state.jerk;

  eigenmath::VectorNd tangent_vector_minimizing_geodesic_distance_to_target;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      tangent_vector_minimizing_geodesic_distance_to_target,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function(x, g));

  // Computes the power basis constants:
  std::array<eigenmath::VectorNd, 8> coefficients;
  coefficients[0] = x;
  coefficients[1] = xd;
  coefficients[2] = xdd / 2.0;
  coefficients[3] = xddd / 6.0;
  coefficients[4] =
      (210.0 * tangent_vector_minimizing_geodesic_distance_to_target -
       120.0 * powers_of_tau[1] * xd - 90.0 * powers_of_tau[1] * gd -
       30.0 * xdd * powers_of_tau[2] + 15.0 * gdd * powers_of_tau[2] -
       4.0 * powers_of_tau[3] * xddd - powers_of_tau[3] * gddd) /
      (6.0 * powers_of_tau[4]);
  coefficients[5] =
      (-168.0 * tangent_vector_minimizing_geodesic_distance_to_target +
       90.0 * powers_of_tau[1] * xd + 78.0 * powers_of_tau[1] * gd +
       20.0 * xdd * powers_of_tau[2] - 14.0 * gdd * powers_of_tau[2] +
       2.0 * powers_of_tau[3] * xddd + powers_of_tau[3] * gddd) /
      (2.0 * powers_of_tau[5]);
  coefficients[6] =
      -(-420 * tangent_vector_minimizing_geodesic_distance_to_target +
        216 * powers_of_tau[1] * xd + 204 * powers_of_tau[1] * gd +
        45 * xdd * powers_of_tau[2] - 39 * gdd * powers_of_tau[2] +
        4 * powers_of_tau[3] * xddd + 3 * powers_of_tau[3] * gddd) /
      (6 * powers_of_tau[6]);
  coefficients[7] =
      (-120 * tangent_vector_minimizing_geodesic_distance_to_target +
       60 * powers_of_tau[1] * xd + 60 * powers_of_tau[1] * gd +
       12 * xdd * powers_of_tau[2] - 12 * gdd * powers_of_tau[2] +
       powers_of_tau[3] * xddd + powers_of_tau[3] * gddd) /
      (6 * powers_of_tau[7]);

  return coefficients;
}

// The interpolation function using Quadratic (2nd-degree) Spline. Given the
// inputs of `start_state`, `target_state`, `time_horizon_seconds`,
// `time_eval_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the interpolated
// state at `time_eval_seconds` as the output, such that when
// `time_eval_seconds` = 0, the interpolated state will be equal to
// `start_state` (position) when `time_eval_seconds` =
// `time_horizon_seconds`, the interpolated state will be equal to
// `target_state` (position). Acceleration of the `start_state` is kept constant
// throughout the interval. Fails when the condition 0 <= `time_eval_seconds` <=
// `time_horizon_seconds` is not satisfied, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
icon::RealtimeStatusOr<StateRnPVA> EvaluateQuadraticSpline(
    const StateRnPA& start_state, const StateRnP& target_state,
    double time_horizon_seconds, double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      time_eval_seconds,
      ClampTimeEvaluationSeconds(time_eval_seconds, time_horizon_seconds));

  // Computes the power basis constants:
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto coefficients,
      ComputeQuadraticPolynomialCoefficients(
          start_state, target_state, time_horizon_seconds,
          tangent_vector_minimizing_geodesic_dist_to_target_compute_function));

  return InterpolatePVAState(coefficients, time_eval_seconds,
                             euler_integration_step_compute_function);
}

// The interpolation function using Cubic (3rd-degree) Spline with only a target
// position constraint. Given the inputs of `start_state`, `target_state`,
// `time_horizon_seconds`, `time_eval_seconds`, and
// `euler_integration_step_compute_function`, computes the interpolated state at
// `time_eval_seconds` as the output, such that when `time_eval_seconds` = 0,
// the interpolated state will be equal to `start_state` (position, velocity,
// acceleration), and when `time_eval_seconds` = `time_horizon_secon…`
// (velocity). Fails when the condition 0 <= `time_eval_seconds` <=
// `time_horizon_seconds` is not satisfied, or if
// `euler_integration_step_compute_function` fail.
icon::RealtimeStatusOr<StateRnPVA> EvaluateCubicSplineWithOnlyTargetPosition(
    const StateRnPVA& start_state, const StateRnP& target_state,
    double time_horizon_seconds, double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      time_eval_seconds,
      ClampTimeEvaluationSeconds(time_eval_seconds, time_horizon_seconds));

  // Computes the power basis constants:
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto coefficients,
      ComputeCubicSplineCoefficientsWithOnlyTargetPosition(
          start_state, target_state, time_horizon_seconds,
          tangent_vector_minimizing_geodesic_dist_to_target_compute_function));

  return InterpolatePVAState(coefficients, time_eval_seconds,
                             euler_integration_step_compute_function);
}

// The interpolation function using Cubic (3rd-degree) Spline with only a target
// velocity constraint. Given the inputs of `start_state`, `target_state`,
// `time_horizon_seconds`, `time_eval_seconds`, and
// `euler_integration_step_compute_function`, computes the interpolated state at
// `time_eval_seconds` as the output, such that when `time_eval_seconds` = 0,
// the interpolated state will be equal to `start_state` (position, velocity,
// acceleration), and when `time_eval_seconds` = `time_horizon_seconds`, the
// interpolated state will be equal to `target_state` (velocity). Fails when the
// condition 0 <= `time_eval_seconds` <= `time_horizon_seconds` is not
// satisfied, or if `euler_integration_step_compute_function` fail.
icon::RealtimeStatusOr<StateRnPVA> EvaluateCubicSplineWithOnlyTargetVelocity(
    const StateRnPVA& start_state, const StateRnV& target_state,
    double time_horizon_seconds, double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      time_eval_seconds,
      ClampTimeEvaluationSeconds(time_eval_seconds, time_horizon_seconds));

  // Computes the power basis constants:
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto coefficients, ComputeCubicSplineCoefficientsWithOnlyTargetVelocity(
                             start_state, target_state, time_horizon_seconds));

  return InterpolatePVAState(coefficients, time_eval_seconds,
                             euler_integration_step_compute_function);
}

// The interpolation function using Cubic (3rd-degree) Spline that preserves
// velocity continuity. Given the inputs of `start_state`, `target_state`,
// `time_horizon_seconds`, `time_eval_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the interpolated
// state at `time_eval_seconds` as the output, such that when
// `time_eval_seconds` = 0, the interpolated state will be equal to
// `start_state` (position and velocity) when `time_eval_seconds` =
// `time_horizon_seconds`, the interpolated state will be equal to
// `target_state` (position and velocity). Fails when the condition 0 <=
// `time_eval_seconds` <= `time_horizon_seconds` is not satisfied, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
icon::RealtimeStatusOr<StateRnPVAJ> EvaluateCubicSplineWithVelocityContinuity(
    const StateRnPV& start_state, const StateRnPV& target_state,
    double time_horizon_seconds, double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      time_eval_seconds,
      ClampTimeEvaluationSeconds(time_eval_seconds, time_horizon_seconds));

  // Computes the power basis constants:
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto coefficients,
      ComputeCubicSplineCoefficientsWithVelocityContinuity(
          start_state, target_state, time_horizon_seconds,
          tangent_vector_minimizing_geodesic_dist_to_target_compute_function));

  return InterpolatePVAJState(coefficients, time_eval_seconds,
                              euler_integration_step_compute_function);
}

// The interpolation function using Cubic (3rd-degree) Spline that preserves
// acceleration continuity. Given the inputs of `start_state`, `target_state`,
// `time_horizon_seconds`, `time_eval_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the interpolated
// state at `time_eval_seconds` as the output, such that when
// `time_eval_seconds` = 0, the interpolated state will be equal to
// `start_state` (position and acceleration) when `time_eval_seconds` =
// `time_horizon_seconds`, the interpolated state will be equal to
// `target_state` (position and acceleration). Fails when the condition 0 <=
// `time_eval_seconds` <= `time_horizon_seconds` is not satisfied, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
icon::RealtimeStatusOr<StateRnPVA>
EvaluateCubicSplineWithAccelerationContinuity(
    const StateRnPA& start_state, const StateRnPA& target_state,
    double time_horizon_seconds, double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      time_eval_seconds,
      ClampTimeEvaluationSeconds(time_eval_seconds, time_horizon_seconds));

  // Computes the power basis constants:
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto coefficients,
      ComputeCubicSplineCoefficientsWithAccelerationContinuity(
          start_state, target_state, time_horizon_seconds,
          tangent_vector_minimizing_geodesic_dist_to_target_compute_function));

  return InterpolatePVAState(coefficients, time_eval_seconds,
                             euler_integration_step_compute_function);
}

// The interpolation function using Quartic (4th-degree) Spline. Given the
// inputs of `start_state`, `target_state`, `time_horizon_seconds`,
// `time_eval_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`, and
// `euler_integration_step_compute_function`, computes the interpolated state at
// `time_eval_seconds` as the output, such that when `time_eval_seconds` = 0,
// the interpolated state will be equal to `start_state` when
// `time_eval_seconds` = `time_horizon_seconds`, the interpolated state will be
// equal to `target_state`. Fails when the condition 0 <= `time_eval_seconds` <=
// `time_horizon_seconds` is not satisfied, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
icon::RealtimeStatusOr<StateRnPVA> EvaluateQuarticSpline(
    const StateRnPVA& start_state, const StateRnPV& target_state,
    double time_horizon_seconds, double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      time_eval_seconds,
      ClampTimeEvaluationSeconds(time_eval_seconds, time_horizon_seconds));

  // Computes the power basis constants:
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto coefficients,
      ComputeQuarticSplineCoefficients(
          start_state, target_state, time_horizon_seconds,
          tangent_vector_minimizing_geodesic_dist_to_target_compute_function));

  return InterpolatePVAState(coefficients, time_eval_seconds,
                             euler_integration_step_compute_function);
}

// The interpolation function using Quintic (5th-degree) Spline. Given the
// inputs of `start_state`, `target_state`, `time_horizon_seconds`,
// `time_eval_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the interpolated
// state at `time_eval_seconds` as the output, such that when
// `time_eval_seconds` = 0, the interpolated state will be equal to
// `start_state` when `time_eval_seconds` = `time_horizon_seconds`, the
// interpolated state will be equal to `target_state`. Fails when the
// condition 0 <= `time_eval_seconds` <= `time_horizon_seconds` is not
// satisfied, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
icon::RealtimeStatusOr<StateRnPVAJ> EvaluateQuinticSpline(
    const StateRnPVA& start_state, const StateRnPVA& target_state,
    double time_horizon_seconds, double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      time_eval_seconds,
      ClampTimeEvaluationSeconds(time_eval_seconds, time_horizon_seconds));

  // Computes the power basis constants:
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto coefficients,
      ComputeQuinticSplineCoefficients(
          start_state, target_state, time_horizon_seconds,
          tangent_vector_minimizing_geodesic_dist_to_target_compute_function));

  return InterpolatePVAJState(coefficients, time_eval_seconds,
                              euler_integration_step_compute_function);
}
// The interpolation function using Septic (7th-degree) Spline. Given the inputs
// of `start_state`, `target_state`, `time_horizon_seconds`,
// `time_eval_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the interpolated
// state at `time_eval_seconds` as the output, such that when
// `time_eval_seconds` = 0, the interpolated state will be equal to
// `start_state` when `time_eval_seconds` = `time_horizon_seconds`, the
// interpolated state will be equal to `target_state`. Fails when the
// condition 0 <= `time_eval_seconds` <= `time_horizon_seconds` is not
// satisfied, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
icon::RealtimeStatusOr<StateRnPVAJ> EvaluateSepticSpline(
    const StateRnPVAJ& start_state, const StateRnPVAJ& target_state,
    double time_horizon_seconds, double time_eval_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      time_eval_seconds,
      ClampTimeEvaluationSeconds(time_eval_seconds, time_horizon_seconds));

  // Computes the power basis constants:
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto coefficients,
      ComputeSepticSplineCoefficients(
          start_state, target_state, time_horizon_seconds,
          tangent_vector_minimizing_geodesic_dist_to_target_compute_function));

  return InterpolatePVAJState(coefficients, time_eval_seconds,
                              euler_integration_step_compute_function);
}

// Quadratic (2nd-degree) Spline-specific version of `GetNextState()`. Given the
// inputs of `current_state`, `target_state`, `time_to_go_seconds`,
// `dt_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the next state as
// the output, such that when this method is called iteratively in a loop and
// the `time_to_go_seconds` is decremented by `dt_seconds` in each iteration,
// the next state will be equal to `target_state` when `time_to_go_seconds`
// drops to 0 (at the final iteration). Fails when `dt_seconds` <
// `kMinimumDtSeconds`, or `time_to_go_seconds` < `dt_seconds`, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
// Therefore the highest control frequency supported is 1/`kMinimumDtSeconds`.
icon::RealtimeStatusOr<StateRnPVA> GetNextStateQuadraticSpline(
    const StateRnPA& current_state, const StateRnP& target_state,
    double time_to_go_seconds, double dt_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // The following checks are necessary to avoid division-by-zero error.
  if (dt_seconds < polynomial_spline_internal::kMinimumDtSeconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateQuadraticSpline(): dt_seconds (", dt_seconds,
        ") must be >= ", polynomial_spline_internal::kMinimumDtSeconds));
  }
  if (time_to_go_seconds < dt_seconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateQuadraticSpline(): time_to_go_seconds (",
        time_to_go_seconds, ") must be >= dt_seconds (", dt_seconds, ")"));
  }

  return EvaluateQuadraticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
      euler_integration_step_compute_function);
}

// Cubic (3rd-degree) Spline-specific version of `GetNextState()` with only a
// target position as boundary condition. Given the inputs of `current_state`,
// `target_state`, `time_to_go_seconds`, `dt_seconds`, and
// `euler_integration_step_compute_function`, computes the next state as the
// output, such that when this method is called iteratively in a loop and the
// `time_to_go_seconds` is decremented by `dt_seconds` in each iteration, the
// next state will be equal to `target_state` when `time_to_go_seconds` drops to
// 0 (at the final iteration). Fails when `dt_seconds` < `kMinimumDtSeconds`, or
// `time_to_go_seconds` < `dt_seconds`, or
// `euler_integration_step_compute_function` fails.
// Therefore the highest control frequency supported is 1/`kMinimumDtSeconds`.
icon::RealtimeStatusOr<StateRnPVA>
GetNextStateCubicSplineWithOnlyTargetPosition(
    const StateRnPVA& current_state, const StateRnP& target_state,
    double time_to_go_seconds, double dt_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // The following checks are necessary to avoid division-by-zero error.
  if (dt_seconds < polynomial_spline_internal::kMinimumDtSeconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateCubicSplineWithOnlyTargetPosition(): dt_seconds (",
        dt_seconds,
        ") must be >= ", polynomial_spline_internal::kMinimumDtSeconds));
  }
  if (time_to_go_seconds < dt_seconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateCubicSplineWithOnlyTargetPosition(): "
        "time_to_go_seconds (",
        time_to_go_seconds, ") must be >= dt_seconds (", dt_seconds, ")"));
  }

  return EvaluateCubicSplineWithOnlyTargetPosition(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
      euler_integration_step_compute_function);
}

// Cubic (3rd-degree) Spline-specific version of `GetNextState()` with only a
// target velocity as boundary condition. Given the inputs of `current_state`,
// `target_state`, `time_to_go_seconds`, `dt_seconds`, and
// `euler_integration_step_compute_function`, computes the next state as the
// output, such that when this method is called iteratively in a loop and the
// `time_to_go_seconds` is decremented by `dt_seconds` in each iteration, the
// next state will be equal to `target_state` when `time_to_go_seconds` drops to
// 0 (at the final iteration). Fails when `dt_seconds` < `kMinimumDtSeconds`, or
// `time_to_go_seconds` < `dt_seconds`, or
// `euler_integration_step_compute_function` fails.
// Therefore the highest control frequency supported is 1/`kMinimumDtSeconds`.
icon::RealtimeStatusOr<StateRnPVA>
GetNextStateCubicSplineWithOnlyTargetVelocity(
    const StateRnPVA& current_state, const StateRnV& target_state,
    double time_to_go_seconds, double dt_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // The following checks are necessary to avoid division-by-zero error.
  if (dt_seconds < polynomial_spline_internal::kMinimumDtSeconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateCubicSplineWithOnlyTargetVelocity(): dt_seconds (",
        dt_seconds,
        ") must be >= ", polynomial_spline_internal::kMinimumDtSeconds));
  }
  if (time_to_go_seconds < dt_seconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateCubicSplineWithOnlyTargetVelocity(): "
        "time_to_go_seconds (",
        time_to_go_seconds, ") must be >= dt_seconds (", dt_seconds, ")"));
  }

  return EvaluateCubicSplineWithOnlyTargetVelocity(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      euler_integration_step_compute_function);
}

// Cubic (3rd-degree) Spline-specific version of `GetNextState()` that preserves
// velocity continuity. Given the inputs of `current_state`, `target_state`,
// `time_to_go_seconds`, `dt_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the next state as
// the output, such that when this method is called iteratively in a loop and
// the `time_to_go_seconds` is decremented by `dt_seconds` in each iteration,
// the next state will be equal to `target_state` when `time_to_go_seconds`
// drops to 0 (at the final iteration). Fails when `dt_seconds` <
// `kMinimumDtSeconds`, or `time_to_go_seconds` < `dt_seconds`, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
// Therefore the highest control frequency supported is 1/`kMinimumDtSeconds`.
icon::RealtimeStatusOr<StateRnPVAJ>
GetNextStateCubicSplineWithVelocityContinuity(
    const StateRnPV& current_state, const StateRnPV& target_state,
    double time_to_go_seconds, double dt_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // The following checks are necessary to avoid division-by-zero error.
  if (dt_seconds < polynomial_spline_internal::kMinimumDtSeconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateCubicSplineWithVelocityContinuity(): dt_seconds (",
        dt_seconds,
        ") must be >= ", polynomial_spline_internal::kMinimumDtSeconds));
  }
  if (time_to_go_seconds < dt_seconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateCubicSplineWithVelocityContinuity(): time_to_go_seconds (",
        time_to_go_seconds, ") must be >= dt_seconds (", dt_seconds, ")"));
  }

  return EvaluateCubicSplineWithVelocityContinuity(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
      euler_integration_step_compute_function);
}

// Cubic (3rd-degree) Spline-specific version of `GetNextState()` that preserves
// acceleration continuity. Given the inputs of `current_state`, `target_state`,
// `time_to_go_seconds`, `dt_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the next state as
// the output, such that when this method is called iteratively in a loop and
// the `time_to_go_seconds` is decremented by `dt_seconds` in each iteration,
// the next state will be equal to `target_state` when `time_to_go_seconds`
// drops to 0 (at the final iteration). Fails when `dt_seconds` <
// `kMinimumDtSeconds`, or `time_to_go_seconds` < `dt_seconds`, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
// Therefore the highest control frequency supported is 1/`kMinimumDtSeconds`.
icon::RealtimeStatusOr<StateRnPVA>
GetNextStateCubicSplineWithAccelerationContinuity(
    const StateRnPA& current_state, const StateRnPA& target_state,
    double time_to_go_seconds, double dt_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // The following checks are necessary to avoid division-by-zero error.
  if (dt_seconds < polynomial_spline_internal::kMinimumDtSeconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateCubicSplineWithAccelerationContinuity(): dt_seconds (",
        dt_seconds,
        ") must be >= ", polynomial_spline_internal::kMinimumDtSeconds));
  }
  if (time_to_go_seconds < dt_seconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateCubicSplineWithAccelerationContinuity(): "
        "time_to_go_seconds (",
        time_to_go_seconds, ") must be >= dt_seconds (", dt_seconds, ")"));
  }

  return EvaluateCubicSplineWithAccelerationContinuity(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
      euler_integration_step_compute_function);
}

// Quartic (4th-degree) Spline-specific variant of `GetNextState()`. Given the
// inputs of `current_state`, `target_state`, `time_to_go_seconds`,
// `dt_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`, and
// `euler_integration_step_compute_function`, computes the next state as the
// output, such that when this method is called iteratively in a loop and the
// `time_to_go_seconds` is decremented by `dt_seconds` in each iteration, the
// next state will be equal to `target_state` when `time_to_go_seconds` drops to
// 0 (at the final iteration). Fails when `dt_seconds` < `kMinimumDtSeconds`, or
// `time_to_go_seconds` < `dt_seconds`, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
// Therefore the highest control frequency supported is 1/`kMinimumDtSeconds`.
icon::RealtimeStatusOr<StateRnPVA> GetNextStateQuarticSpline(
    const StateRnPVA& current_state, const StateRnPV& target_state,
    double time_to_go_seconds, double dt_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // The following checks are necessary to avoid division-by-zero error.
  if (dt_seconds < polynomial_spline_internal::kMinimumDtSeconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateQuarticSpline(): dt_seconds (", dt_seconds,
        ") must be >= ", polynomial_spline_internal::kMinimumDtSeconds));
  }
  if (time_to_go_seconds < dt_seconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateQuarticSpline(): time_to_go_seconds (", time_to_go_seconds,
        ") must be >= dt_seconds (", dt_seconds, ")"));
  }

  return EvaluateQuarticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
      euler_integration_step_compute_function);
}

// Quintic (5th-degree) Spline-specific version of `GetNextState()`. Given the
// inputs of `current_state`, `target_state`, `time_to_go_seconds`,
// `dt_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the next state as
// the output, such that when this method is called iteratively in a loop and
// the `time_to_go_seconds` is decremented by `dt_seconds` in each iteration,
// the next state will be equal to `target_state` when `time_to_go_seconds`
// drops to 0 (at the final iteration). Fails when `dt_seconds` <
// `kMinimumDtSeconds`, or `time_to_go_seconds` < `dt_seconds`, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
// Therefore the highest control frequency supported is 1/`kMinimumDtSeconds`.
icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateQuinticSpline(
    const StateRnPVA& current_state, const StateRnPVA& target_state,
    double time_to_go_seconds, double dt_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // The following checks are necessary to avoid division-by-zero error.
  if (dt_seconds < polynomial_spline_internal::kMinimumDtSeconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateQuinticSpline(): dt_seconds (", dt_seconds,
        ") must be >= ", polynomial_spline_internal::kMinimumDtSeconds));
  }
  if (time_to_go_seconds < dt_seconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateQuinticSpline(): time_to_go_seconds (", time_to_go_seconds,
        ") must be >= dt_seconds (", dt_seconds, ")"));
  }

  return EvaluateQuinticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
      euler_integration_step_compute_function);
}

// Septic (7th-degree) Spline-specific version of `GetNextState()`. Given the
// inputs of `current_state`, `target_state`, `time_to_go_seconds`,
// `dt_seconds`,
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function`,
// and `euler_integration_step_compute_function`, computes the next state as
// the output, such that when this method is called iteratively in a loop and
// the `time_to_go_seconds` is decremented by `dt_seconds` in each iteration,
// the next state will be equal to `target_state` when `time_to_go_seconds`
// drops to 0 (at the final iteration). Fails when `dt_seconds` <
// `kMinimumDtSeconds`, or `time_to_go_seconds` < `dt_seconds`, or if
// `tangent_vector_minimizing_geodesic_dist_to_target_compute_function` or
// `euler_integration_step_compute_function` fail.
// Therefore the highest control frequency supported is 1/`kMinimumDtSeconds`.
icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateSepticSpline(
    const StateRnPVAJ& current_state, const StateRnPVAJ& target_state,
    double time_to_go_seconds, double dt_seconds,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&)>&
        tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
    const absl::FunctionRef<icon::RealtimeStatusOr<eigenmath::VectorNd>(
        const eigenmath::VectorNd&, const eigenmath::VectorNd&, double)>&
        euler_integration_step_compute_function) {
  // The following checks are necessary to avoid division-by-zero error.
  if (dt_seconds < polynomial_spline_internal::kMinimumDtSeconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateSepticSpline(): dt_seconds (", dt_seconds,
        ") must be >= ", polynomial_spline_internal::kMinimumDtSeconds));
  }
  if (time_to_go_seconds < dt_seconds) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "GetNextStateSepticSpline(): time_to_go_seconds (", time_to_go_seconds,
        ") must be >= dt_seconds (", dt_seconds, ")"));
  }

  return EvaluateSepticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      tangent_vector_minimizing_geodesic_dist_to_target_compute_function,
      euler_integration_step_compute_function);
}

icon::RealtimeStatusOr<StateRnPVAJ> EvaluateEuclideanSepticSpline(
    const StateRnPVAJ& start_state, const StateRnPVAJ& target_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return EvaluateSepticSpline(
      start_state, target_state, time_horizon_seconds, time_eval_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA>
GetNextStateEuclideanCubicSplineWithOnlyTargetPosition(
    const StateRnPVA& current_state, const StateRnP& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateCubicSplineWithOnlyTargetPosition(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA>
GetNextStateEuclideanCubicSplineWithOnlyTargetVelocity(
    const StateRnPVA& current_state, const StateRnV& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateCubicSplineWithOnlyTargetVelocity(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA> GetNextStateEuclideanQuarticSpline(
    const StateRnPVA& current_state, const StateRnPV& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateQuarticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA>
GetNextStateOrientation3dCubicSplineWithOnlyTargetPosition(
    const StateRnPVA& current_state, const StateRnP& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateCubicSplineWithOnlyTargetPosition(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeOrientation3dTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeOrientation3dEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA>
GetNextStateOrientation3dCubicSplineWithOnlyTargetVelocity(
    const StateRnPVA& current_state, const StateRnV& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateCubicSplineWithOnlyTargetVelocity(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::ComputeOrientation3dEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA> GetNextStateOrientation3dQuarticSpline(
    const StateRnPVA& current_state, const StateRnPV& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateQuarticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeOrientation3dTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeOrientation3dEulerIntegrationStep);
}

icon::RealtimeStatusOr<QuaternionStateQVA>
GetNextStateOrientation3dCubicSplineWithOnlyTargetPosition(
    const QuaternionStateQVA& current_quat_state,
    const QuaternionStateQ& target_quat_state, double time_to_go_seconds,
    double dt_seconds) {
  StateRnPVA current_state;
  StateRnP target_state;
  StateRnPVA next_state;
  current_state = FromQuaternionStateToStateRn(current_quat_state);
  target_state = FromQuaternionStateToStateRn(target_quat_state);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      next_state,
      GetNextStateOrientation3dCubicSplineWithOnlyTargetPosition(
          current_state, target_state, time_to_go_seconds, dt_seconds));
  QuaternionStateQVA next_quat_state = FromStateRnToQuaternionState(next_state);
  return next_quat_state;
}

icon::RealtimeStatusOr<QuaternionStateQVA>
GetNextStateOrientation3dCubicSplineWithOnlyTargetVelocity(
    const QuaternionStateQVA& current_quat_state,
    const QuaternionStateV& target_quat_state, double time_to_go_seconds,
    double dt_seconds) {
  StateRnPVA current_state;
  StateRnV target_state;
  StateRnPVA next_state;
  current_state = FromQuaternionStateToStateRn(current_quat_state);
  target_state = FromQuaternionStateToStateRn(target_quat_state);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      next_state,
      GetNextStateOrientation3dCubicSplineWithOnlyTargetVelocity(
          current_state, target_state, time_to_go_seconds, dt_seconds));
  QuaternionStateQVA next_quat_state = FromStateRnToQuaternionState(next_state);
  return next_quat_state;
}

icon::RealtimeStatusOr<QuaternionStateQVA>
GetNextStateOrientation3dQuarticSpline(
    const QuaternionStateQVA& current_quat_state,
    const QuaternionStateQV& target_quat_state, double time_to_go_seconds,
    double dt_seconds) {
  StateRnPVA current_state;
  StateRnPV target_state;
  StateRnPVA next_state;
  current_state = FromQuaternionStateToStateRn(current_quat_state);
  target_state = FromQuaternionStateToStateRn(target_quat_state);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      next_state,
      GetNextStateOrientation3dQuarticSpline(current_state, target_state,
                                             time_to_go_seconds, dt_seconds));
  QuaternionStateQVA next_quat_state = FromStateRnToQuaternionState(next_state);
  return next_quat_state;
}

}  // namespace

namespace polynomial_spline_internal {

icon::RealtimeStatusOr<eigenmath::VectorNd>
ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget(
    const eigenmath::VectorNd& current_position,
    const eigenmath::VectorNd& target_position) {
  if (current_position.rows() != target_position.rows()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "current_position and target_position are mis-matched! ",
        current_position.rows(), " vs ", target_position.rows()));
  }
  eigenmath::VectorNd tangent_vector = target_position - current_position;
  return tangent_vector;
}

icon::RealtimeStatusOr<eigenmath::VectorNd>
ComputeEuclideanEulerIntegrationStep(const eigenmath::VectorNd& position,
                                     const eigenmath::VectorNd& velocity,
                                     double dt_seconds) {
  if (position.rows() != velocity.rows()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "position and velocity dimensions are mis-matched! ", position.rows(),
        " vs ", velocity.rows()));
  }
  eigenmath::VectorNd integrated_position = position + (velocity * dt_seconds);
  return integrated_position;
}

icon::RealtimeStatusOr<eigenmath::VectorNd>
ComputeOrientation3dTangentVectorMinimizingGeodesicDistanceToTarget(
    const eigenmath::VectorNd& current_orientation3d_angle_times_axis,
    const eigenmath::VectorNd& target_orientation3d_angle_times_axis) {
  if (current_orientation3d_angle_times_axis.rows() != 3) {
    return icon::InvalidArgumentError(
        "Dimensionality of current_orientation3d_angle_times_axis must be 3!");
  }
  if (target_orientation3d_angle_times_axis.rows() != 3) {
    return icon::InvalidArgumentError(
        "Dimensionality of target_orientation3d_angle_times_axis must be 3!");
  }
  eigenmath::Matrix3d current_to_target_rotation_matrix =
      (eigenmath::AngleTimesAxisToAngleAxis<double>(
           current_orientation3d_angle_times_axis)
           .toRotationMatrix()
           .transpose() *
       eigenmath::AngleTimesAxisToAngleAxis<double>(
           target_orientation3d_angle_times_axis)
           .toRotationMatrix());
  // do log() mapping log: SO(3) -> so(3)
  eigenmath::AngleAxisd  // NOLINT: <modernize-use-auto>
      current_to_target_angle_axis =
          eigenmath::AngleAxisd(current_to_target_rotation_matrix);
  eigenmath::VectorNd tangent_vector =
      eigenmath::AngleAxisToAngleTimesAxis<double>(
          current_to_target_angle_axis);
  return tangent_vector;
}

icon::RealtimeStatusOr<eigenmath::VectorNd>
ComputeOrientation3dEulerIntegrationStep(
    const eigenmath::VectorNd& orientation3d_angle_times_axis,
    const eigenmath::VectorNd& angular_velocity3d, double dt_seconds) {
  if (orientation3d_angle_times_axis.rows() != 3) {
    return icon::InvalidArgumentError(
        "Dimensionality of orientation3d_angle_times_axis must be 3!");
  }
  if (angular_velocity3d.rows() != 3) {
    return icon::InvalidArgumentError(
        "Dimensionality of angular_velocity3d must be 3!");
  }
  eigenmath::Matrix3d ori_rotation_matrix =
      eigenmath::AngleTimesAxisToAngleAxis<double>(
          orientation3d_angle_times_axis)
          .toRotationMatrix();
  // do exp() mapping exp: so(3) -> SO(3)
  eigenmath::Matrix3d delta_rotation_matrix =
      eigenmath::AngleTimesAxisToAngleAxis<double>(angular_velocity3d *
                                                   dt_seconds)
          .toRotationMatrix();
  eigenmath::AngleAxisd integrated_ori3d_angle_axis =
      eigenmath::AngleAxisd(ori_rotation_matrix * delta_rotation_matrix);
  eigenmath::VectorNd integrated_orientation3d_angle_times_axis =
      eigenmath::AngleAxisToAngleTimesAxis<double>(integrated_ori3d_angle_axis);
  return integrated_orientation3d_angle_times_axis;
}

icon::RealtimeStatusOr<StateRnPVA> EvaluateEuclideanQuadraticSpline(
    const StateRnPA& start_state, const StateRnP& target_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return EvaluateQuadraticSpline(
      start_state, target_state, time_horizon_seconds, time_eval_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVAJ>
EvaluateEuclideanCubicSplineWithVelocityContinuity(
    const StateRnPV& start_state, const StateRnPV& target_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return EvaluateCubicSplineWithVelocityContinuity(
      start_state, target_state, time_horizon_seconds, time_eval_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA>
EvaluateEuclideanCubicSplineWithAccelerationContinuity(
    const StateRnPA& start_state, const StateRnPA& target_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return EvaluateCubicSplineWithAccelerationContinuity(
      start_state, target_state, time_horizon_seconds, time_eval_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVAJ> EvaluateEuclideanQuinticSpline(
    const StateRnPVA& start_state, const StateRnPVA& target_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return EvaluateQuinticSpline(
      start_state, target_state, time_horizon_seconds, time_eval_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA> GetNextStateEuclideanQuadraticSpline(
    const StateRnPA& current_state, const StateRnP& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateQuadraticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVAJ>
GetNextStateEuclideanCubicSplineWithVelocityContinuity(
    const StateRnPV& current_state, const StateRnPV& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateCubicSplineWithVelocityContinuity(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVA>
GetNextStateEuclideanCubicSplineWithAccelerationContinuity(
    const StateRnPA& current_state, const StateRnPA& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateCubicSplineWithAccelerationContinuity(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateEuclideanQuinticSpline(
    const StateRnPVA& current_state, const StateRnPVA& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateQuinticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateEuclideanSepticSpline(
    const StateRnPVAJ& current_state, const StateRnPVAJ& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateSepticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeEuclideanEulerIntegrationStep);
}

icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateOrientation3dQuinticSpline(
    const StateRnPVA& current_state, const StateRnPVA& target_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateQuinticSpline(
      current_state, target_state, time_to_go_seconds, dt_seconds,
      polynomial_spline_internal::
          ComputeOrientation3dTangentVectorMinimizingGeodesicDistanceToTarget,
      polynomial_spline_internal::ComputeOrientation3dEulerIntegrationStep);
}

}  // namespace polynomial_spline_internal

icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 6>>
ComputeEuclideanQuinticSplineCoefficients(const StateRnPVA& start_state,
                                          const StateRnPVA& target_state,
                                          double time_horizon_seconds) {
  return ComputeQuinticSplineCoefficients(
      start_state, target_state, time_horizon_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget);
}

icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 8>>
ComputeEuclideanSepticSplineCoefficients(const StateRnPVAJ& start_state,
                                         const StateRnPVAJ& target_state,
                                         double time_horizon_seconds) {
  return ComputeSepticSplineCoefficients(
      start_state, target_state, time_horizon_seconds,
      polynomial_spline_internal::
          ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget);
}

icon::RealtimeStatusOr<CartStatePVA>
GetNextStateCartesianPose3CubicSplineWithOnlyTargetPosition(
    const CartStatePVA& current_state, const CartStateP& target_state,
    double time_to_go_seconds, double dt_seconds) {
  // Convert CartState to a LocalPolynomialState that requires angular velocity
  // and acceleration to be in the local (tangent) space of a reference
  // quaternion. We use the `current_state` as the reference_state here because
  // we are considering the local frame representing the tangent space within
  // the small vicinity/neighborhood of the `current_state`.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LocalPolynomialStatePVA lp_current_state,
      CartStateToLocalCoordinates(
          /* reference_state=*/current_state, /* cart_state=*/current_state));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LocalPolynomialStateP lp_target_state,
      CartStateToLocalCoordinates(
          /* reference_state=*/current_state, /* cart_state=*/target_state));

  // Update the spline next state.
  LocalPolynomialStatePVA lp_next_state;
  INTRINSIC_RT_RETURN_IF_ERROR(lp_next_state.translation3d_state.SetSize(3));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      lp_next_state.translation3d_state,
      GetNextStateEuclideanCubicSplineWithOnlyTargetPosition(
          lp_current_state.translation3d_state,
          lp_target_state.translation3d_state, time_to_go_seconds, dt_seconds));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      lp_next_state.quat_state_with_local_derivatives,
      GetNextStateOrientation3dCubicSplineWithOnlyTargetPosition(
          lp_current_state.quat_state_with_local_derivatives,
          lp_target_state.quat_state_with_local_derivatives, time_to_go_seconds,
          dt_seconds));
  // Convert back to CartState.
  CartStatePVA next_state =
      LocalCoordinatesToCartState(current_state, lp_next_state);

  return next_state;
}

icon::RealtimeStatusOr<CartStatePVA>
GetNextStateCartesianPose3CubicSplineWithOnlyTargetVelocity(
    const CartStatePVA& current_state, const CartStateV& target_state,
    double time_to_go_seconds, double dt_seconds) {
  // Convert CartState to a LocalPolynomialState that requires angular velocity
  // and acceleration to be in the local (tangent) space of a reference
  // quaternion. We use the `current_state` as the reference_state here because
  // we are considering the local frame representing the tangent space within
  // the small vicinity/neighborhood of the `current_state`.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LocalPolynomialStatePVA lp_current_state,
      CartStateToLocalCoordinates(
          /* reference_state=*/current_state, /* cart_state=*/current_state));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LocalPolynomialStateV lp_target_state,
      CartStateToLocalCoordinates(
          /* reference_state=*/current_state, /* cart_state=*/target_state));

  // Update the spline next state.
  LocalPolynomialStatePVA lp_next_state;
  INTRINSIC_RT_RETURN_IF_ERROR(lp_next_state.translation3d_state.SetSize(3));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      lp_next_state.translation3d_state,
      GetNextStateEuclideanCubicSplineWithOnlyTargetVelocity(
          lp_current_state.translation3d_state,
          lp_target_state.translation3d_state, time_to_go_seconds, dt_seconds));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      lp_next_state.quat_state_with_local_derivatives,
      GetNextStateOrientation3dCubicSplineWithOnlyTargetVelocity(
          lp_current_state.quat_state_with_local_derivatives,
          lp_target_state.quat_state_with_local_derivatives, time_to_go_seconds,
          dt_seconds));
  // Convert back to CartState.
  CartStatePVA next_state =
      LocalCoordinatesToCartState(current_state, lp_next_state);

  return next_state;
}

icon::RealtimeStatusOr<CartStatePVA> GetNextStateCartesianPose3QuarticSpline(
    const CartStatePVA& current_state, const CartStatePV& target_state,
    double time_to_go_seconds, double dt_seconds) {
  // Convert CartState to a LocalPolynomialState that requires angular velocity
  // and acceleration to be in the local (tangent) space of a reference
  // quaternion. We use the `current_state` as the reference_state here because
  // we are considering the local frame representing the tangent space within
  // the small vicinity/neighborhood of the `current_state`.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LocalPolynomialStatePVA lp_current_state,
      CartStateToLocalCoordinates(
          /* reference_state=*/current_state, /* cart_state=*/current_state));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LocalPolynomialStatePV lp_target_state,
      CartStateToLocalCoordinates(
          /* reference_state=*/current_state, /* cart_state=*/target_state));

  // Update the spline next state.
  LocalPolynomialStatePVA lp_next_state;
  INTRINSIC_RT_RETURN_IF_ERROR(lp_next_state.translation3d_state.SetSize(3));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      lp_next_state.translation3d_state,
      GetNextStateEuclideanQuarticSpline(lp_current_state.translation3d_state,
                                         lp_target_state.translation3d_state,
                                         time_to_go_seconds, dt_seconds));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      lp_next_state.quat_state_with_local_derivatives,
      GetNextStateOrientation3dQuarticSpline(
          lp_current_state.quat_state_with_local_derivatives,
          lp_target_state.quat_state_with_local_derivatives, time_to_go_seconds,
          dt_seconds));
  // Convert back to CartState.
  CartStatePVA next_state =
      LocalCoordinatesToCartState(current_state, lp_next_state);

  return next_state;
}

icon::RealtimeStatusOr<QuaternionStateQVA>
GetNextStateOrientation3dQuinticSpline(
    const QuaternionStateQVA& current_quat_state,
    const QuaternionStateQVA& target_quat_state, double time_to_go_seconds,
    double dt_seconds) {
  StateRnPVA current_state;
  StateRnPVA target_state;
  StateRnPVA next_state;
  current_state = FromQuaternionStateToStateRn(current_quat_state);
  target_state = FromQuaternionStateToStateRn(target_quat_state);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      next_state,
      polynomial_spline_internal::GetNextStateOrientation3dQuinticSpline(
          current_state, target_state, time_to_go_seconds, dt_seconds));
  QuaternionStateQVA next_quat_state = FromStateRnToQuaternionState(next_state);
  return next_quat_state;
}

icon::RealtimeStatusOr<CartStatePVA> GetNextStateCartesianPose3QuinticSpline(
    const CartStatePVA& current_state, const CartStatePVA& target_state,
    double time_to_go_seconds, double dt_seconds) {
  // Convert CartState to a LocalPolynomialState that requires angular velocity
  // and acceleration to be in the local (tangent) space of a reference
  // quaternion. We use the `current_state` as the reference_state here because
  // we are considering the local frame representing the tangent space within
  // the small vicinity/neighborhood of the `current_state`.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LocalPolynomialStatePVA lp_current_state,
      CartStateToLocalCoordinates(
          /* reference_state=*/current_state, /* cart_state=*/current_state));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LocalPolynomialStatePVA lp_target_state,
      CartStateToLocalCoordinates(
          /* reference_state=*/current_state, /* cart_state=*/target_state));

  // Update the spline next state.
  LocalPolynomialStatePVA lp_next_state;
  INTRINSIC_RT_RETURN_IF_ERROR(lp_next_state.translation3d_state.SetSize(3));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      lp_next_state.translation3d_state,
      polynomial_spline_internal::GetNextStateEuclideanQuinticSpline(
          lp_current_state.translation3d_state,
          lp_target_state.translation3d_state, time_to_go_seconds, dt_seconds));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      lp_next_state.quat_state_with_local_derivatives,
      GetNextStateOrientation3dQuinticSpline(
          lp_current_state.quat_state_with_local_derivatives,
          lp_target_state.quat_state_with_local_derivatives, time_to_go_seconds,
          dt_seconds));
  // Convert back to CartState.
  CartStatePVA next_state =
      LocalCoordinatesToCartState(current_state, lp_next_state);

  return next_state;
}

icon::RealtimeStatusOr<StateRnPVA> GetNextStateJointQuadraticSpline(
    const StateRnPA& current_joint_state, const StateRnP& target_joint_state,
    double time_to_go_seconds, double dt_seconds) {
  return polynomial_spline_internal::GetNextStateEuclideanQuadraticSpline(
      current_joint_state, target_joint_state, time_to_go_seconds, dt_seconds);
}

icon::RealtimeStatusOr<StateRnPVA>
GetNextStateJointCubicSplineWithOnlyTargetPosition(
    const StateRnPVA& current_joint_state, const StateRnP& target_joint_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateEuclideanCubicSplineWithOnlyTargetPosition(
      current_joint_state, target_joint_state, time_to_go_seconds, dt_seconds);
}

icon::RealtimeStatusOr<StateRnPVA>
GetNextStateJointCubicSplineWithOnlyTargetVelocity(
    const StateRnPVA& current_joint_state, const StateRnV& target_joint_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateEuclideanCubicSplineWithOnlyTargetVelocity(
      current_joint_state, target_joint_state, time_to_go_seconds, dt_seconds);
}

icon::RealtimeStatusOr<StateRnPVAJ>
GetNextStateJointCubicSplineWithVelocityContinuity(
    const StateRnPV& current_joint_state, const StateRnPV& target_joint_state,
    double time_to_go_seconds, double dt_seconds) {
  return polynomial_spline_internal::
      GetNextStateEuclideanCubicSplineWithVelocityContinuity(
          current_joint_state, target_joint_state, time_to_go_seconds,
          dt_seconds);
}

icon::RealtimeStatusOr<StateRnPVA>
GetNextStateJointCubicSplineWithAccelerationContinuity(
    const StateRnPA& current_joint_state, const StateRnPA& target_joint_state,
    double time_to_go_seconds, double dt_seconds) {
  return polynomial_spline_internal::
      GetNextStateEuclideanCubicSplineWithAccelerationContinuity(
          current_joint_state, target_joint_state, time_to_go_seconds,
          dt_seconds);
}

icon::RealtimeStatusOr<StateRnPVA> GetNextStateJointQuarticSpline(
    const StateRnPVA& current_joint_state, const StateRnPV& target_joint_state,
    double time_to_go_seconds, double dt_seconds) {
  return GetNextStateEuclideanQuarticSpline(
      current_joint_state, target_joint_state, time_to_go_seconds, dt_seconds);
}

icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateJointQuinticSpline(
    const StateRnPVA& current_joint_state, const StateRnPVA& target_joint_state,
    double time_to_go_seconds, double dt_seconds) {
  return polynomial_spline_internal::GetNextStateEuclideanQuinticSpline(
      current_joint_state, target_joint_state, time_to_go_seconds, dt_seconds);
}

icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateJointSepticSpline(
    const StateRnPVAJ& current_joint_state,
    const StateRnPVAJ& target_joint_state, double time_to_go_seconds,
    double dt_seconds) {
  return polynomial_spline_internal::GetNextStateEuclideanSepticSpline(
      current_joint_state, target_joint_state, time_to_go_seconds, dt_seconds);
}

icon::RealtimeStatusOr<StateRnPVA> EvaluateJointQuadraticSpline(
    const StateRnPA& start_joint_state, const StateRnP& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return polynomial_spline_internal::EvaluateEuclideanQuadraticSpline(
      start_joint_state, target_joint_state, time_horizon_seconds,
      time_eval_seconds);
}

icon::RealtimeStatusOr<StateRnPVAJ>
EvaluateJointCubicSplineWithVelocityContinuity(
    const StateRnPV& start_joint_state, const StateRnPV& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return polynomial_spline_internal::
      EvaluateEuclideanCubicSplineWithVelocityContinuity(
          start_joint_state, target_joint_state, time_horizon_seconds,
          time_eval_seconds);
}

icon::RealtimeStatusOr<StateRnPVA>
EvaluateJointCubicSplineWithAccelerationContinuity(
    const StateRnPA& start_joint_state, const StateRnPA& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return polynomial_spline_internal::
      EvaluateEuclideanCubicSplineWithAccelerationContinuity(
          start_joint_state, target_joint_state, time_horizon_seconds,
          time_eval_seconds);
}

icon::RealtimeStatusOr<StateRnPVAJ> EvaluateJointQuinticSpline(
    const StateRnPVA& start_joint_state, const StateRnPVA& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return polynomial_spline_internal::EvaluateEuclideanQuinticSpline(
      start_joint_state, target_joint_state, time_horizon_seconds,
      time_eval_seconds);
}

icon::RealtimeStatusOr<StateRnPVAJ> EvaluateJointSepticSpline(
    const StateRnPVAJ& start_joint_state, const StateRnPVAJ& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds) {
  return EvaluateEuclideanSepticSpline(start_joint_state, target_joint_state,
                                       time_horizon_seconds, time_eval_seconds);
}

}  // namespace intrinsic
