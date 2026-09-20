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

#ifndef INTRINSIC_MATH_SPLINE_POLYNOMIAL_SPLINE_H_
#define INTRINSIC_MATH_SPLINE_POLYNOMIAL_SPLINE_H_

// Polynomial Spline(s) that do not have any internal state,
// hence can take inputs of any start/current position/velocity/acceleration
// and any target position/velocity/acceleration. Therefore these can be used
// for both offline and online/real-time planning.
// Current implementation includes Quintic Spline (or 5th-degree Spline or
// Minimum-Jerk Spline [1]) for N-dimensional Euclidean vector and
// for 3D orientation.
//
// References:
// [1] Flash T, Hogan N. The coordination of arm movements: an experimentally
//     confirmed mathematical model. J Neurosci. 1985 Jul;5(7):1688-703.

#include <array>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/state_rn.h"

namespace intrinsic {
namespace polynomial_spline_internal {

// The following constant is specified as a lower-bound to avoid
// division-by-zero error in polynomial spline algorithm's next state
// computation.
constexpr double kMinimumDtSeconds = 1.0e-4;

// The smallest time duration that is still considered 0 seconds.
constexpr double kEpsilonTimeDurationSeconds = 1.0e-5;

// `ComputeTangentVectorMinimizingGeodesicDistanceToTarget()` version for
// Euclidean vectors.
// Computes the tangent vector originated at the `current_position` on the
// manifold, which if followed will minimize the geodesic distance between
// the `current_position` and `target_position`.
// Fails if there is a dimension mis-match between `current_position` and
// `target_position`.
icon::RealtimeStatusOr<eigenmath::VectorNd>
ComputeEuclideanTangentVectorMinimizingGeodesicDistanceToTarget(
    const eigenmath::VectorNd& current_position,
    const eigenmath::VectorNd& target_position);

// `ComputeEulerIntegrationStep()` version for Euclidean vectors.
// Integrates `velocity` (in the tangent space) scaled by `dt_seconds` forward
// into `position` on the manifold.
// Fails if there is a dimension mis-match between `position` and `velocity`.
icon::RealtimeStatusOr<eigenmath::VectorNd>
ComputeEuclideanEulerIntegrationStep(const eigenmath::VectorNd& position,
                                     const eigenmath::VectorNd& velocity,
                                     double dt_seconds);

// `ComputeTangentVectorMinimizingGeodesicDistanceToTarget()` version for
// 3D orientation with angle-times-axis representation.
// Computes the tangent vector originated at the
// `current_orientation3d_angle_times_axis` (current 3D orientation in
// angle-times-axis representation) on the manifold, which if followed will
// minimize the geodesic distance between the
// `current_orientation3d_angle_times_axis` and
// `target_orientation3d_angle_times_axis`.
// Fails if the dimension of `current_orientation3d_angle_times_axis` or
// `target_orientation3d_angle_times_axis` is not 3.
icon::RealtimeStatusOr<eigenmath::VectorNd>
ComputeOrientation3dTangentVectorMinimizingGeodesicDistanceToTarget(
    const eigenmath::VectorNd& current_orientation3d_angle_times_axis,
    const eigenmath::VectorNd& target_orientation3d_angle_times_axis);

// `ComputeEulerIntegrationStep()` version for 3D orientation with
// angle-times-axis representation.
// Integrates `angular_velocity3d` (in the tangent space)
// scaled by `dt_seconds` forward into
// `orientation3d_angle_times_axis` (3D orientation in angle-times-axis
// representation) on the manifold.
// Fails if the dimension of `orientation3d_angle_times_axis` or
// `angular_velocity3d` is not 3.
icon::RealtimeStatusOr<eigenmath::VectorNd>
ComputeOrientation3dEulerIntegrationStep(
    const eigenmath::VectorNd& orientation3d_angle_times_axis,
    const eigenmath::VectorNd& angular_velocity3d, double dt_seconds);

// The interpolation function using Quadratic (2nd-degree) Spline for
// N-dimensional Euclidean vectors. At its core, this function will call
// EvaluateQuadraticSpline() which is defined in polynomial_spline.cc. Please
// refer to the documentation of EvaluateQuadraticSpline() to find the possible
// failure cases. Note that only positions are continuous, velocities and
// accelerations are not. Velocities are affine and acceleration is constant.
// When velocities and accelerations are not required, a StatePVA can be easily
// downcasted into a StateP.
icon::RealtimeStatusOr<StateRnPVA> EvaluateEuclideanQuadraticSpline(
    const StateRnPA& start_state, const StateRnP& target_state,
    double time_horizon_seconds, double time_eval_seconds);

// The interpolation function using Cubic (3rd-degree) Spline for N-dimensional
// Euclidean vectors that preserves velocity continuity at boundary conditions.
// At its core, this function will call
// EvaluateCubicSplineWithVelocityContinuity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// EvaluateCubicSplineWithVelocityContinuity() to find the possible failure
// cases. Note that the acceleration profile in a cubic spline will be piecewise
// affine and the jerk profile will be piecewise constant. When accelerations
// and/or jerks are not required, a StatePVAJ can be easily downcasted into a
// StatePV or StatePVA.
icon::RealtimeStatusOr<StateRnPVAJ>
EvaluateEuclideanCubicSplineWithVelocityContinuity(
    const StateRnPV& start_state, const StateRnPV& target_state,
    double time_horizon_seconds, double time_eval_seconds);

// The interpolation function using Cubic (3rd-degree) Spline for N-dimensional
// Euclidean vectors that preserves acceleration continuity at the boundary
// values. At its core, this function will call
// EvaluateCubicSplineWithAccelerationContinuity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// EvaluateCubicSplineWithAccelerationContinuity() to find the possible failure
// cases. Note that the acceleration profile in a cubic spline will be piecewise
// affine. When accelerations are not required, a StatePVA can be easily
// downcasted into a StatePV.
icon::RealtimeStatusOr<StateRnPVA>
EvaluateEuclideanCubicSplineWithAccelerationContinuity(
    const StateRnPA& start_state, const StateRnPA& target_state,
    double time_horizon_seconds, double time_eval_seconds);

// The interpolation function using Quintic (5th-degree) Spline for
// N-dimensional Euclidean vectors. At its core, this function will call
// EvaluateQuinticSpline() which is defined in polynomial_spline.cc. Please
// refer to the documentation of EvaluateQuinticSpline() to find the possible
// failure cases.
icon::RealtimeStatusOr<StateRnPVAJ> EvaluateEuclideanQuinticSpline(
    const StateRnPVA& start_state, const StateRnPVA& target_state,
    double time_horizon_seconds, double time_eval_seconds);

// The Quadratic (2nd-degree) Spline for N-dimensional Euclidean vectors. At its
// core, this function will call GetNextStateQuadraticSpline() which is defined
// in polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuadraticSpline() to find the possible failure
// cases. Note that the acceleration profile in a quadratic spline will be
// piecewise constant. When velocities and accelerations are not required, a
// StatePVA can be easily downcasted into a StateP.
icon::RealtimeStatusOr<StateRnPVA> GetNextStateEuclideanQuadraticSpline(
    const StateRnPA& current_state, const StateRnP& target_state,
    double time_to_go_seconds, double dt_seconds);

// The Cubic (3rd-degree) Spline for N-dimensional Euclidean vectors that
// preserves velocity continuity. At its core, this function will call
// GetNextStateCubicSplineWithVelocityContinuity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// GetNextStateCubicSplineWithVelocityContinuity() to find the possible failure
// cases. Note that the acceleration profile in a cubic spline will be piecewise
// affine and the jerk profile will be piecewise constant. When accelerations
// and/or jerks are not required, a StatePVAJ can be easily downcasted into a
// StatePV or StatePVA.
icon::RealtimeStatusOr<StateRnPVAJ>
GetNextStateEuclideanCubicSplineWithVelocityContinuity(
    const StateRnPV& current_state, const StateRnPV& target_state,
    double time_to_go_seconds, double dt_seconds);

// The Cubic (3rd-degree) Spline for N-dimensional Euclidean vectors that
// preserves acceleration continuity. At its core, this function will call
// GetNextStateCubicSplineWithAccelerationContinuity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// GetNextStateCubicSplineWithAccelerationContinuity() to find the possible
// failure cases. Note that the acceleration profile in a cubic spline will be
// piecewise affine. When accelerations are not required, a StatePVA can be
// easily downcasted into a StatePV.
icon::RealtimeStatusOr<StateRnPVA>
GetNextStateEuclideanCubicSplineWithAccelerationContinuity(
    const StateRnPA& current_state, const StateRnPA& target_state,
    double time_to_go_seconds, double dt_seconds);

// The Quintic (5th-degree) Spline for N-dimensional Euclidean vectors.
// At its core, this function will call GetNextStateQuinticSpline() which is
// defined in polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuinticSpline() to find the possible failure cases.
icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateEuclideanQuinticSpline(
    const StateRnPVA& current_state, const StateRnPVA& target_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Quintic (5th-degree) Spline interface for angle-times-axis
// representation of 3D orientation. At its core, this function will call
// GetNextStateQuinticSpline() which is defined in polynomial_spline.cc. Please
// refer to the documentation of GetNextStateQuinticSpline() to find the
// possible failure cases.
icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateOrientation3dQuinticSpline(
    const StateRnPVA& current_state, const StateRnPVA& target_state,
    double time_to_go_seconds, double dt_seconds);

}  // namespace polynomial_spline_internal

// Computes the Quintic (5th-degree) Spline Coefficients for the polynomial
// "c_5*x^5 + c_4*x^4 + .... + c_1 x + c_0". Returns the array [c_0, ... , c_5].
icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 6>>
ComputeEuclideanQuinticSplineCoefficients(const StateRnPVA& start_state,
                                          const StateRnPVA& target_state,
                                          double time_horizon_seconds);

// Computes the Septic (7th-degree) Spline Coefficients for the polynomial
// "c_7*x^7 + c_6*x^6 + .... + c_1 x + c_0". Returns the array [c_0, ... , c_7].
icon::RealtimeStatusOr<std::array<eigenmath::VectorNd, 8>>
ComputeEuclideanSepticSplineCoefficients(const StateRnPVAJ& start_state,
                                         const StateRnPVAJ& target_state,
                                         double time_horizon_seconds);

// Provides Cubic (3rd-degree) Spline interface for Cartesian Pose3, with the
// special boundary conditions of PVA at the start state and only P at the
// target state. At its core, this function will call
// GetNextStateCubicSplineWithOnlyTargetPosition() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuinticSplineWithOnlyTargetPosition() to find the possible
// failure cases.
icon::RealtimeStatusOr<CartStatePVA>
GetNextStateCartesianPose3CubicSplineWithOnlyTargetPosition(
    const CartStatePVA& current_state, const CartStateP& target_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Cubic (3rd-degree) Spline interface for Cartesian Pose3, with the
// special boundary conditions of PVA at the start state, and V at the target
// state. At its core, this function will call
// GetNextStateCubicSplineWithOnlyTargetVelocity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuinticSplineWithOnlyTargetVelocity() to find the possible
// failure cases.
icon::RealtimeStatusOr<CartStatePVA>
GetNextStateCartesianPose3CubicSplineWithOnlyTargetVelocity(
    const CartStatePVA& current_state, const CartStateV& target_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Quartic (4th-degree) Spline interface for Cartesian Pose3. The
// spline boundary conditions are PVA at the start state, and PV at the target
// state. At its core, this function will call GetNextStateQuarticSpline() which
// is defined in polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuarticSpline() to find the possible failure cases.
icon::RealtimeStatusOr<CartStatePVA> GetNextStateCartesianPose3QuarticSpline(
    const CartStatePVA& current_state, const CartStatePV& target_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Quintic (5th-degree) Spline interface for Quaternion representation
// of 3D orientation. At its core, this function will call
// GetNextStateQuinticSpline() which is defined in polynomial_spline.cc. Please
// refer to the documentation of GetNextStateQuinticSpline() to find the
// possible failure cases.
icon::RealtimeStatusOr<QuaternionStateQVA>
GetNextStateOrientation3dQuinticSpline(
    const QuaternionStateQVA& current_quat_state,
    const QuaternionStateQVA& target_quat_state, double time_to_go_seconds,
    double dt_seconds);

// Provides Quintic (5th-degree) Spline interface for Cartesian Pose3.
// At its core, this function will call GetNextStateQuinticSpline() which is
// defined in polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuinticSpline() to find the possible failure cases.
icon::RealtimeStatusOr<CartStatePVA> GetNextStateCartesianPose3QuinticSpline(
    const CartStatePVA& current_state, const CartStatePVA& target_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Quadratic (2nd-degree) Spline interface for N-dimensional joints. At
// its core, this function will call GetNextStateQuadraticSpline() which is
// defined in polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuadraticSpline() to find the possible failure
// cases. Note that the acceleration profile in a quadratic spline will be
// piecewise constant. When velocities and accelerations are not required, a
// StatePVA can be easily downcasted into a StateP.
icon::RealtimeStatusOr<StateRnPVA> GetNextStateJointQuadraticSpline(
    const StateRnPA& current_joint_state, const StateRnP& target_joint_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Cubic (3rd-degree) Spline interface for N-dimensional joints that
// have only a target position as boundary condition. At its core, this function
// will call GetNextStateCubicSplineWithOnlyTargetPosition() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// GetNextStateCubicSplineWithOnlyTargetPosition() to find the possible
// failure cases.
icon::RealtimeStatusOr<StateRnPVA>
GetNextStateJointCubicSplineWithOnlyTargetPosition(
    const StateRnPVA& current_joint_state, const StateRnP& target_joint_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Cubic (3rd-degree) Spline interface for N-dimensional joints that
// have only a target velocity as boundary condition. At its core, this function
// will call GetNextStateCubicSplineWithOnlyTargetVelocity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// GetNextStateCubicSplineWithOnlyTargetVelocity() to find the possible
// failure cases.
icon::RealtimeStatusOr<StateRnPVA>
GetNextStateJointCubicSplineWithOnlyTargetVelocity(
    const StateRnPVA& current_joint_state, const StateRnV& target_joint_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Cubic (3rd-degree) Spline interface for N-dimensional joints that
// preserves velocity continuity. At its core, this function will call
// GetNextStateCubicSplineWithVelocityContinuity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// GetNextStateCubicSplineWithVelocityContinuity() to find the possible failure
// cases. Note that the acceleration profile in a cubic spline will be piecewise
// affine and the jerk profile will be piecewise constant. When accelerations
// and/or jerks are not required, a StatePVAJ can be easily downcasted into a
// StatePV or StatePVA.
icon::RealtimeStatusOr<StateRnPVAJ>
GetNextStateJointCubicSplineWithVelocityContinuity(
    const StateRnPV& current_joint_state, const StateRnPV& target_joint_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Cubic (3rd-degree) Spline interface for N-dimensional joints that
// preserves acceleration continuity. At its core, this function will call
// GetNextStateCubicSplineWithAccelerationContinuity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// GetNextStateCubicSplineWithAccelerationContinuity() to find the possible
// failure cases. Note that the acceleration profile in a cubic spline will be
// piecewise affine. When accelerations are not required, a StatePVA can be
// easily downcasted into a StatePV.
icon::RealtimeStatusOr<StateRnPVA>
GetNextStateJointCubicSplineWithAccelerationContinuity(
    const StateRnPA& current_joint_state, const StateRnPA& target_joint_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Quartic (4th-degree) Spline interface for N-dimensional joints.
// At its core, this function will call GetNextStateQuarticSpline() which is
// defined in polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuarticSpline() to find the possible failure cases.
icon::RealtimeStatusOr<StateRnPVA> GetNextStateJointQuarticSpline(
    const StateRnPVA& current_joint_state, const StateRnPV& target_joint_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Quintic (5th-degree) Spline interface for N-dimensional joints.
// At its core, this function will call GetNextStateQuinticSpline() which is
// defined in polynomial_spline.cc. Please refer to the documentation of
// GetNextStateQuinticSpline() to find the possible failure cases.
icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateJointQuinticSpline(
    const StateRnPVA& current_joint_state, const StateRnPVA& target_joint_state,
    double time_to_go_seconds, double dt_seconds);

// Provides Septic (7th-degree) Spline interface for N-dimensional joints.
// At its core, this function will call GetNextStateSepticSpline() which is
// defined in polynomial_spline.cc. Please refer to the documentation of
// GetNextStateSepticSpline() to find the possible failure cases.
icon::RealtimeStatusOr<StateRnPVAJ> GetNextStateJointSepticSpline(
    const StateRnPVAJ& current_joint_state,
    const StateRnPVAJ& target_joint_state, double time_to_go_seconds,
    double dt_seconds);

// Provides Quadratic (2nd-degree) Spline interpolation interface for
// N-dimensional joints. At its core, this function will call
// EvaluateQuadraticSpline() which is defined in polynomial_spline.cc. Please
// refer to the documentation of EvaluateQuadraticSpline() to find the possible
// failure cases. Note that the acceleration profile in a quadratic spline will
// be piecewise constant. When velocities and accelerations are not required, a
// StatePVA can be easily downcasted into a StateP.
icon::RealtimeStatusOr<StateRnPVA> EvaluateJointQuadraticSpline(
    const StateRnPA& start_joint_state, const StateRnP& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds);

// Provides Cubic (3rd-degree) Spline interpolation interface for N-dimensional
// joints that preserves velocity continuity. At its core, this function will
// call EvaluateCubicSplineWithVelocityContinuity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// EvaluateCubicSplineWithVelocityContinuity() to find the possible failure
// cases. Note that the acceleration profile in a cubic spline will be piecewise
// affine and the jerk profile will be piecewise constant. When accelerations
// and/or jerks are not required, a StatePVAJ can be easily downcasted into a
// StatePV or StatePVA.
icon::RealtimeStatusOr<StateRnPVAJ>
EvaluateJointCubicSplineWithVelocityContinuity(
    const StateRnPV& start_joint_state, const StateRnPV& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds);

// Provides Cubic (3rd-degree) Spline interpolation interface for N-dimensional
// joints that preserves acceleration continuity. At its core, this function
// will call EvaluateCubicSplineWithAccelerationContinuity() which is defined in
// polynomial_spline.cc. Please refer to the documentation of
// EvaluateCubicSplineWithAccelerationContinuity() to find the possible failure
// cases. Note that the acceleration profile in a cubic spline will be piecewise
// affine. When accelerations are not required, a StatePVA can be easily
// downcasted into a StatePV.
icon::RealtimeStatusOr<StateRnPVA>
EvaluateJointCubicSplineWithAccelerationContinuity(
    const StateRnPA& start_joint_state, const StateRnPA& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds);

// Provides Quintic (5th-degree) Spline interpolation interface for
// N-dimensional joints. At its core, this function will call
// EvaluateQuinticSpline() which is defined in polynomial_spline.cc. Please
// refer to the documentation of EvaluateQuinticSpline() to find the possible
// failure cases.
icon::RealtimeStatusOr<StateRnPVAJ> EvaluateJointQuinticSpline(
    const StateRnPVA& start_joint_state, const StateRnPVA& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds);

// Provides Septic (7th-degree) Spline interpolation interface for N-dimensional
// joints. At its core, this function will call EvaluateSepticSpline() which is
// defined in polynomial_spline.cc. Please refer to the documentation of
// EvaluateSepticSpline() to find the possible failure cases.
icon::RealtimeStatusOr<StateRnPVAJ> EvaluateJointSepticSpline(
    const StateRnPVAJ& start_joint_state, const StateRnPVAJ& target_joint_state,
    double time_horizon_seconds, double time_eval_seconds);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_POLYNOMIAL_SPLINE_H_
