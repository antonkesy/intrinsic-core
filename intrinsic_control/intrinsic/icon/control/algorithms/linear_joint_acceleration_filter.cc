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

#include "intrinsic/icon/control/algorithms/linear_joint_acceleration_filter.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/linear_kalman_filter.h"
#include "intrinsic/icon/proto/linear_joint_acceleration_filter_config.pb.h"
#include "intrinsic/math/linear_systems/discrete_algebraic_riccati_equation.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::StatusOr<LinearJointAccelerationFilter>
LinearJointAccelerationFilter::Create(
    double dt, const eigenmath::MatrixXd& Q, const eigenmath::MatrixXd& R,
    const DiscreteAlgebraicRiccatiSolverSettings& dare_settings,
    const eigenmath::MatrixXd& P_initial) {
  if (Q.cols() != kStateDim || Q.rows() != kStateDim) {
    return absl::FailedPreconditionError("Q matrix has wrong dimensions.");
  }
  if (R.cols() != kOutputDim || R.rows() != kOutputDim) {
    return absl::FailedPreconditionError("R matrix has wrong dimensions.");
  }
  if (P_initial.cols() != kStateDim || P_initial.rows() != kStateDim) {
    return absl::FailedPreconditionError("P matrix has wrong dimensions.");
  }

  // Discrete-time dynamics.
  eigenmath::MatrixXd A;
  eigenmath::MatrixXd B;
  eigenmath::MatrixXd C;
  GetSystemMatrices(dt, A, B, C);

  INTR_ASSIGN_OR_RETURN(
      auto kalman_filter,
      LinearKalmanFilter::Create(Q, R, A, B, C, P_initial, dare_settings));

  return LinearJointAccelerationFilter(kalman_filter);
}

// static
absl::StatusOr<LinearJointAccelerationFilter>
LinearJointAccelerationFilter::Create(
    const intrinsic_proto::icon::LinearJointAccelerationFilterConfig& config,
    const double control_frequency_hz, const eigenmath::MatrixXd& P_initial) {
  // Joint acceleration filter process noise covariance.
  eigenmath::VectorXd q(icon::LinearJointAccelerationFilter::kStateDim);
  q << config.joint_position_process_noise(),
      config.joint_velocity_process_noise(),
      config.joint_acceleration_process_noise(),
      config.joint_jerk_process_noise();

  // Joint acceleration filter measurement uncertainty covariance.
  eigenmath::VectorXd r(icon::LinearJointAccelerationFilter::kOutputDim);
  r << config.joint_position_measurement_noise(),
      config.joint_velocity_measurement_noise();

  DiscreteAlgebraicRiccatiSolverSettings dare_settings;
  if (config.dare_settings().has_accuracy()) {
    dare_settings.accuracy = config.dare_settings().accuracy();
  }
  if (config.dare_settings().has_max_iterations()) {
    dare_settings.max_iterations = config.dare_settings().max_iterations();
  }
  if (config.dare_settings().has_regularizer_epsilon()) {
    dare_settings.regularizer_epsilon =
        config.dare_settings().regularizer_epsilon();
  }
  if (config.dare_settings().dare_iteration_strategy() ==
      intrinsic_proto::icon::DiscreteAlgebraicRicattiEquationSolutionConfig::
          DARE_ITERATION_STRATEGY_NAIVE) {
    dare_settings.strategy =
        DiscreteAlgebraicRiccatiSolverSettings::IterationStrategy::kNaive;
  } else {
    dare_settings.strategy =
        DiscreteAlgebraicRiccatiSolverSettings::IterationStrategy::kRobust;
  }
  return icon::LinearJointAccelerationFilter::Create(
      1.0 / control_frequency_hz, q.asDiagonal(), r.asDiagonal(), dare_settings,
      P_initial);
}

LinearJointAccelerationFilter::LinearJointAccelerationFilter(
    LinearKalmanFilter kf)
    : u_xd_(eigenmath::VectorXd::Zero(kInputDim)),
      y_xd_(eigenmath::VectorXd::Zero(kOutputDim)),
      state_xd_est_(eigenmath::VectorXd::Zero(kStateDim)),
      filter_(std::move(kf)) {}

void LinearJointAccelerationFilter::GetSystemMatrices(double dt,
                                                      eigenmath::MatrixXd& A,
                                                      eigenmath::MatrixXd& B,
                                                      eigenmath::MatrixXd& C) {
  // Discrete-time dynamics.
  A = eigenmath::MatrixXd::Identity(kStateDim, kStateDim);
  A.topRightCorner(kStateDim - 1, kStateDim - 1).diagonal().setConstant(dt);
  B = eigenmath::MatrixXd::Zero(kStateDim, kInputDim);
  C = eigenmath::MatrixXd::Zero(kOutputDim, kStateDim);
  C.leftCols(kOutputDim).setIdentity();
}

void LinearJointAccelerationFilter::SetInitialState(
    const SingleJointState& j0) {
  state_xd_est_(0) = j0.position;
  state_xd_est_(1) = j0.velocity;
  state_xd_est_(2) = j0.acceleration;
  state_xd_est_(3) = j0.jerk;
  filter_.SetInitialState(state_xd_est_);
}

auto LinearJointAccelerationFilter::Predict(
    const SingleJointState& input_joint_state) -> const SingleJointState& {
  // Transcribe input into dynamic Eigen-type.
  u_xd_(0) = input_joint_state.position;
  u_xd_(1) = input_joint_state.velocity;

  state_xd_est_ = filter_.Predict(u_xd_);

  // Transcribe estimate from dynamic Eigen-type into joint state type.
  joint_state_est_.position = state_xd_est_(0);
  joint_state_est_.velocity = state_xd_est_(1);
  joint_state_est_.acceleration = state_xd_est_(2);
  joint_state_est_.jerk = state_xd_est_(3);

  return joint_state_est_;
}

auto LinearJointAccelerationFilter::Correct(
    const SingleJointState& measured_state) -> const SingleJointState& {
  // Transcribe measurement into dynamic Eigen-type.
  y_xd_(0) = measured_state.position;
  y_xd_(1) = measured_state.velocity;

  state_xd_est_ = filter_.Correct(y_xd_);

  // Transcribe estimate from dynamic Eigen-type into joint state type.
  joint_state_est_.position = state_xd_est_(0);
  joint_state_est_.velocity = state_xd_est_(1);
  joint_state_est_.acceleration = state_xd_est_(2);
  joint_state_est_.jerk = state_xd_est_(3);

  return joint_state_est_;
}

const eigenmath::MatrixXd& LinearJointAccelerationFilter::GetCovariance() {
  return filter_.GetCovariance();
}

const DiscreteAlgebraicRiccatiEquationSolution&
LinearJointAccelerationFilter::GetDareSolution() const {
  return filter_.dare_solution();
}

}  // namespace intrinsic::icon
