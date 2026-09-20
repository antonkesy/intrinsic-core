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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_LINEAR_JOINT_ACCELERATION_FILTER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_LINEAR_JOINT_ACCELERATION_FILTER_H_

#include <cstddef>
#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/linear_kalman_filter.h"
#include "intrinsic/icon/proto/linear_joint_acceleration_filter_config.pb.h"
#include "intrinsic/math/linear_systems/discrete_algebraic_riccati_equation.h"

namespace intrinsic::icon {

// Implements a stationary linear time-invariant recursive filter for
// estimating joint acceleration from joint position and joint velocity data.
// The filter dynamics model is an exclusively noise-driven process. The process
// covariance for jerk should be set intentionally high, such that it can be
// used to "bootstrap" an accurate acceleration estimate. Combining the
// noise-drive jerk process with position and velocity measurements allows to
// reconstruct acceleration in a reasonably accurate way.
//
// Note that this class' methods return const-references, accessing those
// variables after the filter class goes out of scope is an error.
class LinearJointAccelerationFilter {
 public:
  static constexpr size_t kStateDim = 4;   // Joint pos, vel, acc and jerk.
  static constexpr size_t kOutputDim = 2;  // Joint pos and vel are measured.
  static constexpr size_t kInputDim = 2;   // Placeholder for initialization.

  struct SingleJointState {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double jerk = 0.0;
  };

  // Creates a joint acceleration estimator from a time-step `dt`, process
  // covariance matrix `Q`, an observation covariance matrix `R`, optional
  // `dare_settings` for the Kalman filter's discrete Algebraic Riccati Equation
  // recursion and an optional intitial covariance matrix estimate `P_initial`,
  // which defaults to Identity.
  static absl::StatusOr<LinearJointAccelerationFilter> Create(
      double dt, const eigenmath::MatrixXd& Q, const eigenmath::MatrixXd& R,
      const DiscreteAlgebraicRiccatiSolverSettings& dare_settings =
          DiscreteAlgebraicRiccatiSolverSettings(),
      const eigenmath::MatrixXd& P_initial =
          eigenmath::MatrixXd::Identity(kStateDim, kStateDim));

  // Create a joint acceleration estimator from a
  // LinearJointAccelerationFilterConfig proto and an optional initial
  // covariance matrix estimate `P_initial`, which defaults to Identity.
  static absl::StatusOr<LinearJointAccelerationFilter> Create(
      const intrinsic_proto::icon::LinearJointAccelerationFilterConfig& config,
      double control_frequency_hz,
      const eigenmath::MatrixXd& P_initial =
          eigenmath::MatrixXd::Identity(kStateDim, kStateDim));

  void SetInitialState(const SingleJointState& j0);

  const eigenmath::MatrixXd& GetCovariance();

  const DiscreteAlgebraicRiccatiEquationSolution& GetDareSolution() const;

  // Run the linear filter prediction step with the reference joint state as
  // commanded to the joint-controller.
  const SingleJointState& Predict(const SingleJointState& input_joint_state);

  // Run the linear filter correction step using the measured joint state.
  const SingleJointState& Correct(const SingleJointState& measured_state);

 private:
  explicit LinearJointAccelerationFilter(LinearKalmanFilter kf);

  static void GetSystemMatrices(double dt, eigenmath::MatrixXd& A,
                                eigenmath::MatrixXd& B, eigenmath::MatrixXd& C);

  eigenmath::VectorXd u_xd_;  // Dynamic Eigen-type for input transcription.
  eigenmath::VectorXd y_xd_;  // Dynamic Eigen-type for output transcription.
  eigenmath::VectorXd
      state_xd_est_;  // Dynamic Eigen-type for state transcription.
  SingleJointState joint_state_est_;  // The resulting joint state.
  LinearKalmanFilter filter_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_LINEAR_JOINT_ACCELERATION_FILTER_H_
