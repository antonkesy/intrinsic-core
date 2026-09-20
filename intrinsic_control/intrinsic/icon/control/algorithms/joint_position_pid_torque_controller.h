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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_POSITION_PID_TORQUE_CONTROLLER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_POSITION_PID_TORQUE_CONTROLLER_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/proto/joint_position_pid_torque_controller_config.pb.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace icon {

// PID controller to convert cyclic position setpoints to torque setpoints.
class JointPositionPidTorqueController {
 public:
  struct Params {
    // Proportional gain for the controller.
    double kp = 0;
    // Integral gain for the controller. This only takes effect when the
    // absolute position error is below integration_enable_range.
    double ki = 0;
    // Derivative gain for the controller.
    double kd = 0;
    // Maximum absolute permissible torque. This must be positive and the output
    // will be clamped to this value.
    double max_torque = 0;
    // Value between 0 and 1 determining the velocity smoothing. Higher value
    // corresponds to less smoothing.
    double alpha = 0;
    // The maximum absolute change in torque between timesteps. The change in
    // torque between timesteps will be clamped to this amount.
    double max_torque_delta_per_timestep = 0;
    // The maximum contribution to the target torque from the integral term. The
    // magnitude will be clamped to this value.
    double max_integral_torque = 0;
    // The length of rolling window for which the integral term is evaluated in
    // seconds.
    double integration_window_seconds = 0;
    // The maximum absolute position error for which the integral term is used.
    double integration_enable_range = 0;
    // The timestep between measurements, used to scale the integral term.
    double timestep = 1;
  };

  struct State {
    explicit State(int integral_buffer_size) {
      // initialize the integral vector to be 0s of size integral_buffer_size
      integral.resize(integral_buffer_size, 0);
      integral_itr = integral.begin();
    }
    double smoothed_velocity_estimate = 0;
    double integral_control_effort = 0;
    std::optional<double> target_torque = std::nullopt;
    std::vector<double>::iterator integral_itr;
    std::vector<double> integral;
  };

  static absl::StatusOr<std::unique_ptr<JointPositionPidTorqueController>>
  Create(intrinsic_proto::icon::JointPositionPidTorqueControllerConfig config);

  // Returns true when the torque setpoint was calculated successfully and
  // state_.target_torque was updated. Returns false otherwise.
  bool CalculateFeedbackSetPoints(const JointPositionCommand& input_command,
                                  const JointStatePV& state);

  double GetTargetTorque();

  // Resets any internal state (integrator, velocity smoothing, etc.).
  void Reset();

 protected:
  explicit JointPositionPidTorqueController(Params params);
  Params params_;
  State state_;
};
}  // namespace icon
}  // namespace intrinsic
#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_POSITION_PID_TORQUE_CONTROLLER_H_
