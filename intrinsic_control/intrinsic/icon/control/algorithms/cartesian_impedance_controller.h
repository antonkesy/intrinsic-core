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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_CONTROLLER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_CONTROLLER_H_

#include <cstddef>
#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/settling_time_counter.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

// Generic implementation of a Cartesian Impedance Controller, computes
// joint-torques which achieve a desired task-space dynamic behaviour, i.e. the
// end-effector behaves like a second-order dynamical system with user-defined
// spring constant, damping factor and virtual inertia.
class CartesianImpedanceController {
 public:
  CartesianImpedanceController() = delete;
  CartesianImpedanceController(const CartesianImpedanceController& other) =
      delete;
  CartesianImpedanceController& operator=(
      const CartesianImpedanceController& other) = delete;

  // Creates instance of the controller, fails in case of zero number of joints
  // or zero control frequency.
  static absl::StatusOr<std::unique_ptr<CartesianImpedanceController>> Create(
      const kinematics::ModelInterface& kinematics_model, double frequency_hz);

  // Sets a new cartesian target. Does not reset internal states, therefore
  // allows to propagate integrated errors to new targets.
  void SetCartesianTarget(
      const cartesian_impedance::RealTimeCartesianTarget& target);

  void SetNullspaceTarget(
      const cartesian_impedance::RealTimeNullspaceTarget& target);

  void SetAlgorithmConfiguration(
      const cartesian_impedance::AlgorithmConfiguration& config);

  void SetConstraints(
      const cartesian_impedance::RealTimeConstraints& constraints);

  // Processes sensor data and evaluates internal status variables for
  // convergence checking. Needs to be called by an RTCL action in Sense().
  RealtimeStatus PrepareControl(
      const JointStatePVA& joint_state,
      const Wrench& sensed_wrench_in_tip_frame,
      const Wrench& post_sensor_dynamic_load_in_tip_frame);

  // Evaluates feedback law and returns joint torque
  // setpoints. Needs to be called by an RTCL action in Control().
  RealtimeStatusOr<JointStateT> ComputeControl(const JointStatePVA& joint_state,
                                               RigidBodyInterface& dynamics);

  // Evaluates feedback law and returns joint accelerations
  // setpoints. Needs to be called by an RTCL action in Control().
  RealtimeStatusOr<JointStateA> ComputeControl(
      const JointStatePVA& joint_state);

  // Reset internal state.
  void ResetInternalState();

  const std::optional<cartesian_impedance::RealTimeCartesianTarget>&
  GetCartesianTarget() const;

  const std::optional<cartesian_impedance::RealTimeNullspaceTarget>&
  GetNullspaceTarget() const;

  // Returns the sensed wrench at the tool frame expressed in the task frame.
  Wrench SensedWrenchAtToolInTaskFrame() const;

  // Returns the current post-sensor dynamic load at the tool frame, expressed
  // in the task frame.
  Wrench PostSensorDynamicLoadInTaskFrame() const;

  // Returns the l2-norm of the sensed force in [N] at the tool frame expressed
  // in the task frame.
  double SensedForceMagnitude() const;

  // Returns the l2-norm of the sensed torque in [Nm] at the tool frame
  // expressed in the task frame.
  double SensedTorqueMagnitude() const;

  // A convenience method to return the current task_t_robot_tip transform,
  // serves to save re-evaluation of the forward kinematics at action-level.
  Pose3d GetRobotTaskToTip() const;

  // A convenience method to return the current task_t_robot_tool transform,
  // serves to save re-evaluation of the forward kinematics at action-level.
  Pose3d GetRobotTaskToTool() const;

  // A convenience method to return the current Jacobian matrix, serves
  // to save re-evaluation of the forward kinematics at action-level. Returns
  // error status if Jacobian is not available.
  RealtimeStatusOr<eigenmath::Matrix6Nd> GetJacobian() const;

  // A convenience method to return the current Jacobian pseudo-inverse, serves
  // to save re-evaluation of the forward kinematics at action-level. Returns
  // error status if Jacobian is not available.
  RealtimeStatusOr<eigenmath::MatrixNMd> GetJacobianPinv() const;

  eigenmath::Vector6d GetCartesianAccelerationCommand() const;

  // A convenience method to return the current end-effector twist expressed in
  // the base frame, serves to avoid re-evaluation of the forward kinematics at
  // action-level.
  const Twist& GetTwist() const;

  // Returns time in seconds for how long the robot state has been settled, i.e.
  // velocities are below user-defined thresholds. Returns a negative number in
  // case robot is not settled.
  double UpdateSettlingTime(const JointStateV& joint_velocity,
                            double translational_velocity_error_threshold,
                            double angular_velocity_error_threshold,
                            double joint_velocity_threshold);

  kinematics::State& GetKinematicsState() { return kinematics_state_; }

  kinematics::ElementId GetTipFrameId() { return tip_frame_id_; }

  cartesian_impedance::CartesianErrorState GetCartesianErrorState() const;

  // Joint torques resultant from the exerted wrench on the robot passed in
  // during PrepareControl via `sensed_wrench_in_tip_frame`.
  RealtimeStatusOr<eigenmath::VectorNd> GetSensedWrenchJointTorques() const;

 private:
  CartesianImpedanceController(
      const kinematics::ModelInterface& kinematics_model,
      kinematics::ElementId tip_frame_id, double frequency_hz);

  // Must be set before first call to PrepareControl();
  std::optional<cartesian_impedance::RealTimeCartesianTarget>
      cartesian_reference_;
  // Must have been set before first call to PrepareControl();
  std::optional<cartesian_impedance::RealTimeNullspaceTarget>
      nullspace_reference_;
  // Must have been set before first call to PrepareControl();
  std::optional<cartesian_impedance::RealTimeConstraints> constraints_;

  cartesian_impedance::AlgorithmConfiguration algorithm_configuration_;

  cartesian_impedance::CartesianErrorState error_state_in_base_frame_;

  kinematics::State kinematics_state_;
  kinematics::ElementId tip_frame_id_;

  const size_t njoints_;
  const double frequency_hz_;

  // Internal variables shared between PrepareControl() and ComputeControl().
  Pose3d base_t_tool_sensed_;
  Twist sensed_twist_in_base_frame_;
  std::optional<eigenmath::Matrix6Nd> jacobian_;  // In the tool frame.
  std::optional<eigenmath::MatrixNMd> jacobian_pinv_ = std::nullopt;
  eigenmath::MatrixNMd jacobian_time_derivative_;
  Wrench sensed_wrench_at_tool_in_task_frame_ = Wrench::ZERO;
  Wrench post_sensor_dynamic_load_in_task_frame_ = Wrench::ZERO;
  Wrench sensed_wrench_in_base_frame_ = Wrench::ZERO;
  eigenmath::Vector6d cartesian_acceleration_command_;
  SettlingTimeCounter settling_time_counter_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_CONTROLLER_H_
