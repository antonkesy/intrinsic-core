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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_ADMITTANCE_CONTROLLER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_ADMITTANCE_CONTROLLER_H_

#include <cstddef>
#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/admittance_controller_interface.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/settling_time_counter.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

// Generic implementation of a Cartesian Admittance Controller. Computes
// both joint position and joint velocity updates and can therefore be used in
// position, velocity and position/velocity actions.
class CartesianAdmittanceController : public AdmittanceControllerInterface {
 public:
  CartesianAdmittanceController() = delete;
  CartesianAdmittanceController(const CartesianAdmittanceController& other) =
      delete;
  CartesianAdmittanceController& operator=(
      const CartesianAdmittanceController& other) = delete;

  // Creates instance of the controller, fails in case of zero number of joints
  // or zero control frequency.
  static absl::StatusOr<std::unique_ptr<CartesianAdmittanceController>> Create(
      size_t njoints, double frequency_hz);

  void SetCartesianTarget(
      const cartesian_impedance::RealTimeCartesianTarget& target) override;

  void SetNullspaceTarget(
      const cartesian_impedance::RealTimeNullspaceTarget& target) override;

  void SetAlgorithmConfiguration(
      const cartesian_impedance::AlgorithmConfiguration& config) override;

  // Returns error status if any of the joint velocity or acceleration limits
  // contains an 'inf' value.
  RealtimeStatus SetConstraints(
      const cartesian_impedance::RealTimeConstraints& constraints) override;

  // Processes sensor data and evaluates internal status variables for
  // convergence checking. Needs to be called by an RTCL action in Sense().
  //
  // Fails if there are no joints in `chain`.
  RealtimeStatus PrepareControl(
      kinematics::ElementId tip_id, const Wrench& sensed_wrench_in_tip_frame,
      const Wrench& post_sensor_dynamic_load_in_tip_frame,
      kinematics::State& state) override;

  // Evaluates feedback law and returns joint position and velocity control
  // setpoints. Needs to be called by an RTCL action in Control().
  RealtimeStatusOr<JointStatePVA> ComputeControl(
      const kinematics::InverseKinematicsInterface* /*ik*/,
      const Wrench& /*sensed_wrench_in_tip_frame*/) override;

  // Resets the internal state, expects user to set the
  // `previously_commanded_joint_state`, which will be used to initialize
  // Jacobians for `chain`.
  RealtimeStatus ResetInternalState(
      const Pose3d& previously_commanded_base_t_tool,
      const Twist& previously_commanded_twist_in_base_frame,
      const Acceleration& previously_commanded_acceleration_in_base_frame,
      const JointStatePVA& previously_commanded_joint_state) override;

  // Returns time in seconds for how long the robot state has been settled, i.e.
  // velocities are below user-defined thresholds. In case the robot gets
  // "unsettled", i.e. moved, the counter returns negative cycle time,
  // -1.0/frequency_hz.
  double UpdateSettlingTime(const JointStateV& joint_velocity,
                            double translational_velocity_error_threshold,
                            double angular_velocity_error_threshold,
                            double joint_velocity_threshold) override;

  const std::optional<cartesian_impedance::RealTimeCartesianTarget>&
  GetCartesianTarget() const override;

  const std::optional<cartesian_impedance::RealTimeNullspaceTarget>&
  GetNullspaceTarget() const override;

  // Returns the l2-norm of the force being sensed at the target frame,
  // expressed in the base frame in [N].
  double SensedForceMagnitude() const override;

  // Returns the l2-norm of the torque being sensed at the target frame,
  // expressed in the base frame in [Nm].
  double SensedTorqueMagnitude() const override;

  // A convenience method to return the current base_t_target transform, serves
  // to save re-evaluation of the forward kinematics at action-level.
  Pose3d GetRobotTaskToTool() const override;

  // A convenience method to return the current base_t_tip transform, serves
  // to save re-evaluation of the forward kinematics at action-level.
  Pose3d GetRobotTaskToTip() const override;

  // A convenience method to return the current nominal (target) end-effector
  // twist expressed in the base frame, serves to avoid re-evaluation of the
  // forward kinematics at action-level.
  Twist GetTwist() const override;

  // A convenience method to return the current Jacobian matrix, serves
  // to save re-evaluation of the forward kinematics at action-level. Returns
  // error status if Jacobian is not available.
  RealtimeStatusOr<eigenmath::Matrix6Nd> GetJacobian() const override;

  cartesian_impedance::CartesianErrorState GetCartesianErrorState()
      const override;

  eigenmath::Vector6d GetCartesianAccelerationCommand() const override {
    return cartesian_acceleration_command_in_base_frame_;
  }

  Wrench GetSensedWrenchAtToolInTaskFrame() const override {
    return sensed_wrench_at_tool_in_task_frame_;
  }

  Wrench GetPostSensorDynamicLoadInTaskFrame() const override {
    return post_sensor_dynamic_load_in_task_frame_;
  }

  JointStatePVA GetJointSpaceMotionReference() const override {
    return joint_space_motion_reference_;
  }

 private:
  CartesianAdmittanceController(size_t njoints, double frequency_hz);

  // Must be set before first call to PrepareControl();
  std::optional<cartesian_impedance::RealTimeCartesianTarget>
      cartesian_reference_;
  // Must be set before first call to PrepareControl();
  std::optional<cartesian_impedance::RealTimeNullspaceTarget>
      nullspace_reference_;
  // Must be set before first call to PrepareControl();
  std::optional<cartesian_impedance::RealTimeConstraints> constraints_;

  cartesian_impedance::AlgorithmConfiguration algorithm_configuration_;

  cartesian_impedance::CartesianErrorState error_state_in_base_frame_;

  const size_t njoints_;
  const double frequency_hz_;
  // Cartesian acceleration command wrt robot base frame as computed by the
  // task-space admittance control law.
  eigenmath::Vector6d cartesian_acceleration_command_in_base_frame_;
  // Motion reference which gets returned to a calling action.
  JointStatePVA joint_space_motion_reference_;
  // Internal variables shared between PrepareControl() and ComputeControl().
  Pose3d base_t_tool_nominal_;
  Twist nominal_twist_in_base_frame_ = Twist::ZERO;
  std::optional<eigenmath::Matrix6Nd> jacobian_;
  std::optional<eigenmath::MatrixNMd> jacobian_pinv_ = std::nullopt;
  eigenmath::MatrixNMd jacobian_pinv_time_derivative_;
  Wrench sensed_wrench_at_tool_in_task_frame_ = Wrench::ZERO;
  Wrench post_sensor_dynamic_load_in_task_frame_ = Wrench::ZERO;

  SettlingTimeCounter settling_time_counter_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_ADMITTANCE_CONTROLLER_H_
