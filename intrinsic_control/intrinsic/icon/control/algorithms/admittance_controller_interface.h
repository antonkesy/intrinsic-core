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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_ADMITTANCE_CONTROLLER_INTERFACE_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_ADMITTANCE_CONTROLLER_INTERFACE_H_

#include <optional>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

// Abstract interface class for Cartesian Admittance control algorithms.
class AdmittanceControllerInterface {
 public:
  virtual ~AdmittanceControllerInterface() = default;

  virtual void SetCartesianTarget(
      const cartesian_impedance::RealTimeCartesianTarget& target) = 0;

  virtual void SetNullspaceTarget(
      const cartesian_impedance::RealTimeNullspaceTarget& target) = 0;

  virtual void SetAlgorithmConfiguration(
      const cartesian_impedance::AlgorithmConfiguration& config) = 0;

  virtual RealtimeStatus SetConstraints(
      const cartesian_impedance::RealTimeConstraints& constraints) = 0;

  // Processes sensor data and evaluates internal status variables for
  // convergence checking. Needs to be called by an RTCL action in Sense().
  virtual RealtimeStatus PrepareControl(
      kinematics::ElementId tip_id, const Wrench& sensed_wrench_in_tip_frame,
      const Wrench& post_sensor_dynamic_load_in_tip_frame,
      kinematics::State& state) = 0;

  // Evaluates feedback law and returns joint position and velocity control
  // setpoints. Needs to be called by an RTCL action in Control().
  virtual RealtimeStatusOr<JointStatePVA> ComputeControl(
      const kinematics::InverseKinematicsInterface* ik,
      const Wrench& sensed_wrench_in_tip_frame) = 0;

  // Resets the internal state of the controller to the provided values. Can be
  // called by an RTCL action's OnEnter() method.
  virtual RealtimeStatus ResetInternalState(
      const Pose3d& previously_commanded_base_t_target,
      const Twist& previously_commanded_twist_in_base_frame,
      const Acceleration& previously_commanded_acceleration_in_base_frame,
      const JointStatePVA& previously_commanded_joint_state) = 0;

  // Recomputes the current settling time based on provided thresholds. This
  // function is free to propagate internal state, multiple calls to this
  // function during a single control cycle are not expected to return the same
  // result.
  virtual double UpdateSettlingTime(
      const JointStateV& joint_velocity,
      double translational_velocity_error_threshold,
      double angular_velocity_error_threshold,
      double joint_velocity_threshold) = 0;

  // Returns the current Cartesian target if set, and nullopt otherwise.
  virtual const std::optional<cartesian_impedance::RealTimeCartesianTarget>&
  GetCartesianTarget() const = 0;

  // Returns the current nullspace target if set, and nullopt otherwise.
  virtual const std::optional<cartesian_impedance::RealTimeNullspaceTarget>&
  GetNullspaceTarget() const = 0;

  // Returns the l2-norm of the force being sensed at the target frame,
  // expressed in the base frame in [N].
  virtual double SensedForceMagnitude() const = 0;

  // Returns the l2-norm of the torque being sensed at the target frame,
  // expressed in the base frame in [Nm].
  virtual double SensedTorqueMagnitude() const = 0;

  // A convenience method to return the current base_t_tool transform, serves
  // to save re-evaluation of the forward kinematics at action-level.
  virtual Pose3d GetRobotTaskToTool() const = 0;

  // A convenience method to return the current base_t_tip transform, serves
  // to save re-evaluation of the forward kinematics at action-level.
  virtual Pose3d GetRobotTaskToTip() const = 0;

  // A convenience method to return the current nominal (target) end-effector
  // twist expressed in the base frame, serves to avoid re-evaluation of the
  // forward kinematics at action-level.
  virtual Twist GetTwist() const = 0;

  // A convenience method to return the current Jacobian matrix, serves
  // to save re-evaluation of the forward kinematics at action-level. Returns
  // error status if Jacobian is not available.
  virtual RealtimeStatusOr<eigenmath::Matrix6Nd> GetJacobian() const = 0;

  // Returns the current Cartesian error of the controller, expressed in the
  // base-aligned coordinate system.
  virtual cartesian_impedance::CartesianErrorState GetCartesianErrorState()
      const = 0;

  // Returns the current Cartesian acceleration command expressed in the base
  // frame.
  virtual eigenmath::Vector6d GetCartesianAccelerationCommand() const = 0;

  // Returns the current sensed wrench at the tool frame, expressed in the
  // task frame.
  virtual Wrench GetSensedWrenchAtToolInTaskFrame() const = 0;

  // Returns the current post-sensor dynamic load at the tool frame, expressed
  // in the task frame.
  virtual Wrench GetPostSensorDynamicLoadInTaskFrame() const = 0;

  virtual JointStatePVA GetJointSpaceMotionReference() const = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_ADMITTANCE_CONTROLLER_INTERFACE_H_
