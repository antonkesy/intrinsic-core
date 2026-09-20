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

#ifndef INTRINSIC_KINEMATICS_STATE_H_
#define INTRINSIC_KINEMATICS_STATE_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/state_values.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// The reference frame for the frame velocity/acceleration to be expressed in.
// - `kLocal` expresses the velocity/acceleration in the local frame. This
//    refers to the frame directly attached to the moving part or body frame. If
//    an additional nonzero position offset from this frame has been defined, it
//    will also be considered.
// - `kRobotBase` expresses the velocity/acceleration in the robot base frame.
// - `kLocalRobotBaseAligned` expresses the velocity/acceleration in the local
//    frame of the robot frame whose axis are aligned with the base frame. If an
//    additional nonzero position offset from the local frame has been defined,
//    it will also be considered.
enum class ReferenceFrame {
  kLocal,
  kRobotBase,
  kLocalRobotBaseAligned,
};

// The changegable information of a robot/kinematic system. This includes:
// - Joint position
// - joint velocity
// - joint acceleration
// - joint effort.
// In addition, it provides the interface to the kinematic functions associated
// with these changeable information such as forward and inverse kinematics.
// Each kinematic state is associated with an underlying kinematic model that
// contains the structural information of the kinematic structure (e.g., robot).
// A kinematic model represents a kinematic tree, i.e., it is not a chain where
// we can easily numerate each degree of freedom (dof). The mapping between the
// ids and element id will be provided by the following two functions:
// - icon::RealtimeStatusOr<int> GetDofIndexForElementId(const ElementId& id)
// - icon::RealtimeStatusOr<ElementId> GetElementIdForDofIndex(int dof_index)
//      const
// that are accessible through the model. E.g. of getting the information:
//   state.GetModel->GetDofIndexForElementId()
class State {
 public:
  // Constructor for the kinematic state associated with a kinematic model and
  // use the default state values provided by the model to initialize default
  // dof positions and limits.
  explicit State(const ModelInterface* model);

  // State is move-only.
  State& operator=(State&& other) = default;
  State(State&& other) noexcept = default;

  // Returns transforms of arbitrary kinematic frames with respect to the
  // underlying kinematic structures base frame.
  icon::RealtimeStatusOr<Pose3d> GetTransform(
      const ElementId& kinematic_element_id) const;

  // Returns transforms between two arbitrary elements in kinematic model.
  // Returns NOT_FOUND if any of the ids does not exist in the associated
  // kinematic state.
  icon::RealtimeStatusOr<Pose3d> GetTransform(const ElementId& from,
                                              const ElementId& to) const;

  // Setting and getting state information of the degrees of freedom and joints.
  // Please note that GetDof* returns only the values of the joints that are
  // considered a degree of freedom. GetJoint* returns the joint value of a
  // joint element.
  JointStateP GetDofPositions() const;
  icon::RealtimeStatusOr<double> GetJointPosition(
      const ElementId& joint_id) const;

  JointStateV GetDofVelocities() const;
  icon::RealtimeStatusOr<double> GetJointVelocity(
      const ElementId& joint_id) const;

  JointStateA GetDofAccelerations() const;
  icon::RealtimeStatusOr<double> GetJointAcceleration(
      const ElementId& joint_id) const;

  JointStateT GetDofEfforts() const;
  icon::RealtimeStatusOr<double> GetDofEffort(const ElementId& dof_id) const;

  icon::RealtimeStatus SetStatePV(const JointStatePV& joint_state_pv);
  icon::RealtimeStatus SetStatePVA(const JointStatePVA& joint_state_pva);

  icon::RealtimeStatus SetDofPositions(const JointStateP& dof_positions,
                                       bool check_limits = true);
  icon::RealtimeStatus SetDofPosition(const ElementId& dof_id, double value);

  icon::RealtimeStatus SetDofVelocities(const JointStateV& dof_velocities,
                                        bool check_limits = true);
  icon::RealtimeStatus SetDofVelocity(const ElementId& dof_id, double value);
  void SetDofVelocitiesZero();

  icon::RealtimeStatus SetDofAccelerations(const JointStateA& dof_accelerations,
                                           bool check_limits = true);
  icon::RealtimeStatus SetDofAcceleration(const ElementId& dof_id,
                                          double value);
  void SetDofAccelerationsZero();

  icon::RealtimeStatus SetDofEfforts(const JointStateT& dof_efforts);
  icon::RealtimeStatus SetDofEffort(const ElementId& dof_id, double value);
  void SetDofEffortsZero();

  // Sets the joint configuration to the default configuration of the model
  // associated with this state.
  void SetToDefaultJointPosition();

  // Returns the position, velocity, acceleration and jerk limits of the
  // system.
  JointLimits GetDofLimits() const;

  // Sets the position, velocity, acceleration, and jerk limits of the robot.
  // These limits cannot exceed the system limits of the model.
  icon::RealtimeStatus SetDofLimits(const JointLimits& limits);

  // TODO(b/356320217): Performance optimization via precomputation.
  // Computes the spatial velocity of the target frame `frame_id` and returns
  // its result expressed in the `reference_frame`. Includes the effect on
  // velocity of the offset `robot_frame_p_target` from the frame `frame_id`.
  // The output is a 6-vector of the form [linear_velocity, angular_velocity].
  icon::RealtimeStatusOr<eigenmath::Vector6d> ComputeSpatialFrameVelocity(
      const ElementId& frame_id,
      const eigenmath::Vector3d& robot_frame_p_target,
      ReferenceFrame reference_frame) const;

  // TODO(b/356320217): Performance optimization via precomputation.
  // Computes the spatial acceleration of the target frame `frame_id` and
  // returns its result expressed in the `reference_frame`. Includes the effect
  // on acceleration of the offset `robot_frame_p_target` from the frame
  // `frame_id`. The output is a 6-vector of the form
  // [linear_acceleration, angular_acceleration].
  icon::RealtimeStatusOr<eigenmath::Vector6d> ComputeSpatialFrameAcceleration(
      const ElementId& frame_id,
      const eigenmath::Vector3d& robot_frame_p_target,
      ReferenceFrame reference_frame) const;

  // TODO(b/356320217): Performance optimization via precomputation.
  // Computes the classical acceleration of the target frame `frame_id`
  // and returns its result expressed in the `reference_frame`. Includes the
  // effect on acceleration of the offset `robot_frame_p_target` from the frame
  // `frame_id`. The output is a 6-vector of the form
  // [linear_acceleration, angular_acceleration].
  // Note that the method `ComputeSpatialFrameAcceleration` returns the
  // `frame_id` spatial acceleration, which is the true derivative of the
  // spatial velocity as defined by Roy Featherstone in "Rigid Body Dynamics
  // Algorithms". This method returns the classical acceleration of
  // `frame_id`, which is an apparent derivative of the spatial velocity.
  // The classical acceleration is obtained by adding the Coriolis acceleration
  // to the spatial acceleration.
  // Consider a body rotating at a constant angular velocity about a fixed line.
  // This body has a constant spatial velocity, and thus it has zero spatial
  // acceleration, yet almost every point in the body is travelling in a
  // circular path, and is thus accelerating due to centrifugal/centripetal
  // effects (classical acceleration).
  icon::RealtimeStatusOr<eigenmath::Vector6d> ComputeClassicalFrameAcceleration(
      const ElementId& frame_id,
      const eigenmath::Vector3d& robot_frame_p_target,
      ReferenceFrame reference_frame) const;

  // Computes the (Geometric) Jacobian of a specific joint frame.
  icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeJacobian(
      const ElementId& frame_id,
      const eigenmath::Vector3d& robot_frame_p_target =
          eigenmath::Vector3d::Zero()) const;

  // Computes the (Geometric) Jacobian of a target point attached to a rigid
  // body identified by the `frame_id`. Moreover, the Jacobian here is
  // expressed in/seen from the view frame.
  // In some applications, this view frame is set to be the same as the target
  // frame, i.e. `base_R_view` = base_R_target.
  // base_p_target = base_t_robot_frame * robot_frame_p_target;
  // For example, to get the Jacobian of the tip expressed in the tip frame:
  // ComputeJacobian(tip_frame_id, eigenmath::Vector3d::Zero(),
  //                 base_t_tip.quaternion());
  icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeJacobian(
      const ElementId& frame_id,
      const eigenmath::Vector3d& robot_frame_p_target,
      const eigenmath::Quaterniond& base_R_view) const;

  // Computes the (Geometric) Jacobian of the tip frame, expressed w.r.t.
  // the base frame. Assumes that a single tip is present in the model.
  icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeJacobian() const;

  // Computes the Analytic Jacobian of a target frame attached to a rigid
  // body identified by the `frame_id`, expressed w.r.t. the base frame.
  icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeAnalyticJacobian(
      const ElementId& frame_id,
      const Pose3d& robot_frame_t_target = Pose3d::Identity()) const;

  // Computes the Analytic Jacobian of a target frame attached to a rigid
  // body identified by the `frame_id`. Moreover, the Analytic Jacobian
  // here is expressed in/seen from the view frame.
  // In some applications, this view frame is set to be the same as the target
  // frame, i.e. `base_R_view` = base_R_target.
  // base_t_target = base_t_robot_frame * robot_frame_t_target;
  // For example, to get the Analytic Jacobian of the tip frame
  // (`frame_id` = tip_frame_id and `robot_frame_t_target` =
  // Pose3d::Identity()), expressed in the tip frame
  // (`base_R_view` = base_t_tip.quaternion() with base_t_tip obtained from
  // forward kinematics):
  // ComputeAnalyticJacobian(tip_frame_id, Pose3d::Identity(),
  //                         base_t_tip.quaternion());
  icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeAnalyticJacobian(
      const ElementId& frame_id, const Pose3d& robot_frame_t_target,
      const eigenmath::Quaterniond& base_R_view) const;

  // Computes the Analytic Jacobian of the tip frame, expressed w.r.t.
  // the base frame. Assumes that a single tip is present in the model.
  icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeAnalyticJacobian() const;

  // Returns the underlying kinematic model for which this state is defined.
  const ModelInterface* GetKinematicModel() const;

 private:
  // Pointer to the model encompassing a kinematic systems or kinematic
  // sub-systems for which this state is defined. Please note that kinematic
  // state does not own this model.
  const ModelInterface* model_;

  // Number degrees of freedom.
  int number_dof_;

  StateValues values_;
};
}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_STATE_H_
