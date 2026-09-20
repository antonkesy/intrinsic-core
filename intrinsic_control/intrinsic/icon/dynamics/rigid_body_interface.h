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

#ifndef INTRINSIC_ICON_DYNAMICS_RIGID_BODY_INTERFACE_H_
#define INTRINSIC_ICON_DYNAMICS_RIGID_BODY_INTERFACE_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic {
namespace icon {

// Abstract Interface for computing Rigid Body Dynamics of a kinematic
// chain in the ICON framework.
//
// Standard convention for dynamics parameters should follow URDF format
// described in Modern Robotics: Mechanics, Planning, and Control By Kevin M.
// Lynch and Frank C. Park, Chapter 4.2.
//
// Links should be described by the transformation that defines the position and
// orientation of a frame at the link’s center of mass relative to the link’s
// joint frame, the mass [kg] of the link, and the inertia matrix expressed
// about the link center of mass [kg* m^2].
//
// When the kinematic chain has a FLOATING base type, the first 6 dofs of
// the θ,`θ,``θ vectors are represented as three translational joints and three
// rotational joints centered at the base coordinates.
//
// For details structuring additional parameters (such as friction or damping),
// look to the implementation header for your particular solver.
//
// Notation follows the Newton-Euler convention described in
// Modern Robotics: Mechanics, Planning, and Control By Kevin M. Lynch and Frank
// C. Park, Chapter 8.4 eqn (8.77)
//
// τ = M(θ)¨θ + c(θ, ˙θ) + g(θ) + J_t(θ) * F_tip
//
// Where θ,`θ,``θ are the joint coordinates, velocities, and accelerations.
// M(θ) is the mass matrix.
// c(θ, ˙θ) are quadratic velocity forces resulting from the coriolis matrix.
// g(θ) are gravitational forces.
// J_t(θ) is the tool tip jacobian transpose.
// F_tip is the wrench that the end effector is applying to the environment.
// τ is the vector of joint torques.
class RigidBodyInterface {
 public:
  virtual ~RigidBodyInterface() = default;

  // Returns the number of degrees of freedom in the underlying kinematic chain.
  virtual int GetNumDof() const = 0;

  // Sets vector-valued gravitational acceleration (expressed in the base
  // frame). Example: if the base frame's z-axis is pointing upwards, the
  // standard gravitational acceleration vector will be (0,0,-9.81).
  virtual void SetGravity(const eigenmath::Vector3d& gravity) = 0;

  // Returns a unique tip identifier of type ElementId representing the default
  // tip for the current robot model.
  virtual icon::RealtimeStatusOr<kinematics::ElementId> GetDefaultTip()
      const = 0;

  // Evaluates forward kinematics and expresses the pose of the end-effector
  // specified by `frame_id` in the robot base frame.
  virtual icon::RealtimeStatusOr<Pose3d> ComputeForwardKinematics(
      const eigenmath::VectorNd& joint_coordinates,
      kinematics::ElementId frame_id) = 0;

  // Returns the vector of joint_torques τ resulting from the underlying
  // kinematic chain with 'joint_coordinates' θ, 'joint_velocities' `θ, and
  // 'joint_accelerations' ``θ.
  virtual icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeInverseDynamics(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocities,
      const eigenmath::VectorNd& joint_accelerations) = 0;

  // Returns the vector of joint_torques τ resulting from the underlying
  // kinematic chain with 'joint_coordinates' θ, 'joint_velocities' `θ,
  // 'joint_accelerations' ``θ,  and end effector applying a wrench
  // 'wrench_at_tip' at the tool-center-point specified by `frame_id`.
  virtual icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeInverseDynamics(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocities,
      const eigenmath::VectorNd& joint_accelerations,
      const Wrench& wrench_at_tip, kinematics::ElementId frame_id) = 0;

  // Computes the resulting joint_accelerations ``θ for the underlying
  // kinematic chain with 'joint_coordinates' θ, 'joint_velocities' `θ, and
  // 'joint_torques' τ.
  virtual icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeForwardDynamics(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocities,
      const eigenmath::VectorNd& joint_torques) = 0;

  // Computes the joint space inertia matrix inverse' M^{-1}(θ) for the
  // underlying kinematic chain with 'joint_coordinates' θ.
  // Parameter 'inertia_matrix_inverse' should be dimension [dim(θ), dim(θ)].
  virtual icon::RealtimeStatusOr<eigenmath::MatrixNd>
  ComputeJointSpaceInertiaMatrixInverse(
      const eigenmath::VectorNd& joint_coordinates) = 0;

  // Computes the Jacobian J(θ) expressed in the base frame for the underlying
  // kinematic chain with 'joint_coordinates' θ and frame of interest with
  // `frame_id`. The `frame_to_target_position_offset` is the constant offset
  // that describes the relative position of the tool target frame to the
  // `frame_id`. The `frame_to_target_position_offset` offset is non-zero when
  // there is not a `frame_id` that describes the frame at which the target is
  // located. In this case, we can use the `frame_to_target_position_offset`
  // constant offset to describe the relative pose from the known available
  // frame at `frame_id` to the target frame.
  virtual icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeJacobian(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::Vector3d& frame_to_target_position_offset,
      kinematics::ElementId frame_id) = 0;

  // Computes the first derivative with respect to time of the Jacobian J(θ).
  // It is expressed in a LOCAL_WORLD_ALIGNED frame, meaning that the frame
  // position is local at `frame_id`, but the axis are aligned with the world
  // frame. The `joint_coordinates` and `joint_velocity` express the current
  // joint position and velocity where the Jacobian time derivative should be
  // computed. The `frame_to_target_position_offset` is the constant offset that
  // describes the relative position of the tool target frame to the `frame_id`.
  // The `frame_to_target_position_offset` offset is non-zero when there is not
  // a `frame_id` that describes the frame at which the target is located. In
  // this case, we can use the `frame_to_target_position_offset` constant offset
  // to describe the relative pose from the known available frame at `frame_id`
  // to the target frame. Note that this function returns the matrix 'dJ/dt'
  // that would be used for computing the classical acceleration of the point of
  // interest at `frame_id` with the given `frame_to_target_position_offset`.
  virtual icon::RealtimeStatusOr<eigenmath::Matrix6Nd>
  ComputeJacobianTimeDerivative(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocity,
      const eigenmath::Vector3d& frame_to_target_position_offset,
      kinematics::ElementId frame_id) = 0;

  // Computes the vector resulting from the matrix product 'dJ/dt * dq/dt',
  // where 'dJ/dt' is the Jacobian's time derivative (computed for example with
  // ComputeJacobianTimeDerivative). It is expressed in a LOCAL_WORLD_ALIGNED
  // frame, meaning that the frame position is local at `frame_id`, but the axis
  // are aligned with the world frame. The `joint_coordinates` and
  // `joint_velocity` express the current joint position and velocity where the
  // Jacobian time derivative times joint velocity should be computed. The
  // `frame_to_target_position_offset` is the constant offset that describes the
  // relative position of the tool target frame to the `frame_id`. The
  // `frame_to_target_position_offset` offset is non-zero when there is not a
  // `frame_id` that describes the frame at which the target is located. In this
  // case, we can use the `frame_to_target_position_offset` constant offset to
  // describe the relative pose from the known available frame at `frame_id` to
  // the target frame. Note that this function returns the vector 'dJ/dt *
  // dq/dt' that would be used for computing the classical acceleration of the
  // point of interest at `frame_id` with the given
  // `frame_to_target_position_offset`.
  virtual icon::RealtimeStatusOr<eigenmath::Vector6d>
  ComputeJacobianTimeDerivativeVector(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocity,
      const eigenmath::Vector3d& frame_to_target_position_offset,
      kinematics::ElementId frame_id) = 0;

  // Computes the generalized gravity vector of gravity forces resulting from
  // g(θ) with 'joint_coordinates' θ.
  virtual icon::RealtimeStatusOr<eigenmath::VectorNd>
  ComputeGeneralizedGravityVector(
      const eigenmath::VectorNd& joint_coordinates) = 0;

  // Returns the Coriolis vector resulting from the coriolis matrix c(θ, ˙θ)
  // with 'joint_coordinates' θ and 'joint_velocities' `θ.
  virtual icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeCoriolisVector(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocities) = 0;

  // Computes the joint space inertia matrix resulting from the joint space
  // inertia matrix M(θ) with 'joint_coordinates' θ.
  virtual icon::RealtimeStatusOr<eigenmath::MatrixNd>
  ComputeJointSpaceInertiaMatrix(
      const eigenmath::VectorNd& joint_coordinates) = 0;

  // Updates the kinematic chain end effector with tip id 'tip_id's
  // dynamic parameters to have a parent frame to tool-center-of-gravity
  // transformation 'tip_t_cog' (expressed in the body frame), with link mass
  // 'mass'[kg], and a 3x3 symmetric inertia matrix 'inertia_matrix' [kg * m^2]
  // (expressed about the link center of mass).
  //
  // Returns icon::RealTimeStatus::OK on success.
  // Returns icon::RealTimeStatus::UnimplementedError if not supported by the
  // underlying implementation.
  // May return other errors as well.
  virtual icon::RealtimeStatus SetEndEffectorDynamicsParameters(
      const Pose3d& tip_t_cog, double mass,
      const eigenmath::Matrix3d& inertia_matrix,
      kinematics::ElementId tip_id) = 0;

  // Updates the kinematic chain end effector with tip id 'tip_id's
  // dynamic parameters to have a parent frame to tool-center-of-gravity
  // translation 'tip_t_cog' (expressed in the body frame) and identity
  // orientation, and with link mass 'mass'[kg]. The values of the inertia
  // matrix are not changed. Use the full signature to modify its values by
  // providing a rotation as well as the appropriate inertial values.
  //
  // Returns icon::RealTimeStatus::OK on success.
  // Returns icon::RealTimeStatus::UnimplementedError if not supported by the
  // underlying implementation.
  // May return other errors as well.
  virtual icon::RealtimeStatus SetEndEffectorDynamicsParameters(
      const eigenmath::Vector3d& tip_t_cog, double mass,
      kinematics::ElementId tip_id) = 0;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_DYNAMICS_RIGID_BODY_INTERFACE_H_
