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

#ifndef INTRINSIC_ICON_DYNAMICS_ROBOTICS_LIBRARY_DYNAMICS_H_
#define INTRINSIC_ICON_DYNAMICS_ROBOTICS_LIBRARY_DYNAMICS_H_

#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"
#include "rl/mdl/Dynamic.h"

namespace intrinsic::icon {

class RoboticsLibraryDynamics : public RigidBodyInterface {
 public:
  RoboticsLibraryDynamics() = delete;
  explicit RoboticsLibraryDynamics(
      std::unique_ptr<::rl::mdl::Dynamic> dynamic,
      std::unique_ptr<const kinematics::ModelInterface> model_interface)
      : dynamic_(std::move(dynamic)),
        model_interface_(std::move(model_interface)) {}

  ~RoboticsLibraryDynamics() override = default;

  int GetNumDof() const override { return dynamic_->getDof(); }

  void SetGravity(const eigenmath::Vector3d& gravity) override;

  icon::RealtimeStatusOr<kinematics::ElementId> GetDefaultTip() const override;

  icon::RealtimeStatusOr<Pose3d> ComputeForwardKinematics(
      const eigenmath::VectorNd& joint_coordinates,
      kinematics::ElementId frame_id) override;

  icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeInverseDynamics(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocities,
      const eigenmath::VectorNd& joint_accelerations) override;

  icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeInverseDynamics(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocities,
      const eigenmath::VectorNd& joint_accelerations,
      const Wrench& wrench_at_tip, kinematics::ElementId frame_id) override;

  icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeForwardDynamics(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocities,
      const eigenmath::VectorNd& joint_torques) override;

  icon::RealtimeStatusOr<eigenmath::MatrixNd>
  ComputeJointSpaceInertiaMatrixInverse(
      const eigenmath::VectorNd& joint_coordinates) override;

  icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeJacobian(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::Vector3d& frame_to_target_position_offset,
      kinematics::ElementId frame_id) override;

  icon::RealtimeStatusOr<eigenmath::Matrix6Nd> ComputeJacobianTimeDerivative(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocity,
      const eigenmath::Vector3d& frame_to_target_position_offset,
      kinematics::ElementId frame_id) override;

  // By default, the dynamics library propagates the accelerations from body to
  // body by setting the gravity vector to zero. Thus, its results will match
  // the same results from Pinocchio. At the end, the gravity vector is returned
  // to its original value, in case other functions computing dynamic quantities
  // depend on it.
  icon::RealtimeStatusOr<eigenmath::Vector6d>
  ComputeJacobianTimeDerivativeVector(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocity,
      const eigenmath::Vector3d& frame_to_target_position_offset,
      kinematics::ElementId frame_id) override;

  icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeGeneralizedGravityVector(
      const eigenmath::VectorNd& joint_coordinates) override;

  icon::RealtimeStatusOr<eigenmath::VectorNd> ComputeCoriolisVector(
      const eigenmath::VectorNd& joint_coordinates,
      const eigenmath::VectorNd& joint_velocities) override;

  icon::RealtimeStatusOr<eigenmath::MatrixNd> ComputeJointSpaceInertiaMatrix(
      const eigenmath::VectorNd& joint_coordinates) override;

  icon::RealtimeStatus SetEndEffectorDynamicsParameters(
      const Pose3d& tip_t_cog, double mass,
      const eigenmath::Matrix3d& inertia_matrix,
      kinematics::ElementId frame_id) override;

  icon::RealtimeStatus SetEndEffectorDynamicsParameters(
      const eigenmath::Vector3d& tip_t_cog, double mass,
      kinematics::ElementId tip_id) override;

 private:
  std::unique_ptr<::rl::mdl::Dynamic> dynamic_;
  std::unique_ptr<const kinematics::ModelInterface> model_interface_;
};
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_DYNAMICS_ROBOTICS_LIBRARY_DYNAMICS_H_
