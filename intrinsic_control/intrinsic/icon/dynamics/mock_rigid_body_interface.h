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

#ifndef INTRINSIC_ICON_DYNAMICS_MOCK_RIGID_BODY_INTERFACE_H_
#define INTRINSIC_ICON_DYNAMICS_MOCK_RIGID_BODY_INTERFACE_H_

#include <gmock/gmock.h>

#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

class MockRigidBodyInterface : public RigidBodyInterface {
 public:
  MOCK_METHOD(int, GetNumDof, (), (const, override));

  MOCK_METHOD(void, SetGravity, (const eigenmath::Vector3d& gravity),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<kinematics::ElementId>, GetDefaultTip, (),
              (const, override));

  MOCK_METHOD(icon::RealtimeStatusOr<Pose3d>, ComputeForwardKinematics,
              (const eigenmath::VectorNd& joint_coordinates,
               kinematics::ElementId frame_id),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::VectorNd>,
              ComputeInverseDynamics,
              (const eigenmath::VectorNd& joint_coordinates,
               const eigenmath::VectorNd& joint_velocies,
               const eigenmath::VectorNd& joint_accelerations),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::VectorNd>,
              ComputeInverseDynamics,
              (const eigenmath::VectorNd& joint_coordinates,
               const eigenmath::VectorNd& joint_velocies,
               const eigenmath::VectorNd& joint_accelerations,
               const Wrench& wrench_at_tip, kinematics::ElementId frame_id),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::VectorNd>,
              ComputeForwardDynamics,
              (const eigenmath::VectorNd& joint_coordinates,
               const eigenmath::VectorNd& joint_velocies,
               const eigenmath::VectorNd& joint_torques),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::MatrixNd>,
              ComputeJointSpaceInertiaMatrixInverse,
              (const eigenmath::VectorNd& joint_coordinates), (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::Matrix6Nd>, ComputeJacobian,
              (const eigenmath::VectorNd& joint_coordinates,
               const eigenmath::Vector3d& frame_to_target_position_offset,
               kinematics::ElementId frame_id),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::Matrix6Nd>,
              ComputeJacobianTimeDerivative,
              (const eigenmath::VectorNd& joint_coordinates,
               const eigenmath::VectorNd& joint_velocity,
               const eigenmath::Vector3d& frame_to_target_position_offset,
               kinematics::ElementId frame_id),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::Vector6d>,
              ComputeJacobianTimeDerivativeVector,
              (const eigenmath::VectorNd& joint_coordinates,
               const eigenmath::VectorNd& joint_velocity,
               const eigenmath::Vector3d& frame_to_target_position_offset,
               kinematics::ElementId frame_id),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::VectorNd>,
              ComputeCoriolisVector,
              (const eigenmath::VectorNd& joint_coordinates,
               const eigenmath::VectorNd& joint_velocies),
              (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::VectorNd>,
              ComputeGeneralizedGravityVector,
              (const eigenmath::VectorNd& joint_coordinates), (override));

  MOCK_METHOD(icon::RealtimeStatusOr<eigenmath::MatrixNd>,
              ComputeJointSpaceInertiaMatrix,
              (const eigenmath::VectorNd& joint_coordinates), (override));

  MOCK_METHOD(icon::RealtimeStatus, SetEndEffectorDynamicsParameters,
              (const Pose3d& parent_t_cog, double mass,
               const eigenmath::Matrix3d& inertia_matrix,
               kinematics::ElementId tip_id),
              (override));

  MOCK_METHOD(icon::RealtimeStatus, SetEndEffectorDynamicsParameters,
              (const eigenmath::Vector3d& tip_t_cog, double mass,
               kinematics::ElementId tip_id),
              (override));
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_DYNAMICS_MOCK_RIGID_BODY_INTERFACE_H_
