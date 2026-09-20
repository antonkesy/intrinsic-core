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

#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/force_sensor_controller_migration_utils.h"

#include <cstdlib>

#include "intrinsic/eigenmath/skew_symmetric_matrix.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/constants.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/pluecker_transform.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

PostSensorPayload PostSensorPayload::operator+(
    const PostSensorPayload& other) const {
  const double total_mass = this->mass_kg + other.mass_kg;

  const eigenmath::Vector3d new_cog =
      (std::abs(total_mass) < kPayloadChangePrecision)
          ? eigenmath::Vector3d::Zero()
          : eigenmath::Vector3d((this->mass_kg * this->ft_t_cog +
                                 other.mass_kg * other.ft_t_cog) /
                                total_mass);
  return PostSensorPayload{.ft_t_cog = new_cog, .mass_kg = total_mass};
}

Wrench ComputeWrenchFtSupportMass(double support_mass_kg,
                                  const Pose3d& robot_T_ft,
                                  const eigenmath::Vector3d& ft_p_cog) {
  // We construct a temporary frame "cog_world_aligned" which is located at the
  // CoG of the support mass (tool_cog) and is aligned with the world frame in
  // terms of orientation. We express the wrench at tool_cog represented in
  // world orientation.
  Wrench wrench_at_support_mass_world_aligned = Wrench::ZERO;
  wrench_at_support_mass_world_aligned.head<3>() =
      support_mass_kg * intrinsic::kDefaultGravityVector;

  // Compute orientation of the world w.r.t. force sensor frame.
  eigenmath::Quaterniond ft_R_robot = robot_T_ft.quaternion().conjugate();

  // Express the pose of the temporary frame "cog_world_aligned" w.r.t. the
  // force sensor frame.
  Pose3d ft_T_cog_world_aligned = Pose3d(ft_R_robot, ft_p_cog);

  // Compute the 6d transformation matrix H for transforming wrenches from
  // tool_cog to the force sensor frame .
  Eigen::Matrix<double, 6, 6> ft_H_cog_world_aligned =
      PlueckerForceTransform(ft_T_cog_world_aligned);

  // Return wrench on the F/T sensor due to support mass.
  return Wrench(ft_H_cog_world_aligned * wrench_at_support_mass_world_aligned);
}

Wrench ComputePostSensorDynamicLoadAtFTSensor(
    const Twist& ft_twist, const eigenmath::Vector6d& ft_acceleration,
    double support_mass_kg, const intrinsic::Pose3d& robot_T_ft,
    const eigenmath::Vector3d& ft_p_cog) {
  // Pose of the CoG of the support mass w.r.t. the force-torque sensor frame.
  intrinsic::Pose3d ft_T_cog(ft_p_cog);

  // Motion transform matrix from FT-sensor frame to a sensor-aligned frame
  // located at the CoG.
  const eigenmath::Matrix6d cog_M_ft = PlueckerMotionTransformInverse(ft_T_cog);

  // Force transform matrix from a sensor-aligned frame sitting in the CoG to
  // the FT-sensor frame.
  const eigenmath::Matrix6d ft_F_cog = PlueckerForceTransform(ft_T_cog);

  // STEP 1 - reaction force due to purely linear translational acceleration.
  // We need to transform the 6-d spatial acceleration expressed at the
  // FT-sensor frame to a spatial acceleration vector expressed at a coordinate
  // system located at the CoG of the support mass, but aligned with the
  // FT-sensor frame.
  eigenmath::Vector6d cog_acceleration_ft_aligned = cog_M_ft * ft_acceleration;
  Wrench cog_translational_load_ft_aligned = Wrench::ZERO;
  cog_translational_load_ft_aligned.head<3>() =
      cog_acceleration_ft_aligned.head<3>() * support_mass_kg;

  // The pure translational dynamic force at the CoG becomes a wrench at the
  // FT-sensor.
  Wrench ft_translational_load(ft_F_cog * cog_translational_load_ft_aligned);

  // STEP 2 - reaction force due to pure angular acceleration at the FT-sensor
  // frame (external load is not assumed to have a full inertia tensor, it is
  // only a point mass). The rotational 3d inertia matrix is diagonal and
  // consists of the support_mass multiplied by the inertia tensor of an offset
  // point-mass (see parallel axes theorem).
  Wrench ft_angular_load = Wrench::ZERO;
  ft_angular_load.tail<3>() =
      support_mass_kg * eigenmath::SkewSymmetricMatrix(ft_p_cog).transpose() *
      eigenmath::SkewSymmetricMatrix(ft_p_cog) * ft_acceleration.tail<3>();

  // STEP 3 - centrifugal forces due to angular velocity of the support mass CoM
  Twist cog_twist_ft_aligned(cog_M_ft * ft_twist);
  eigenmath::Vector3d omega_local_cog_ft_aligned =
      cog_twist_ft_aligned.tail<3>();
  Pose3d cog_T_robot_ft_aligned = (robot_T_ft * ft_T_cog).inverse();
  Wrench cog_centrifugal_load_ft_aligned = Wrench::ZERO;
  cog_centrifugal_load_ft_aligned.head<3>() =
      -support_mass_kg *
      omega_local_cog_ft_aligned.cross((omega_local_cog_ft_aligned.cross(
          cog_T_robot_ft_aligned.translation())));

  // The pure translational centrifugal force at the CoG becomes a wrench at the
  // FT-sensor.
  Wrench ft_centrifugal_load(ft_F_cog * cog_centrifugal_load_ft_aligned);

  return Wrench(ft_translational_load + ft_angular_load + ft_centrifugal_load);
}

RealtimeStatusOr<ComputePostSensorDynamicsResult> ComputePostSensorDynamicLoad(
    const eigenmath::VectorNd& current_joint_velocity,
    const eigenmath::VectorNd& current_joint_acceleration,
    kinematics::ElementId target_link_element_id, const Pose3d& base_t_target,
    const Pose3d& target_t_ft_sensor,
    const PostSensorPayload& total_post_sensor_payload,
    kinematics::State* kinematics_state) {
  // Jacobian of the target frame in base (robot) coordinates.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::MatrixNMd jacobian,
      kinematics_state->ComputeJacobian(target_link_element_id));

  // Robot base rotation expressed in target frame.
  eigenmath::Matrix3d target_R_robot = base_t_target.inverse().rotationMatrix();
  // Compute the target frame Jacobian expressed in the target frame by
  // rotating the Jacobian of the target frame expressed in base frame.
  eigenmath::Matrix6Nd target_jacobian_target(6, jacobian.cols());
  target_jacobian_target.topRows<3>() =
      target_R_robot.lazyProduct(jacobian.topRows<3>());
  target_jacobian_target.bottomRows<3>() =
      target_R_robot.lazyProduct(jacobian.bottomRows<3>());

  // Compute the Jacobian of the FT-sensor frame expressed in the FT-sensor
  // frame by transporting the target-frame jacobian along the rigid
  // end-effector link.
  eigenmath::Matrix6d ft_M_target =
      PlueckerMotionTransformInverse(target_t_ft_sensor);
  eigenmath::Matrix6Nd ft_jacobian_ft = ft_M_target * target_jacobian_target;

  Twist ft_twist(ft_jacobian_ft * current_joint_velocity);
  // compute approximation of the FT-sensor's acceleration.
  eigenmath::Vector6d ft_acceleration =
      ft_jacobian_ft * current_joint_acceleration;

  ComputePostSensorDynamicsResult result;

  result.dynamic_load_at_ft = ComputePostSensorDynamicLoadAtFTSensor(
      ft_twist, ft_acceleration, total_post_sensor_payload.mass_kg,
      /*robot_T_ft=*/base_t_target * target_t_ft_sensor,
      total_post_sensor_payload.ft_t_cog);

  eigenmath::Matrix6d target_F_ft = PlueckerForceTransform(target_t_ft_sensor);

  result.dynamic_load_at_target = target_F_ft * result.dynamic_load_at_ft;

  return result;
}

}  // namespace intrinsic::icon
