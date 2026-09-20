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

#include "intrinsic/motion_planning/skills/motion_planning_util.h"

#include <limits>
#include <optional>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/motion_planning/proto/v1/robot_specification.pb.h"
#include "intrinsic/world/objects/kinematic_object.h"

namespace intrinsic::motion_planning {

// TODO(b/309009554): Remove hard-coded limits
CartesianLimits CreateDefaultCartLimits(bool set_infinite_jerk_limits) {
  const double kMaxRotationalAcceleration = 10.0;  // rad/s^2
  const double kMaxRotationalVelocity = 1.0;       // rad/2
  const double kMaxRotationalJerk =
      set_infinite_jerk_limits ? std::numeric_limits<double>::infinity()
                               : 10.0;  // rad/s^3

  const double kMaxTranslationalPosition = 100;      // m
  const double kMaxTranslationalVelocity = 10;       // m/s
  const double kMaxTranslationalAcceleration = 100;  // m/s^2
  const double kMaxTranslationalJerk =
      set_infinite_jerk_limits ? std::numeric_limits<double>::infinity()
                               : 100;  //  m/s^3

  CartesianLimits cart_limits;
  cart_limits.max_rotational_acceleration = kMaxRotationalAcceleration;
  cart_limits.max_rotational_jerk = kMaxRotationalJerk;
  cart_limits.max_rotational_velocity = kMaxRotationalVelocity;
  cart_limits.max_translational_jerk.setConstant(kMaxTranslationalJerk);
  cart_limits.min_translational_jerk.setConstant(-kMaxTranslationalJerk);

  cart_limits.max_translational_acceleration.setConstant(
      kMaxTranslationalAcceleration);
  cart_limits.min_translational_acceleration.setConstant(
      -kMaxTranslationalAcceleration);

  cart_limits.max_translational_velocity.setConstant(kMaxTranslationalVelocity);
  cart_limits.min_translational_velocity.setConstant(
      -kMaxTranslationalVelocity);

  cart_limits.max_translational_position.setConstant(kMaxTranslationalPosition);
  cart_limits.min_translational_position.setConstant(
      -kMaxTranslationalPosition);

  return cart_limits;
}

intrinsic_proto::motion_planning::v1::RobotSpecification
CreateRobotSpecification(
    const world::KinematicObject& robot,
    const std::optional<Eigen::VectorXd>& start_configuration) {
  intrinsic_proto::motion_planning::v1::RobotSpecification robot_specification;
  // Set object reference for robot.
  *robot_specification.mutable_robot_reference()->mutable_object_id() =
      robot.ObjectReference();

  if (start_configuration.has_value()) {
    *robot_specification.mutable_start_configuration() =
        icon::ToJointVecProto(start_configuration.value());
  }

  return robot_specification;
}

}  // namespace intrinsic::motion_planning
