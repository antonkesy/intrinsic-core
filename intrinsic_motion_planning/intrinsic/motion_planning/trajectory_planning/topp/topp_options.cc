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

#include "intrinsic/motion_planning/trajectory_planning/topp/topp_options.h"

#include "intrinsic/motion_planning/trajectory_planning/topp/joint_optimization_options.pb.h"

namespace intrinsic {
namespace topp {

OptimizationOptions DefaultToppOptions() {
  OptimizationOptions options;
  // Default kinematic constraints.
  options.mutable_kinematic_constraints()
      ->set_activate_joint_acceleration_constraints(true);
  options.mutable_kinematic_constraints()->set_activate_joint_jerk_constraints(
      false);
  // Default Cartesian constraints.
  options.mutable_cartesian_constraints()
      ->set_activate_cartesian_velocity_constraints(false);
  options.mutable_cartesian_constraints()
      ->set_activate_cartesian_acceleration_constraints(false);
  return options;
}

OptimizationOptions ToppOptionsWithJointAndCartesianConstraints(
    bool joint_acceleration, bool joint_jerk, bool cart_velocity,
    bool cart_acceleration) {
  OptimizationOptions options;
  // Default kinematic constraints.
  options.mutable_kinematic_constraints()
      ->set_activate_joint_acceleration_constraints(joint_acceleration);
  options.mutable_kinematic_constraints()->set_activate_joint_jerk_constraints(
      joint_jerk);
  // Default Cartesian constraints.
  options.mutable_cartesian_constraints()
      ->set_activate_cartesian_velocity_constraints(cart_velocity);
  options.mutable_cartesian_constraints()
      ->set_activate_cartesian_acceleration_constraints(cart_acceleration);
  return options;
}

OptimizationOptions ToppOptionsWithTorqueAndCartesianConstraints(
    bool joint_torque, bool joint_torque_rate, bool cart_velocity,
    bool cart_acceleration) {
  OptimizationOptions options;
  // Default dynamic constraints.
  options.mutable_dynamic_constraints()->set_activate_joint_torque_constraints(
      joint_torque);
  options.mutable_dynamic_constraints()
      ->set_activate_joint_torque_rate_constraints(joint_torque_rate);
  // Default Cartesian constraints.
  options.mutable_cartesian_constraints()
      ->set_activate_cartesian_velocity_constraints(cart_velocity);
  options.mutable_cartesian_constraints()
      ->set_activate_cartesian_acceleration_constraints(cart_acceleration);
  return options;
}

}  // namespace topp
}  // namespace intrinsic
