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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_OPTIONS_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_OPTIONS_H_

#include "intrinsic/motion_planning/trajectory_planning/topp/joint_optimization_options.pb.h"

namespace intrinsic {
namespace topp {

// Constructs the default optimization options for time parameterization.
//
// By default, this configuration strictly enforces kinematic joint acceleration
// constraints. Joint jerk, Cartesian velocity, and Cartesian acceleration
// constraints are explicitly disabled.
//
// For more details on the parameterization types, refer to:
// go/intrinsic-motion-planning-type-parameterization
OptimizationOptions DefaultToppOptions();

// Constructs optimization options by explicitly setting the active kinematic
// (joint-based) and Cartesian constraints.
//
// Parameters:
//  * `joint_acceleration`: If true, limits the maximum joint acceleration.
//  * `joint_jerk`: If true, limits the maximum joint jerk.
//  * `cart_velocity`: If true, limits the maximum Cartesian velocity.
//  * `cart_acceleration`: If true, limits the maximum Cartesian acceleration.
//
// Returns:
//  An `OptimizationOptions` object populated with the specified kinematic and
//  Cartesian constraint flags.
OptimizationOptions ToppOptionsWithJointAndCartesianConstraints(
    bool joint_acceleration, bool joint_jerk, bool cart_velocity,
    bool cart_acceleration);

// Constructs optimization options by explicitly setting the active dynamic
// (torque-based) and Cartesian constraints.
//
// Parameters:
//  * `joint_torque`: If true, limits the maximum joint torque (requires a valid
//    rigid body dynamics model when solving).
//  * `joint_torque_rate`: If true, limits the rate of change of joint torques.
//  * `cart_velocity`: If true, limits the maximum Cartesian velocity.
//  * `cart_acceleration`: If true, limits the maximum Cartesian acceleration.
//
// Returns:
//  An `OptimizationOptions` object populated with the specified dynamic and
//  Cartesian constraint flags.
OptimizationOptions ToppOptionsWithTorqueAndCartesianConstraints(
    bool joint_torque, bool joint_torque_rate, bool cart_velocity,
    bool cart_acceleration);

}  // namespace topp
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_TOPP_OPTIONS_H_
