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

#include "intrinsic/kinematics/types/check_cartesian_limits.h"

#include "intrinsic/icon/utils/log.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"

namespace intrinsic {

bool IsWithinLimits(const CartStateP& state, const CartesianLimits& limits) {
  if ((state.pose.translation().array() <
       limits.min_translational_position.array())
          .any() ||
      (state.pose.translation().array() >
       limits.max_translational_position.array())
          .any()) {
    INTRINSIC_RT_LOG(ERROR)
        << "Cartesian translational pose limits exceeded: Minimum: ["
        << limits.min_translational_position[0] << ", "
        << limits.min_translational_position[1] << ", "
        << limits.min_translational_position[2] << "] vs Actual: ["
        << state.pose.translation()[0] << ", " << state.pose.translation()[1]
        << ", " << state.pose.translation()[2] << "] vs Maximum: ["
        << limits.max_translational_position[0] << ", "
        << limits.max_translational_position[1] << ", "
        << limits.max_translational_position[2] << "]";
    return false;
  }
  return true;
}

bool IsWithinLimits(const CartStateV& state, const CartesianLimits& limits) {
  if ((state.velocity.head<3>().array() <
       limits.min_translational_velocity.array())
          .any() ||
      (state.velocity.head<3>().array() >
       limits.max_translational_velocity.array())
          .any()) {
    INTRINSIC_RT_LOG(ERROR)
        << "Cartesian translational velocity limits exceeded: Minimum: ["
        << limits.min_translational_velocity[0] << ", "
        << limits.min_translational_velocity[1] << ", "
        << limits.min_translational_velocity[2] << "] vs Actual: ["
        << state.velocity.head<3>()[0] << ", " << state.velocity.head<3>()[1]
        << ", " << state.velocity.head<3>()[2] << "] vs Maximum: ["
        << limits.max_translational_velocity[0] << ", "
        << limits.max_translational_velocity[1] << ", "
        << limits.max_translational_velocity[2] << "]";
    return false;
  }
  if (state.velocity.tail<3>().norm() > limits.max_rotational_velocity) {
    INTRINSIC_RT_LOG(ERROR)
        << "Cartesian rotational velocity limits exceeded: Norm(Actual): "
        << state.velocity.tail<3>().norm()
        << " vs Maximum: " << limits.max_rotational_velocity;
    return false;
  }
  return true;
}

bool IsWithinAccLimits(const CartStateA& state, const CartesianLimits& limits) {
  if ((state.acceleration.head<3>().array() <
       limits.min_translational_acceleration.array())
          .any() ||
      (state.acceleration.head<3>().array() >
       limits.max_translational_acceleration.array())
          .any()) {
    INTRINSIC_RT_LOG(ERROR)
        << "Cartesian translational acceleration limits exceeded: Minimum: ["
        << limits.min_translational_acceleration[0] << ", "
        << limits.min_translational_acceleration[1] << ", "
        << limits.min_translational_acceleration[2] << "] vs Actual: ["
        << state.acceleration.head<3>()[0] << ", "
        << state.acceleration.head<3>()[1] << ", "
        << state.acceleration.head<3>()[2] << "] vs Maximum: ["
        << limits.max_translational_acceleration[0] << ", "
        << limits.max_translational_acceleration[1] << ", "
        << limits.max_translational_acceleration[2] << "]";
    return false;
  }
  if (state.acceleration.tail<3>().norm() >
      limits.max_rotational_acceleration) {
    INTRINSIC_RT_LOG(ERROR)
        << "Cartesian rotational acceleration limits exceeded: Norm(Actual): "
        << state.acceleration.tail<3>().norm()
        << " vs Maximum: " << limits.max_rotational_acceleration;
    return false;
  }
  return true;
}

bool IsWithinJerkLimits(const CartStateJ& state,
                        const CartesianLimits& limits) {
  if ((state.jerk.head<3>().array() < limits.min_translational_jerk.array())
          .any() ||
      (state.jerk.head<3>().array() > limits.max_translational_jerk.array())
          .any()) {
    INTRINSIC_RT_LOG(ERROR)
        << "Cartesian translational jerk limits exceeded: Minimum: ["
        << limits.min_translational_jerk[0] << ", "
        << limits.min_translational_jerk[1] << ", "
        << limits.min_translational_jerk[2] << "] vs Actual: ["
        << state.jerk.head<3>()[0] << ", " << state.jerk.head<3>()[1] << ", "
        << state.jerk.head<3>()[2] << "] vs Maximum: ["
        << limits.max_translational_jerk[0] << ", "
        << limits.max_translational_jerk[1] << ", "
        << limits.max_translational_jerk[2] << "]";
    return false;
  }
  if (state.jerk.tail<3>().norm() > limits.max_rotational_jerk) {
    INTRINSIC_RT_LOG(ERROR)
        << "Cartesian rotational jerk limits exceeded: Norm(Actual): "
        << state.jerk.tail<3>().norm()
        << " vs Maximum: " << limits.max_rotational_jerk;
    return false;
  }
  return true;
}

bool IsWithinLimits(const CartStatePV& state, const CartesianLimits& limits) {
  if (!IsWithinLimits(static_cast<const CartStateP&>(state), limits)) {
    INTRINSIC_RT_LOG(ERROR) << "Cartesian position limits exceeded.";
    return false;
  }
  if (!IsWithinLimits(static_cast<const CartStateV&>(state), limits)) {
    INTRINSIC_RT_LOG(ERROR) << "Cartesian velocity limits exceeded.";
    return false;
  }
  return true;
}

bool IsWithinLimits(const CartStatePVA& state, const CartesianLimits& limits) {
  bool is_within_pv_limits =
      IsWithinLimits(static_cast<const CartStatePV&>(state), limits);

  bool is_within_a_limits =
      IsWithinAccLimits(static_cast<const CartStateA&>(state), limits);

  return is_within_a_limits && is_within_pv_limits;
}

bool IsWithinLimits(const CartStateVAJ& state, const CartesianLimits& limits) {
  if (!IsWithinLimits(static_cast<const CartStateV&>(state), limits))
    return false;
  if (!IsWithinAccLimits(static_cast<const CartStateA&>(state), limits))
    return false;
  if (!IsWithinJerkLimits(static_cast<const CartStateJ&>(state), limits))
    return false;

  return true;
}

bool IsWithinLimits(const CartesianLimits& limits_to_check,
                    const CartesianLimits& limits) {
  return !((limits_to_check.min_translational_position.array() <
            limits.min_translational_position.array())
               .any() ||
           (limits_to_check.max_translational_position.array() >
            limits.max_translational_position.array())
               .any() ||
           (limits_to_check.min_translational_velocity.array() <
            limits.min_translational_velocity.array())
               .any() ||
           (limits_to_check.max_translational_velocity.array() >
            limits.max_translational_velocity.array())
               .any() ||
           (limits_to_check.min_translational_acceleration.array() <
            limits.min_translational_acceleration.array())
               .any() ||
           (limits_to_check.max_translational_acceleration.array() >
            limits.max_translational_acceleration.array())
               .any() ||
           (limits_to_check.min_translational_jerk.array() <
            limits.min_translational_jerk.array())
               .any() ||
           (limits_to_check.max_translational_jerk.array() >
            limits.max_translational_jerk.array())
               .any() ||
           limits_to_check.max_rotational_velocity >
               limits.max_rotational_velocity ||
           limits_to_check.max_rotational_acceleration >
               limits.max_rotational_acceleration ||
           limits_to_check.max_rotational_jerk > limits.max_rotational_jerk);
}

}  // namespace intrinsic
