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

#ifndef INTRINSIC_KINEMATICS_TYPES_JOINT_LIMITS_MATCHER_H_
#define INTRINSIC_KINEMATICS_TYPES_JOINT_LIMITS_MATCHER_H_

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <string_view>
#include <type_traits>

#include "intrinsic/eigenmath/type_checks.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/is_approx_with_inf.h"
#include "intrinsic/kinematics/types/to_fixed_string.h"

namespace intrinsic {

MATCHER_P(JointLimitIsApprox, expected,
          "is approximately equal to\n" +
              std::string(ToFixedString(expected))) {
  using Scalar = eigenmath::ScalarTypeOf<std::decay_t<decltype(arg)>>;
  double tolerance = Eigen::NumTraits<Scalar>::dummy_precision();

  if (!IsApproxWithInf(expected.min_position, arg.min_position, tolerance,
                       result_listener)) {
    *result_listener << "\n"
                     << std::string_view(ToFixedString(arg))
                     << "\nmin_position differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_position, arg.max_position, tolerance,
                       result_listener)) {
    *result_listener << "\n"
                     << std::string_view(ToFixedString(arg))
                     << "\nmax_position differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_velocity, arg.max_velocity, tolerance,
                       result_listener)) {
    *result_listener << "\n"
                     << std::string_view(ToFixedString(arg))
                     << "\nmax_velocity differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_acceleration, arg.max_acceleration,
                       tolerance, result_listener)) {
    *result_listener << "\n"
                     << std::string_view(ToFixedString(arg))
                     << "\nmax_acceleration differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_jerk, arg.max_jerk, tolerance,
                       result_listener)) {
    *result_listener << "\n"
                     << std::string_view(ToFixedString(arg))
                     << "\nmax_jerk differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_torque, arg.max_torque, tolerance,
                       result_listener)) {
    *result_listener << "\n"
                     << std::string_view(ToFixedString(arg))
                     << "\nmax_torque differ";
    return false;
  }
  return true;
}

MATCHER_P(JointLimitXdIsApprox, expected,
          "is approximately equal to\n" + ToString(expected)) {
  using Scalar = eigenmath::ScalarTypeOf<std::decay_t<decltype(arg)>>;
  double tolerance = Eigen::NumTraits<Scalar>::dummy_precision();

  if (!IsApproxWithInf(expected.min_position, arg.min_position, tolerance,
                       result_listener)) {
    *result_listener << "\n" << ToString(arg) << "\nmin_position differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_position, arg.max_position, tolerance,
                       result_listener)) {
    *result_listener << "\n" << ToString(arg) << "\nmax_position differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_velocity, arg.max_velocity, tolerance,
                       result_listener)) {
    *result_listener << "\n" << ToString(arg) << "\nmax_velocity differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_acceleration, arg.max_acceleration,
                       tolerance, result_listener)) {
    *result_listener << "\n" << ToString(arg) << "\nmax_acceleration differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_jerk, arg.max_jerk, tolerance,
                       result_listener)) {
    *result_listener << "\n" << ToString(arg) << "\nmax_jerk differ";
    return false;
  }
  if (!IsApproxWithInf(expected.max_torque, arg.max_torque, tolerance,
                       result_listener)) {
    *result_listener << "\n" << ToString(arg) << "\nmax_torque differ";
    return false;
  }
  return true;
}

}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_TYPES_JOINT_LIMITS_MATCHER_H_
