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

#include "intrinsic/icon/control/algorithms/settling_time_counter.h"

#include <cmath>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

double SettlingTimeCounter::ComputeSettlingTime(
    double translational_velocity_settling_band,
    double angular_velocity_settling_band, double joint_velocity_settling_band,
    const JointStateV& joint_velocity, const Twist& endeffector_twist) {
  if ((endeffector_twist.head<3>().array().abs().maxCoeff() <=
       std::fabs(translational_velocity_settling_band)) &&
      (endeffector_twist.tail<3>().array().abs().maxCoeff() <=
       std::fabs(angular_velocity_settling_band)) &&
      (joint_velocity.velocity.array().abs().maxCoeff() <=
       std::fabs(joint_velocity_settling_band))) {
    settled_for_seconds_ += cycle_time_;
  } else {
    settled_for_seconds_ = -cycle_time_;
  }

  return settled_for_seconds_;
}

}  // namespace intrinsic::icon
