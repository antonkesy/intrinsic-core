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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SETTLING_TIME_COUNTER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SETTLING_TIME_COUNTER_H_

#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

class SettlingTimeCounter {
 public:
  SettlingTimeCounter() = delete;

  explicit SettlingTimeCounter(double frequency_hz)
      : cycle_time_(1.0 / frequency_hz), settled_for_seconds_(-cycle_time_) {}

  // Resets the counted settling time to negative cycle time.
  void Reset() { settled_for_seconds_ = -cycle_time_; }

  // Returns the time in [sec] for which the 'joint_velocity' and (optional)
  // 'endeffector_twist' have remained within in a settling band defined by
  // 'translational_velocity_settling_band', 'angular_velocity_settling_band'
  // and 'joint_velocity_settling_band'. For the returned value to be correct,
  // this method needs to be called every cycle. Returns negative cycle time if
  // the system has not settled, or if the system becomes 'unsettled' again.
  double ComputeSettlingTime(double translational_velocity_settling_band,
                             double angular_velocity_settling_band,
                             double joint_velocity_settling_band,
                             const JointStateV& joint_velocity,
                             const Twist& endeffector_twist = Twist::ZERO);

 private:
  double cycle_time_;
  double settled_for_seconds_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SETTLING_TIME_COUNTER_H_
