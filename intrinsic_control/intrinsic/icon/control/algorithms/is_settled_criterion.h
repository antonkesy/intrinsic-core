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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_IS_SETTLED_CRITERION_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_IS_SETTLED_CRITERION_H_

#include <cstddef>
#include <memory>

#include "absl/log/log.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Implements basic functionality to determine if a robot has reached a settled
// state. The criterion generalizes to different robots that have different
// noise properties, and at the same time can be operated intuitively. The
// intended use of this criterion is for controllers that aim at reaching a
// position target (settled state) such as trajectory-tracking, reflexxes-based
// position controllers and so on. It does not work for systems with non-zero
// target velocity. If such a criterion is needed, a possible extension could be
// to provide the non-zero target velocity at initialization and include this
// non-zero target velocity in the cost instead of the current values of zero.
// The criterion is defined as a dynamical system that heuristically provides a
// measure of the uncertainty of the settled state. It is similar to a filter
// in the sense that it has a constant component that increases the uncertainty
// and one that decreases it modulated by a cost that measures the closeness of
// the current state to the settled state.
// Thus, a user would only need to select a threshold that defines the level of
// uncertainty of the settled state below which the robot is considered as
// settled. A smaller threshold would lead to a larger settling time.
class IsSettledCriterion {
 public:
  // Creates an `IsSettledCriterion` where `frequency_hz` is the
  // frequency at which the updates of the belief about the settled state will
  // be performed, which matches the frequency of the control loop. A value for
  // the `max_uncertainty_threshold` can also be provided (defaults to 0.2).
  static absl::StatusOr<std::unique_ptr<IsSettledCriterion>> Create(
      double frequency_hz,
      double max_uncertainty_threshold = kDefaultMaxUncertaintyThreshold);

  // Initializes member variables to prepare for a new motion
  RealtimeStatus Initialize();

  // Updates belief over the settled state using new measurements. The
  // `measured_joint_velocities` and `commanded_joint_velocities` inputs denote
  // exactly what its name describes. The binary input `has_trajectory_ended`
  // signals to the criterion that the planned trajectory being replayed has
  // been fully replayed. Returns:
  // - InvalidArgumentError: if the inputs have invalid size.
  // - The current binary belief over the settled state otherwise.
  RealtimeStatusOr<bool> Update(
      const eigenmath::VectorNd& measured_joint_velocities,
      const eigenmath::VectorNd& commanded_joint_velocities,
      bool has_trajectory_ended);

  // Returns the current double precision uncertainty over the settled state
  // ranging from 0 to 1, where 1 means maximum uncertainty, and 0 minimum
  // uncertainty.
  double GetUncertainty() const;

  // Updates the value of the uncertainty threshold used for evaluating
  // convergence. Returns:
  // - OutOfRangeError: if the input is outside the expected bounds.
  RealtimeStatus SetMaxUncertaintyThreshold(double max_uncertainty_threshold);

  // Returns the current value of the uncertainty threshold
  double GetMaxUncertaintyThreshold() const;

 private:
  explicit IsSettledCriterion(double frequency_hz);

  // Internal variables to cache values and keep track of the uncertainty.
  double frequency_hz_;
  double uncertainty_ = kInitialUncertainty;
  double max_uncertainty_threshold_ = kDefaultMaxUncertaintyThreshold;

  // Internal variables to hold the estimates of the noise properties.
  size_t num_samples_ = 0;
  double variance_estimate_ = 0.0;

  // Parameters of the is settled state belief model.
  static constexpr double kMinUncertainty = 0.0;
  static constexpr double kMaxUncertainty = 1.0;
  static constexpr double kInitialUncertainty = 1.0;
  static constexpr double kDefaultMaxUncertaintyThreshold = 0.2;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_IS_SETTLED_CRITERION_H_
