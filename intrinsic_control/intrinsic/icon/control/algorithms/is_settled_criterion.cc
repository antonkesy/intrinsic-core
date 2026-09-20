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

#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

// Values within which the model is valid and are used to bound it.
constexpr double kMinNoiseLevel = 0.020;
constexpr double kMaxNoiseLevel = 0.070;

double EvaluateSigmoid(double value) { return 1.0 / (1.0 + std::exp(-value)); }

// Returns a bounded (by the model) estimate of the uncertainty increase rate.
double ComputeUncertaintyRateIncrease(double variance_estimate) {
  return std::min(variance_estimate, ::intrinsic::IPow(kMaxNoiseLevel, 2));
}

// Returns a non-negative value that defines how close the robot is to a
// settled state. The cost is constructed as the sum of:
// - a cost term indicating if the reference motion generation has terminated or
// not, which is a direction function of `has_trajectory_ended`,
// - and a velocity cost that measures the closeness of the measured- to the
// settled joint velocities (considered to be zero).
double IsStateSettledCost(const eigenmath::VectorNd& measured_joint_velocities,
                          bool has_trajectory_ended) {
  // Indicator function cost due to finishing the trajectory replay.
  const double kGoalReachedIndicatorCostWeight = 100.0;
  double indicator_cost = kGoalReachedIndicatorCostWeight *
                          static_cast<double>(!has_trajectory_ended);

  // Cost due to settling at zero joint velocities.
  const double kSettledVelocityCostWeight = 1.0e4;
  double velocity_cost =
      kSettledVelocityCostWeight * measured_joint_velocities.squaredNorm();

  return indicator_cost + velocity_cost;
}

// Returns the rate at which uncertainty decreases according to the estimated
// data variance. It does so by modeling (in an average sense) how new
// measurements at given noise levels decrease the uncertainty about the settled
// state. To be more precise, the model uses data to 1) estimate its variance
// and 2) visually determine the average settling time. This is performed for a
// pair of data points and we interpolate in between.
// The ordinary differential equation (ODE) used to update uncertainty is:
//    x_dot = - a * x + b
// `b` is a small constant term proportional to the data variance.
// `a` is determined by this function. The ODE's solution is:
//    x(t) = (1-b/a) exp(-a*t) + b/a
// Given that b/a is very small, the solution is approximated by
//    x(t) = exp(-a*t)
// Thus, at a given noise_level (as measured by the data variance), settling
// time t_avg_settling_time (visually determined from data points), and a given
// uncertainty_level for reaching a settled state, `a` can be determined as:
//    a = -ln(x(t)=uncertainty_level) / t_avg_settling_time  : for noise_level.
// This model then linearly interpolates the value of `a`:
//    a_lower_limit = -ln(0.2) / (250ms * Sigmoid(1)) : for noise_level = 0.020.
//    a_upper_limit = -ln(0.2) / (400ms * Sigmoid(1)) : for noise_level = 0.070.
// These parameters have been manually tuned. Do not change them without
// consulting with @bponton.
double ComputeUncertaintyRateDecrease(double variance_estimate) {
  // Model constants from data.
  const double kMinAvgSettlingTime = 0.250;
  const double kMaxAvgSettlingTime = 0.400;
  const double kUncertaintyLevel = 0.2;

  // Computed linear model parameters
  const double kLowerLimit = -std::log(kUncertaintyLevel) /
                             (EvaluateSigmoid(1.0) * kMinAvgSettlingTime);
  const double kUpperLimit = -std::log(kUncertaintyLevel) /
                             (EvaluateSigmoid(1.0) * kMaxAvgSettlingTime);
  const double kModelSlope =
      (kUpperLimit - kLowerLimit) / (kMaxNoiseLevel - kMinNoiseLevel);
  const double kModelIntercept = kUpperLimit - kMaxNoiseLevel * kModelSlope;

  // Apply linear model to current variance estimate
  double bounded_std_dev = std::min(
      std::max(std::sqrt(variance_estimate), kMinNoiseLevel), kMaxNoiseLevel);
  double uncertainty_rate_decrease =
      kModelSlope * bounded_std_dev + kModelIntercept;

  return uncertainty_rate_decrease;
}

// Returns the rate that the belief model uses to update the uncertainty.
// This uncertainty rate is computed as the sum of two components:
// - an increasing component that grows linearly,
// - and a decreasing component that decreases exponentially as a function of
// the how close the state is to the settled state (as measured by the
// `IsStateSettledCost` function).
double ComputeUncertaintyUpdateRate(double is_settled_cost,
                                    double variance_estimate,
                                    double current_uncertainty) {
  // Components that affect the total rate.
  double uncertainty_rate_increase =
      ComputeUncertaintyRateIncrease(variance_estimate);
  double uncertainty_rate_decrease =
      ComputeUncertaintyRateDecrease(variance_estimate) *
      EvaluateSigmoid(1.0 - is_settled_cost) * current_uncertainty;

  return uncertainty_rate_increase - uncertainty_rate_decrease;
}

}  // namespace

// static
absl::StatusOr<std::unique_ptr<IsSettledCriterion>> IsSettledCriterion::Create(
    double frequency_hz, double max_uncertainty_threshold) {
  if (frequency_hz <= 0.0) {
    return InvalidArgumentError(absl::StrCat(
        "The frequency should be larger than zero. Got ", frequency_hz, "."));
  }
  if (max_uncertainty_threshold <= kMinUncertainty ||
      max_uncertainty_threshold > kMaxUncertainty) {
    return OutOfRangeError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
        "The threshold should be within (", kMinUncertainty, ", ",
        kMaxUncertainty, "]. Got ", max_uncertainty_threshold, "."));
  }

  auto is_settled_criterion =
      absl::WrapUnique(new IsSettledCriterion(frequency_hz));
  INTR_RETURN_IF_ERROR(is_settled_criterion->SetMaxUncertaintyThreshold(
      max_uncertainty_threshold));
  return is_settled_criterion;
}

IsSettledCriterion::IsSettledCriterion(double frequency_hz)
    : frequency_hz_(frequency_hz) {}

RealtimeStatus IsSettledCriterion::Initialize() {
  num_samples_ = 0;
  variance_estimate_ = 0.0;
  uncertainty_ = kInitialUncertainty;
  return OkStatus();
}

RealtimeStatusOr<bool> IsSettledCriterion::Update(
    const eigenmath::VectorNd& measured_joint_velocities,
    const eigenmath::VectorNd& commanded_joint_velocities,
    bool has_trajectory_ended) {
  if (measured_joint_velocities.size() != commanded_joint_velocities.size()) {
    return InvalidArgumentError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
        "The size of measured_joint_velocities ",
        measured_joint_velocities.size(), " and commanded_joint_velocities ",
        commanded_joint_velocities.size(), " should be equal."));
  }

  // Recursively update the variance estimate with incoming data.
  const double max_joint_squared_deviation =
      (measured_joint_velocities - commanded_joint_velocities).squaredNorm() /
      measured_joint_velocities.size();
  const double variance_update_due_to_single_sample =
      (max_joint_squared_deviation - variance_estimate_) / (num_samples_ + 1.0);
  ++num_samples_;
  variance_estimate_ += variance_update_due_to_single_sample;

  // Update the current uncertainty value.
  const double is_settled_cost =
      IsStateSettledCost(measured_joint_velocities, has_trajectory_ended);
  if (is_settled_cost < -std::numeric_limits<double>::epsilon()) {
    return InternalError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
        "The settled cost has become negative. Got ", is_settled_cost, "."));
  }
  const double uncertainty_rate = ComputeUncertaintyUpdateRate(
      is_settled_cost, variance_estimate_, uncertainty_);
  uncertainty_ += uncertainty_rate / frequency_hz_;

  // Sanity check for the uncertainty value within the expected bounds, namely
  // larger than `kMinUncertainty` and lower than `kMaxUncertainty`.
  uncertainty_ = std::max(uncertainty_, kMinUncertainty);
  uncertainty_ = std::min(uncertainty_, kMaxUncertainty);

  return uncertainty_ <= max_uncertainty_threshold_;
}

double IsSettledCriterion::GetUncertainty() const { return uncertainty_; }

RealtimeStatus IsSettledCriterion::SetMaxUncertaintyThreshold(
    double max_uncertainty_threshold) {
  if (max_uncertainty_threshold <= kMinUncertainty ||
      max_uncertainty_threshold >= kMaxUncertainty) {
    return OutOfRangeError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
        "The max uncertainty threshold should be within (", kMinUncertainty,
        ", ", kMaxUncertainty, "). Got ", max_uncertainty_threshold, "."));
  }
  max_uncertainty_threshold_ = max_uncertainty_threshold;
  return OkStatus();
}

double IsSettledCriterion::GetMaxUncertaintyThreshold() const {
  return max_uncertainty_threshold_;
}

}  // namespace intrinsic::icon
