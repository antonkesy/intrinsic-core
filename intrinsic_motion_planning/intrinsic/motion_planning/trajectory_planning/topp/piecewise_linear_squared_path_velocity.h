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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PIECEWISE_LINEAR_SQUARED_PATH_VELOCITY_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PIECEWISE_LINEAR_SQUARED_PATH_VELOCITY_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/phase_space_target_generators.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/piecewise_linear_squared_path_velocity.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/squared_path_velocity_interface.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_trajectory_result.pb.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::topp {

// A representation of a squared path velocity profile defined across discrete
// collocation points in phase space using piecewise linear interpolation.
class PiecewiseLinearSquaredPathVelocity : public SquaredPathVelocityInterface {
 public:
  static absl::StatusOr<std::unique_ptr<PiecewiseLinearSquaredPathVelocity>>
  Create(absl::Span<const double> path_variables,
         absl::Span<const double> cumulative_times,
         absl::Span<const PhaseSpacePoint> phase_states);

  // Computes the time corresponding to a given `path_variable`.
  // Linearly interpolates between adjacent collocation times.
  absl::StatusOr<double> GetTimeForPathVariable(
      double path_variable) const override;

  absl::StatusOr<std::vector<std::vector<double>>>
  EvaluateSquaredPathVelocitiesAndDerivatives() const override;

  // This method is unimplemented.
  absl::StatusOr<std::vector<std::vector<double>>>
  EvaluateSquaredPathVelocitiesAndDerivatives(
      absl::Span<const double> query_values) const override;

  absl::StatusOr<std::vector<double>> EvaluateTimes() const override;

  FixedVector<double, 2> Domain() const override;

  absl::Status PopulateProto(
      intrinsic_proto::topp::ToppTrajectoryResult* proto_msg) const override;

  const std::vector<double>& PathVariables() const { return path_variables_; }
  const std::vector<double>& CumulativeTimes() const {
    return cumulative_times_;
  }
  const std::vector<PhaseSpacePoint>& PhaseStates() const {
    return phase_states_;
  }

 private:
  PiecewiseLinearSquaredPathVelocity(std::vector<double> path_variables,
                                     std::vector<double> cumulative_times,
                                     std::vector<PhaseSpacePoint> phase_states);

  std::vector<double> path_variables_;
  std::vector<double> cumulative_times_;
  std::vector<PhaseSpacePoint> phase_states_;
};

// Serializes a `PiecewiseLinearSquaredPathVelocity` to proto.
intrinsic_proto::topp::PiecewiseLinearSquaredPathVelocityProto ToProto(
    const PiecewiseLinearSquaredPathVelocity& squared_path_velocity);

// Deserializes a `PiecewiseLinearSquaredPathVelocity` from proto.
absl::StatusOr<std::unique_ptr<PiecewiseLinearSquaredPathVelocity>> FromProto(
    const intrinsic_proto::topp::PiecewiseLinearSquaredPathVelocityProto&
        proto);

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PIECEWISE_LINEAR_SQUARED_PATH_VELOCITY_H_
