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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_SQUARED_PATH_VELOCITY_INTERFACE_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_SQUARED_PATH_VELOCITY_INTERFACE_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_trajectory_result.pb.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::topp {

// Abstract interface representing a continuous parameterization of a squared
// path velocity profile along a path.
class SquaredPathVelocityInterface {
 public:
  virtual ~SquaredPathVelocityInterface() = default;

  // Computes the time corresponding to a given `path_variable`.
  virtual absl::StatusOr<double> GetTimeForPathVariable(
      double path_variable) const = 0;

  // Computes the squared path velocities and its derivatives at collocation
  // points of the discrete trajectory.
  virtual absl::StatusOr<std::vector<std::vector<double>>>
  EvaluateSquaredPathVelocitiesAndDerivatives() const = 0;

  // Computes the squared path velocities and its derivatives at
  // `query_values`.
  virtual absl::StatusOr<std::vector<std::vector<double>>>
  EvaluateSquaredPathVelocitiesAndDerivatives(
      absl::Span<const double> query_values) const = 0;

  // Computes the timings across the domain of the parameterization at
  // collocation points of the discrete trajectory.
  virtual absl::StatusOr<std::vector<double>> EvaluateTimes() const = 0;

  // Returns the domain range `[start_path_variable, end_path_variable]`.
  virtual FixedVector<double, 2> Domain() const = 0;

  // Populates the squared path velocity of a `ToppTrajectoryResult` proto.
  virtual absl::Status PopulateProto(
      intrinsic_proto::topp::ToppTrajectoryResult* proto_msg) const = 0;
};

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_SQUARED_PATH_VELOCITY_INTERFACE_H_
