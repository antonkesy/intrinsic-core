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

#include "intrinsic/motion_planning/trajectory_planning/topp/piecewise_linear_squared_path_velocity.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/time_series_utils.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/phase_space_target_generators.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/piecewise_linear_squared_path_velocity.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_trajectory_result.pb.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::topp {

absl::StatusOr<std::unique_ptr<PiecewiseLinearSquaredPathVelocity>>
PiecewiseLinearSquaredPathVelocity::Create(
    absl::Span<const double> path_variables,
    absl::Span<const double> cumulative_times,
    absl::Span<const PhaseSpacePoint> phase_states) {
  if (path_variables.size() < 2 || cumulative_times.size() < 2 ||
      phase_states.size() < 2) {
    return absl::InvalidArgumentError(
        absl::StrCat("Path variables [", path_variables.size(), "], times [",
                     cumulative_times.size(), "], and phase states [",
                     phase_states.size(), "] must have at least size 2."));
  }
  if (path_variables.size() != cumulative_times.size() ||
      path_variables.size() != phase_states.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Path variables [", path_variables.size(), "], times [",
                     cumulative_times.size(), "], and phase states [",
                     phase_states.size(), "] must have the same size."));
  }

  return std::unique_ptr<PiecewiseLinearSquaredPathVelocity>(
      new PiecewiseLinearSquaredPathVelocity(
          std::vector<double>(path_variables.begin(), path_variables.end()),
          std::vector<double>(cumulative_times.begin(), cumulative_times.end()),
          std::vector<PhaseSpacePoint>(phase_states.begin(),
                                       phase_states.end())));
}

PiecewiseLinearSquaredPathVelocity::PiecewiseLinearSquaredPathVelocity(
    std::vector<double> path_variables, std::vector<double> cumulative_times,
    std::vector<PhaseSpacePoint> phase_states)
    : path_variables_(std::move(path_variables)),
      cumulative_times_(std::move(cumulative_times)),
      phase_states_(std::move(phase_states)) {}

absl::StatusOr<double>
PiecewiseLinearSquaredPathVelocity::GetTimeForPathVariable(
    double path_variable) const {
  constexpr double kPathTolerance = 1.0e-6;
  if (path_variable < path_variables_.front() - kPathTolerance) {
    return cumulative_times_.front();
  }
  if (path_variable > path_variables_.back() + kPathTolerance) {
    return cumulative_times_.back();
  }

  const int index = intrinsic::GetLowerIndexForValue<double>(
      absl::MakeConstSpan(path_variables_), path_variable);

  if (index >= static_cast<int>(path_variables_.size()) - 1) {
    return cumulative_times_.back();
  }

  const double s_start = path_variables_[index];
  const double s_end = path_variables_[index + 1];
  const double t_start = cumulative_times_[index];
  const double t_end = cumulative_times_[index + 1];

  const double ds = s_end - s_start;
  if (AlmostEquals(ds, 0.0, kPathTolerance)) {
    return t_start;
  }

  const double fraction = (path_variable - s_start) / ds;
  const double interpolated_time = t_start + fraction * (t_end - t_start);
  return std::clamp(interpolated_time, t_start, t_end);
}

absl::StatusOr<std::vector<std::vector<double>>>
PiecewiseLinearSquaredPathVelocity::
    EvaluateSquaredPathVelocitiesAndDerivatives() const {
  std::vector<std::vector<double>> result;
  result.reserve(phase_states_.size());

  for (size_t i = 0; i < phase_states_.size(); ++i) {
    const double b = phase_states_[i].b;
    const double bp = phase_states_[i].bp;
    result.push_back({b, bp});
  }

  return result;
}

absl::StatusOr<std::vector<std::vector<double>>>
PiecewiseLinearSquaredPathVelocity::EvaluateSquaredPathVelocitiesAndDerivatives(
    absl::Span<const double> query_values) const {
  return absl::UnimplementedError(
      "Querying squared path velocity derivatives at arbitrary path variables "
      "is unimplemented.");
}

absl::StatusOr<std::vector<double>>
PiecewiseLinearSquaredPathVelocity::EvaluateTimes() const {
  return cumulative_times_;
}

FixedVector<double, 2> PiecewiseLinearSquaredPathVelocity::Domain() const {
  return {path_variables_.front(), path_variables_.back()};
}

absl::Status PiecewiseLinearSquaredPathVelocity::PopulateProto(
    intrinsic_proto::topp::ToppTrajectoryResult* proto_msg) const {
  if (proto_msg == nullptr) {
    return absl::InvalidArgumentError("proto_msg cannot be nullptr.");
  }
  *proto_msg->mutable_piecewise_linear_squared_path_velocity() = ToProto(*this);
  return absl::OkStatus();
}

intrinsic_proto::topp::PiecewiseLinearSquaredPathVelocityProto ToProto(
    const PiecewiseLinearSquaredPathVelocity& squared_path_velocity) {
  intrinsic_proto::topp::PiecewiseLinearSquaredPathVelocityProto proto;
  for (double s : squared_path_velocity.PathVariables()) {
    proto.add_path_variables(s);
  }
  for (double t : squared_path_velocity.CumulativeTimes()) {
    proto.add_cumulative_times(t);
  }
  for (const auto& state : squared_path_velocity.PhaseStates()) {
    auto* state_proto = proto.add_phase_states();
    state_proto->set_b(state.b);
    state_proto->set_bp(state.bp);
  }
  return proto;
}

absl::StatusOr<std::unique_ptr<PiecewiseLinearSquaredPathVelocity>> FromProto(
    const intrinsic_proto::topp::PiecewiseLinearSquaredPathVelocityProto&
        proto) {
  std::vector<double> path_variables(proto.path_variables().begin(),
                                     proto.path_variables().end());
  std::vector<double> cumulative_times(proto.cumulative_times().begin(),
                                       proto.cumulative_times().end());
  std::vector<PhaseSpacePoint> phase_states;
  phase_states.reserve(proto.phase_states_size());
  for (const auto& state_proto : proto.phase_states()) {
    phase_states.push_back({state_proto.b(), state_proto.bp()});
  }

  return PiecewiseLinearSquaredPathVelocity::Create(std::move(path_variables),
                                                    std::move(cumulative_times),
                                                    std::move(phase_states));
}

}  // namespace intrinsic::topp
