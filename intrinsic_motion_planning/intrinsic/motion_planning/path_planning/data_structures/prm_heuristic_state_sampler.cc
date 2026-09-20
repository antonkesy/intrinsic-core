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

#include "intrinsic/motion_planning/path_planning/data_structures/prm_heuristic_state_sampler.h"

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/random/distributions.h"
#include "absl/random/random.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/uniform_state_sampler.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/validators_config.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
PrmHeuristicStateSampler::PrmHeuristicStateSampler(
    const proto::PrmHeuristicStateSamplerConfig& config,
    const KinematicsSystemProxy& proxy)
    : StateSampler(proxy.GetJointLimits()),
      goal_sampling_probability_(config.goal_sampling_probability()),
      start_sampling_probability_(config.start_sampling_probability()),
      straight_line_sampling_probability_(
          config.straight_line_sampling_probability()),
      seed_extension_sampling_probability_(
          config.seed_extension_sampling_probability()),
      max_sampling_attempts_(config.max_sampling_attempts()),
      end_point_exploration_distance_(config.end_point_exploration_distance()),
      initial_seed_distance_(config.initial_seed_distance()),
      seed_(0) {
  proto::UniformStateSamplerConfig uniform_sampler_config;
  uniform_sampler_ =
      std::make_unique<UniformStateSampler>(uniform_sampler_config, proxy);

  // Rescale probabilities if necessary
  double overall_probability = goal_sampling_probability_ +
                               start_sampling_probability_ +
                               straight_line_sampling_probability_ +
                               seed_extension_sampling_probability_;
  if (overall_probability > 1.0) {
    goal_sampling_probability_ /= overall_probability;
    start_sampling_probability_ /= overall_probability;
    straight_line_sampling_probability_ /= overall_probability;
    seed_extension_sampling_probability_ /= overall_probability;
  }

  // Set the configuration validator using the proxy.
  dof_validator_fn_ = [&proxy](const eigenmath::VectorXd& q_dest) -> bool {
    return proxy.IsWithinLimits(q_dest).value();
  };
}

absl::StatusOr<eigenmath::VectorXd>
PrmHeuristicStateSampler::QuasiRandomJointConfiguration() {
  return uniform_sampler_->GetRandomConfiguration();
}

absl::StatusOr<eigenmath::VectorXd>
PrmHeuristicStateSampler::QuasiRandomJointConfigurationNearby(
    const eigenmath::VectorXd& config, double distance) {
  return uniform_sampler_->GetRandomConfigurationNearby(config, distance);
}

void PrmHeuristicStateSampler::AddSamplingSeedConfiguration(
    const eigenmath::VectorXd& new_seed_config) {
  PrmHeuristicStateSampler::Seed new_seed(new_seed_config);
  sampling_seeds_.push_back(new_seed);
}

absl::StatusOr<eigenmath::VectorXd> PrmHeuristicStateSampler::NearbySampling(
    const eigenmath::VectorXd& seed_configuration, double distance,
    int iterations) {
  // Perform sampling close to the goal region.
  INTR_ASSIGN_OR_RETURN(
      eigenmath::VectorXd q_new,
      QuasiRandomJointConfigurationNearby(seed_configuration, distance));

  // If no valid configuration was found during the sampling process, return
  // error message.
  return absl::DeadlineExceededError(
      "Not able to find valid sample within given timefram.");
}

absl::StatusOr<eigenmath::VectorXd> PrmHeuristicStateSampler::GoalSampling(
    const eigenmath::VectorXd& goal) {
  // Initialize the connections to the goal configuration.
  if (!goal_connections_.empty() && !goal_connections_[0].isApprox(goal)) {
    goal_connections_.clear();
  }

  if (goal_connections_.empty()) {
    goal_connections_.push_back(goal);
  }

  // Choose a node that is connected to the goal area.
  int seed_index = absl::Uniform<int>(gen_, 0, goal_connections_.size() - 1);
  eigenmath::VectorXd goal_connector = goal_connections_[seed_index];

  // Make adaptive search for the sampling with inverse square of number of
  // sampling attempts.
  constexpr double kMaxDistance = 1.0;
  constexpr double kExplorationFactor = 0.000001;
  const double distance =
      end_point_exploration_distance_ +
      (kMaxDistance - kMaxDistance / (kExplorationFactor * seed_ * seed_ + 1));

  INTR_ASSIGN_OR_RETURN(
      eigenmath::VectorXd q_new,
      NearbySampling(goal_connector, distance, max_sampling_attempts_));

  // Generate configurations close to the goal while measuring how densely
  // populated the region is.
  const double max_density_distance =
      end_point_exploration_distance_ * end_point_exploration_distance_;
  int goals_close_by_seed = 0, goals_close_by_new_config = 0;
  for (int i = 0; i < goal_connections_.size(); ++i) {
    if ((goal_connections_[i] - q_new).squaredNorm() > max_density_distance) {
      ++goals_close_by_new_config;
    }
    if ((goal_connections_[i] - goal_connector).squaredNorm() >
        max_density_distance) {
      ++goals_close_by_seed;
    }
  }
  // Remove seed if it is too densely populated
  if (goals_close_by_seed > max_sampling_attempts_) {
    goal_connections_.erase(goal_connections_.begin() + seed_index);
  }

  // If the region around the new point is too densely populated, do not
  // insert
  if (goals_close_by_new_config < max_sampling_attempts_) {
    goal_connections_.push_back(q_new);
  }

  return q_new;
}

absl::StatusOr<eigenmath::VectorXd>
PrmHeuristicStateSampler::SeedExtensionSampling() {
  constexpr double kMaxDistance = 1.0;
  double distance = initial_seed_distance_ + kMaxDistance;
  int seed_index = -1;
  while (!sampling_seeds_.empty()) {
    seed_index = absl::Uniform<int>(gen_, 0, sampling_seeds_.size());
    CHECK_GE(seed_index, 0);
    CHECK_LE(seed_index, sampling_seeds_.size() - 1);
    if (sampling_seeds_[seed_index].number_extension_attempts_ <=
        max_sampling_attempts_) {
      distance -= ((double)max_sampling_attempts_ -
                   sampling_seeds_[seed_index].number_extension_attempts_) /
                  max_sampling_attempts_;
      break;
    } else {
      sampling_seeds_.erase(sampling_seeds_.begin() + seed_index);
    }
  }

  if (seed_index >= 0 && seed_index < sampling_seeds_.size()) {
    sampling_seeds_[seed_index].number_extension_attempts_++;
    INTR_ASSIGN_OR_RETURN(eigenmath::VectorXd q_new,
                          NearbySampling(sampling_seeds_[seed_index].seed_,
                                         distance, max_sampling_attempts_));
    sampling_seeds_[seed_index].number_successful_extensions_++;
    return q_new;
  }

  return absl::InternalError(
      "Seed index for SeedExtensionSampling exceeds bounds.");
}

absl::StatusOr<eigenmath::VectorXd>
PrmHeuristicStateSampler::GetRandomConfiguration(
    const eigenmath::VectorXd& start, const eigenmath::VectorXd& goal,
    const PathPlannerGraph* start_graph, const PathPlannerGraph* goal_graph) {
  // Determine how we sample randomly.
  double p = absl::Uniform<float>(absl::IntervalClosed, gen_, 0, 1.0);

  // Goal sampling
  if (p <= goal_sampling_probability_) {
    auto result = GoalSampling(goal);
    if (result.ok()) {
      return result.value();
    }
    return QuasiRandomJointConfiguration();
  }

  p -= goal_sampling_probability_;
  // Start sampling
  if (p < start_sampling_probability_) {
    auto result =
        NearbySampling(start, initial_seed_distance_, max_sampling_attempts_);
    if (result.ok()) {
      return result.value();
    }
    return QuasiRandomJointConfiguration();
  }

  p -= start_sampling_probability_;
  // Straight line in config space sampling.
  if (p < straight_line_sampling_probability_) {
    double interpolation_point =
        absl::Uniform<float>(absl::IntervalClosed, gen_, 0, 1.0);
    eigenmath::VectorXd q_seed = start + interpolation_point * (goal - start);
    auto result = NearbySampling(q_seed, initial_seed_distance_, 1);
    if (result.ok()) {
      return result.value();
    }
    return QuasiRandomJointConfiguration();
  }

  p -= straight_line_sampling_probability_;
  // Seed extensions.
  if (p < seed_extension_sampling_probability_) {
    auto result = SeedExtensionSampling();
    if (result.ok()) {
      return result.value();
    }
  }

  // Random sampling.
  return QuasiRandomJointConfiguration();
}

absl::StatusOr<eigenmath::VectorXd>
PrmHeuristicStateSampler::GetRandomConfiguration() {
  return uniform_sampler_->GetRandomConfiguration();
}

absl::StatusOr<eigenmath::VectorXd>
PrmHeuristicStateSampler::GetRandomConfigurationNearby(
    const eigenmath::VectorXd& seed_configuration, double distance) {
  return uniform_sampler_->GetRandomConfigurationNearby(seed_configuration,
                                                        distance);
}

absl::StatusOr<eigenmath::VectorXd>
PrmHeuristicStateSampler::GetRandomConfigurationNearby(
    const eigenmath::VectorXd& seed_configuration,
    const eigenmath::VectorXd& distance) {
  return uniform_sampler_->GetRandomConfigurationNearby(seed_configuration,
                                                        distance);
}

void PrmHeuristicStateSampler::AddAvoidanceConfiguration(
    const eigenmath::VectorXd& key, const eigenmath::VectorXd& configuration) {
  uniform_sampler_->AddAvoidanceConfiguration(key, configuration);
}

void PrmHeuristicStateSampler::ResetAvoidanceConfigurations() {
  uniform_sampler_->ResetAvoidanceConfigurations();
}

absl::Status PrmHeuristicStateSampler::SetSamplingLimits(
    const JointLimitsXd& limits) {
  INTR_RETURN_IF_ERROR(SetSamplingLimitsCommon(limits));
  return absl::OkStatus();
}

absl::Status PrmHeuristicStateSampler::SetAndClampSamplingLimtis(
    const JointLimitsXd& limits) {
  JointLimitsXd clamped_limits = limits;
  if (clamped_limits.IsSizeConsistent() &&
      limits.size() == system_limits_.size()) {
    if (!eigenmath::ClampVector(system_limits_.min_position,
                                system_limits_.max_position,
                                clamped_limits.min_position) ||
        !eigenmath::ClampVector(system_limits_.min_position,
                                system_limits_.max_position,
                                clamped_limits.max_position)) {
      return icon::InternalError("Clamping system position limits failed.");
    } else {
      return SetSamplingLimits(clamped_limits);
    }
  }
  return icon::InternalError(
      "Provided limits are of different size than the system limits associated "
      "with the state sampler.");
}

absl::StatusOr<std::unique_ptr<StateSampler>> PrmHeuristicStateSampler::Create(
    const proto::StateSamplerSpecification& spec,
    const KinematicsSystemProxy& proxy) {
  proto::PrmHeuristicStateSamplerConfig proto;
  if (!spec.config().UnpackTo(&proto)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to unpack config to specified type. Failure occurred for ",
        spec.name()));
  }
  auto state_sampler = std::make_unique<PrmHeuristicStateSampler>(proto, proxy);
  return std::unique_ptr<StateSampler>(std::move(state_sampler));
}

REGISTER_STATE_SAMPLER(PrmHeuristicStateSampler, "PrmHeuristicStateSampler",
                       PrmHeuristicStateSampler::Create);

}  // namespace intrinsic
