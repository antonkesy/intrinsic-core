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

#include "intrinsic/motion_planning/path_planning/data_structures/uniform_state_sampler.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "boost/math/special_functions/prime.hpp"
#include "google/protobuf/any.pb.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/halton_sequence.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/util/proto/parse_text_proto.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {
using ::intrinsic::ParseTextProtoOrDie;
proto::UniformStateSamplerConfig SetDefaultConfig(
    const proto::UniformStateSamplerConfig& config) {
  proto::UniformStateSamplerConfig default_config = ParseTextProtoOrDie(R"pb(
    goal_sampling_probability: 0.05
    goal_sampling_distance: 0.1
    ;
      )pb");
  default_config.MergeFrom(config);
  return default_config;
}

double GetQuasiRandomValue(double lower_limit, double upper_limit, int* seed) {
  // It's possible to wrap around while incrementing the seed. In this case the
  // random generator may become degenerate. This check ensures we avoid this
  // case.
  CHECK(seed != nullptr);
  if (*seed < 0) {
    *seed = 0;
  }

  constexpr int kPrimeIndex = 0;
  double random_number =
      eigenmath::HaltonSequence(*seed, boost::math::prime(kPrimeIndex));
  double random_number_scaled =
      random_number * (upper_limit - lower_limit) + lower_limit;
  (*seed)++;
  return random_number_scaled;
}

}  // namespace

UniformStateSampler::UniformStateSampler(
    const proto::UniformStateSamplerConfig& config,
    const KinematicsSystemProxy& proxy)
    : StateSampler(proxy.GetJointLimits()),
      config_(SetDefaultConfig(config)),
      goal_sampling_probability_(config_.goal_sampling_probability()),
      goal_sampling_distance_(config_.goal_sampling_distance()),
      halton_sequence_index_(0) {
  // Goal sampling probability must be a valid probability value, i.e., between
  // 0 and 1.
  CHECK_GE(goal_sampling_probability_, 0)
      << "Wrong config parameters. goal_sampling_probability needs to be "
         "between 0 and 1";
  CHECK_LE(goal_sampling_probability_, 1)
      << "Wrong config parameters. goal_sampling_probability needs to be "
         "between 0 and 1";
  // The goal sampling distance must be a non negative value.
  CHECK_GE(goal_sampling_distance_, 0)
      << "Wrong config parameters. goal_sampling_distance needs to be a non "
         "negative value.";

  // Set the configuration validator using the proxy.
  dof_validator_fn_ = [&proxy](const eigenmath::VectorXd& q_dest) -> bool {
    return proxy.IsWithinLimits(q_dest).value();
  };
}

absl::StatusOr<eigenmath::VectorXd>
UniformStateSampler::GetRandomConfiguration() {
  // It's possible to wrap around while incrementing the seed. In this case the
  // random generator may become degenerate. This check ensures we avoid this
  // case.
  if (halton_sequence_index_ < 0) {
    halton_sequence_index_ = 0;
  }

  // The lower und upper_dof_limits are only outer approximations of the joint
  // limits of the robot. In reality the joint limits are slightly more complex.
  // A final check using the dof_validator_fn_ function pointer therefore is
  // necessary to ensure the generated configuration is truly within the joint
  // limits.
  while (true) {
    INTR_ASSIGN_OR_RETURN(
        const eigenmath::VectorXd config,
        eigenmath::GetQuasiRandomVectorXd(sampling_limits_.min_position,
                                          sampling_limits_.max_position,
                                          &halton_sequence_index_));
    if (dof_validator_fn_(config)) {
      return config;
    }
  }
}

absl::StatusOr<eigenmath::VectorXd> UniformStateSampler::GetRandomConfiguration(
    const eigenmath::VectorXd& start, const eigenmath::VectorXd& goal,
    const PathPlannerGraph* start_graph, const PathPlannerGraph* goal_graph) {
  constexpr double kLowerLimit = 0;
  constexpr double kUpperLimit = 1;
  auto sampling_probability =
      GetQuasiRandomValue(kLowerLimit, kUpperLimit, &halton_sequence_index_);

  if (sampling_probability <= goal_sampling_probability_) {
    return GetGoalSamplingConfiguration(goal);
  }

  return GetRandomConfiguration();
}

absl::StatusOr<eigenmath::VectorXd>
UniformStateSampler::GetRandomConfigurationNearby(
    const eigenmath::VectorXd& seed_configuration, double distance) {
  return GetRandomConfigurationNearby(
      seed_configuration,
      eigenmath::VectorXd::Constant(seed_configuration.size(), distance));
}

absl::StatusOr<eigenmath::VectorXd>
UniformStateSampler::GetRandomConfigurationNearby(
    const eigenmath::VectorXd& seed_configuration,
    const eigenmath::VectorXd& distance) {
  CHECK_EQ(distance.size(), sampling_limits_.size())
      << "Dimensions of provided distance vector does not correspond the "
         "dimensionality of the dof limits for this state sampler.";

  // Set limits according to the degree of freedom limits and distance relative
  // to the seed_configuration.
  eigenmath::VectorXd lower_limits = sampling_limits_.min_position;
  eigenmath::VectorXd upper_limits = sampling_limits_.max_position;
  for (int i = 0; i < lower_limits.size(); ++i) {
    lower_limits[i] =
        std::max(lower_limits[i], seed_configuration[i] - distance[i]);
    upper_limits[i] =
        std::min(upper_limits[i], seed_configuration[i] + distance[i]);
  }

  // The lower und upper_dof_limits are only outer approximations of the joint
  // limits of the robot. In reality the joint limits a slightly more complex.
  // A final check using the dof_validator_fn_ function pointer therefore is
  // necessary to ensure the generated configuration is truly within the joint
  // limits.
  while (true) {
    INTR_ASSIGN_OR_RETURN(
        eigenmath::VectorXd random_config,
        eigenmath::GetQuasiRandomVectorXd(lower_limits, upper_limits,
                                          &halton_sequence_index_));
    if (dof_validator_fn_(random_config)) {
      return random_config;
    }
  }
}

absl::StatusOr<eigenmath::VectorXd>
UniformStateSampler::GetGoalSamplingConfiguration(
    const eigenmath::VectorXd& goal) {
  return GetRandomConfigurationNearby(goal, goal_sampling_distance_);
}

void UniformStateSampler::AddSamplingSeedConfiguration(
    const eigenmath::VectorXd& configuration) {}

void UniformStateSampler::AddAvoidanceConfiguration(
    const eigenmath::VectorXd& key, const eigenmath::VectorXd& configuration) {}

void UniformStateSampler::ResetAvoidanceConfigurations() {}

absl::Status UniformStateSampler::SetAndClampSamplingLimtis(
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
      "Provided limits have a different size than the system limits associated "
      "with the state sampler.");
}

absl::StatusOr<std::unique_ptr<StateSampler>> UniformStateSampler::Create(
    const proto::StateSamplerSpecification& spec,
    const KinematicsSystemProxy& proxy) {
  proto::UniformStateSamplerConfig proto;
  if (!spec.config().UnpackTo(&proto)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to unpack config to specified type. Failure occurred for ",
        spec.name()));
  }
  auto state_sampler = std::make_unique<UniformStateSampler>(proto, proxy);
  return std::unique_ptr<StateSampler>(std::move(state_sampler));
}

REGISTER_STATE_SAMPLER(UniformStateSampler, "UniformStateSampler",
                       UniformStateSampler::Create);

}  // namespace intrinsic
