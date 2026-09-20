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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_UNIFORM_STATE_SAMPLER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_UNIFORM_STATE_SAMPLER_H_

#include <functional>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"

namespace intrinsic {

// A state sampler that samples uniformly within the configuration space of a
// robot for sampling based path planning. The UniformStateSampler uses the
// halton sequence to generate a random configuration within the configuration
// space defined by it's lower_dof_limits and upper_dof_limits. The final
// check if the generated configuration is within the joint limits of the
// robot happens through the dof_validator_fn that is passed along in the
// constructor.
//
// The UniformStateSampler allows biasing the sampling towards the goal region
// using the function GetRandomConfiguration(start, goal, start_graph,
// goal_graph). It will sample nearby the defined goal configuration according
// to the goal_sampling_probability defined in the proto config provided in the
// constructor. The distance for within one degree of freedom that defines
// 'near by' is defined by goal_sampling_distance that is defined as well in the
// proto config file.
class UniformStateSampler : public StateSampler {
 public:
  static absl::StatusOr<std::unique_ptr<StateSampler>> Create(
      const proto::StateSamplerSpecification& spec,
      const KinematicsSystemProxy& proxy);

  // Creates a uniform state sampler from the config and the kinematics system
  // proxy. It is expected that the proxy has to outlive the
  // UniformStateSampler.
  UniformStateSampler(const proto::UniformStateSamplerConfig& config,
                      const KinematicsSystemProxy& proxy);

  // Uniform state sampling within the degree of freedom limits using the
  // halton sequence (see https://en.wikipedia.org/wiki/Halton_sequence).
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfiguration() override;

  // Uniform state sampling that samples with some probability 'near by' the
  // goal configuration. The probability of sampling within the goal region and
  // the size of the goal region are defined within the configurtion proto of
  // the uniform state sampler.
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfiguration(
      const eigenmath::VectorXd& start, const eigenmath::VectorXd& goal,
      const PathPlannerGraph* start_graph,
      const PathPlannerGraph* goal_graph) override;

  // Generates a random configuration close by a defined seed_configuration.
  // The method generates a uniformly distributed value for each degree of
  // freedom within around the value of the seed_configuration +/- distance.
  // The configuration is guaranteed to be within the joint limits defined
  // by the lower_dof_limits, upper_dof_limits, and dof_validator_fn.
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfigurationNearby(
      const eigenmath::VectorXd& seed_configuration, double distance) override;

  // Generates a random configuration close by a defined seed_configuration.
  // The method generates a uniformly distributed value for each degree of
  // freedom within around the value of the seed_configuration +/- distance.
  // The configuration is guaranteed to be within the joint limits defined
  // by the lower_dof_limits, upper_dof_limits, and dof_validator_fn.
  // The distance from the seed_configuration can be defined for each degree
  // of freedom individually. The dimensionality of distance therefore has to
  // correspond with the one of the *dof_limits.
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfigurationNearby(
      const eigenmath::VectorXd& seed_configuration,
      const eigenmath::VectorXd& distance) override;

  // UniformStateSampler does not use sampling seed configurations to bias the
  // sampling. Since it is a uniform state sampler, we don't want to bias the
  // sampling towards arbitrary seeds. It is therefore not implemented. Adding
  // sampling seeds will have no effect on the sampling.
  //
  void AddSamplingSeedConfiguration(
      const eigenmath::VectorXd& configuration) override;

  // UniformStateSampler does not bias the sampling such that it avoids
  // configurations. Since it is a uniform state sampler, we don't want to bias
  // the sampling to avoid certain configurations. It is therefore not
  // implemented. Adding configurations will have no effect on the sampling.
  void AddAvoidanceConfiguration(
      const eigenmath::VectorXd& key,
      const eigenmath::VectorXd& configuration) override;

  // UniformStateSampler does not bias the sampling such that it avoids
  // configurations. Resetting the avoidance configuration set has no effect on
  // the sampling.
  void ResetAvoidanceConfigurations() override;

  absl::Status SetSamplingLimits(const JointLimitsXd& limits) override {
    return SetSamplingLimitsCommon(limits);
  }

  absl::Status SetAndClampSamplingLimtis(const JointLimitsXd& limits) override;

 protected:
  // Performs a sampling near the specified goal configuration.
  absl::StatusOr<eigenmath::VectorXd> GetGoalSamplingConfiguration(
      const eigenmath::VectorXd& goal);

  // State Sampler configuration
  proto::UniformStateSamplerConfig config_;

  // Parameters that define how often GetRandomConfiguration(start,goal,graph)
  // samples within the goal area and how far from the goal goal configuration
  // the algorithm samples.
  const double goal_sampling_probability_;
  const double goal_sampling_distance_;

  // Seed for quasi random sampling.
  int halton_sequence_index_;

  // This function returns true when the configuration is within the limits of
  // the robot. Please note that the lower and upper_dof_limits are only an
  // outer approximation and that there are small pockets of invalid states
  // that are captured with the dof_validator_fn_.
  std::function<bool(const eigenmath::VectorXd& q_dest)> dof_validator_fn_;
};
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_UNIFORM_STATE_SAMPLER_H_
