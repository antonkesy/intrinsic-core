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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_PRM_HEURISTIC_STATE_SAMPLER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_PRM_HEURISTIC_STATE_SAMPLER_H_

#include <functional>
#include <memory>
#include <vector>

#include "absl/random/random.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/data_structures/planner_graph.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/uniform_state_sampler.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"

namespace intrinsic {

// Encapsulation of the state sampling procedure for path planning. It allows
// to do a certain percentage of sampling around the goal area, start area,
// along the straight line between start and end, and sampling in some
// difficult areas for which 'seeds' are defined that were in collision.
class PrmHeuristicStateSampler : public StateSampler {
 public:
  static absl::StatusOr<std::unique_ptr<StateSampler>> Create(
      const proto::StateSamplerSpecification& spec,
      const KinematicsSystemProxy& proxy);

  // A seed encapsulates a configuration that is used to focus the sampling,
  // the number of extension attempts, i.e., how often we tried to sample that
  // configuration, and how often we succeeded.
  struct Seed {
    eigenmath::VectorXd seed_;
    int number_extension_attempts_;
    int number_successful_extensions_;

    Seed() : number_extension_attempts_(0), number_successful_extensions_(0) {}

    explicit Seed(const eigenmath::VectorXd& seed)
        : seed_(seed),
          number_extension_attempts_(0),
          number_successful_extensions_(0) {}
  };

  // Generates a state sampler from the config and the proxy. The state sampler
  // will generate a configuration validation function from the proxy. It must
  // be ensured that the proxy outlives the State Sampler.
  PrmHeuristicStateSampler(const proto::PrmHeuristicStateSamplerConfig& config,
                           const KinematicsSystemProxy& proxy);

  // Returns a random configuration in the configuration space of the robot.
  // The returned configuration is within the joint limits of the robot, but
  // not necessarily valid. I.e., the configuration needs to be checked with
  // isValid(config).
  // The sampling will sample uniformly in the whole configuration space, within
  // the goal/start area, along the straight line, and around some conflict
  // points (i.e., seeds) according to the parameters specified during
  // construction of the StateSampler.
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfiguration(
      const eigenmath::VectorXd& start, const eigenmath::VectorXd& goal,
      const PathPlannerGraph* start_graph,
      const PathPlannerGraph* goal_graph) override;

  // Adds a new seeding point to the list of seeding points used for seed
  // extension sampling.
  void AddSamplingSeedConfiguration(
      const eigenmath::VectorXd& new_seed) override;

  // State sampler interface functions below that are delegated to the uniform
  // sampler.

  absl::StatusOr<eigenmath::VectorXd> GetRandomConfiguration() override;

  absl::StatusOr<eigenmath::VectorXd> GetRandomConfigurationNearby(
      const eigenmath::VectorXd& seed_configuration, double distance) override;

  absl::StatusOr<eigenmath::VectorXd> GetRandomConfigurationNearby(
      const eigenmath::VectorXd& seed_configuration,
      const eigenmath::VectorXd& distance) override;

  void AddAvoidanceConfiguration(
      const eigenmath::VectorXd& key,
      const eigenmath::VectorXd& configuration) override;

  void ResetAvoidanceConfigurations() override;

  absl::Status SetSamplingLimits(const JointLimitsXd& limits) override;

  absl::Status SetAndClampSamplingLimtis(const JointLimitsXd& limits) override;

 private:
  // Samples a configuration near the goal configuration or a configuration that
  // is connected with the goal. The configuration is guaranteed to be within
  // limits but not necessarily a collision free/valid configuration.
  absl::StatusOr<eigenmath::VectorXd> GoalSampling(
      const eigenmath::VectorXd& goal);

  // Samples a configuration near by a seed_configuration and test if it is
  // valid. If a valid configuration  cannot be found, it returns a deadline
  // exceeded error;
  absl::StatusOr<eigenmath::VectorXd> NearbySampling(
      const eigenmath::VectorXd& seed_configuration, double distance,
      int iterations);

  // Takes a random sample from the seeds_ vector and performs a near by
  // sampling. Returns a deadline exceeded error if no valid configuration was
  // found within a certain amount of time.
  absl::StatusOr<eigenmath::VectorXd> SeedExtensionSampling();

  // Generates a random joint configuration either using the
  // OMPLRealVectorStateSpace or the halton sequence depending on the intiation
  // of the member variable halton_sequence_.
  // The returned configuration is within the joint limits of the robot, but
  // not necessarily valid. I.e., the configuration needs to be checked with
  // isValid(config).
  absl::StatusOr<eigenmath::VectorXd> QuasiRandomJointConfiguration();

  // Generates a random joint configuration nearby config with a +/- distance
  // using the halton sequence.
  // The returned configuration is within the joint limits of the robot, but
  // not necessarily valid. I.e., the configuration needs to be checked with
  // isValid(config).
  absl::StatusOr<eigenmath::VectorXd> QuasiRandomJointConfigurationNearby(
      const eigenmath::VectorXd& config, double distance);

  std::function<bool(const eigenmath::VectorXd& q_dest)> dof_validator_fn_;

  // Collision configuration used to explore for near by valid configuration.
  std::vector<Seed> sampling_seeds_;
  // Configurations connected with the goal configuration.
  std::vector<eigenmath::VectorXd> goal_connections_;

  // Probabilities that define where we sample with which probability.
  double goal_sampling_probability_;
  double start_sampling_probability_;
  double straight_line_sampling_probability_;
  double seed_extension_sampling_probability_;

  const int max_sampling_attempts_;
  const double end_point_exploration_distance_;
  const double initial_seed_distance_;

  // Some sampling functions are delegated to the uniform sampler.
  std::unique_ptr<UniformStateSampler> uniform_sampler_;

  // For random number generation.
  int seed_;
  absl::BitGen gen_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_PRM_HEURISTIC_STATE_SAMPLER_H_
