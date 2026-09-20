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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_RRT_SCORING_HEURISTIC_STATE_SAMPLER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_RRT_SCORING_HEURISTIC_STATE_SAMPLER_H_

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/motion_planning/path_planning/data_structures/nearest_neighbor_utils.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/uniform_state_sampler.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"

namespace intrinsic {
// A state sampler that uses a scoring heuristic to choose the best candidate
// of randomly generated configurations if context information such as the
// path planning graph, start configuration, and goal configuration are
// provided. Otherwise, this state sampler behaves like a UniformStateSampler,
// i.e., GetRandomConfiguration() and GetRandomConfigurationNearBy() generate
// uniformly distributed configurations within the degree of freedom limits
// of the robot. That also means that all candidates that are generated to be
// scored are generated using the Halton sequence.
//
// Like the UniformStateSampler, the RrtScoringHeuristicStateSampler ensures
// that the generated configurations are within the configuration space defined
// by it's lower_dof_limits and upper_dof_limits. The final check if the
// generated configuration is within the joint limits of the robot happens
// through the dof_validator_fn that is passed along in the constructor.
//
// The scoring of the randomly generated candidates use three heuristics to
// compute the final score:
//   * The configuration should be close to `start_tree` so that we are likely
//     to be able to connect to it. It should have a little bit of spacing to
//     help the tree grow.
//   * Prefer points that are closer to the other tree when possible since we
//     are more likely to be able to connect the two trees this way.
//   * Prefer points away from areas we failed to extend to before. This helps
//     us avoid getting stuck checking the same region over and over.
//
// The parameters of the scoring function are configurable through the
// proto::RrtScoringHeuristicStateSamplerConfig. Default parameters will be
// set if parameters are not specified in the proto.
class RrtScoringHeuristicStateSampler : public UniformStateSampler {
 public:
  static absl::StatusOr<std::unique_ptr<StateSampler>> Create(
      const proto::StateSamplerSpecification& spec,
      const KinematicsSystemProxy& proxy);

  RrtScoringHeuristicStateSampler(
      const proto::RrtScoringHeuristicStateSamplerConfig& config,
      const KinematicsSystemProxy& proxy);

  RrtScoringHeuristicStateSampler(
      const RrtScoringHeuristicStateSampler& other) = default;
  RrtScoringHeuristicStateSampler operator=(
      const RrtScoringHeuristicStateSampler& other) = delete;

  RrtScoringHeuristicStateSampler(RrtScoringHeuristicStateSampler&& other) =
      default;
  RrtScoringHeuristicStateSampler opeator(
      const RrtScoringHeuristicStateSampler&& other) = delete;

  absl::StatusOr<eigenmath::VectorXd> GetRandomConfiguration() override;

  absl::StatusOr<eigenmath::VectorXd> GetRandomConfiguration(
      const eigenmath::VectorXd& start, const eigenmath::VectorXd& goal,
      const PathPlannerGraph* start_graph,
      const PathPlannerGraph* goal_graph) override;

  // Adds a configuration through the set of expand_failures. If the maximum
  // number of failures is reached, the method will automatically remove the
  // oldest configuration in the structure.
  // The key corresponds to the root node of the currently growing tree. I.e.,
  // either the goal or start configuration.
  // The failures are used as part of the scoring function. Sampling close to
  // these configurations is discuraged.
  void AddAvoidanceConfiguration(
      const eigenmath::VectorXd& key,
      const eigenmath::VectorXd& configuration) override;

  // Clears the configurations saved in the expand_failure data structure.
  void ResetAvoidanceConfigurations() override;

 private:
  // Scores a random number candidate based on :
  // * How close it is to the start_graph
  // * How close it is to the goal_graph
  // * How close it is to previous configurations that have been identified as
  //   hard to sample.
  // If the call is successful, the method returns a modified candidate
  // configuration alongside its score. The modified candidate corresponds to
  // the input `candidate` with the exceptions of the tool degrees of freedom
  // that will be biased to the closest configuration in the tree according to
  // the sampling weight set in the state sampler configuration. If the sampling
  // weight is 0, the tool dof will correspond to the existing configuration in
  // the tree. If the weight is 1, the modified candidate will correspond to
  // the input parameter.
  // It is expected that a start graph is provided. If no goal graph is provided
  // it is assumed that the goal is contained in the start_graph and the
  // connected components for the start and goal are extracted from the start
  // graph.
  absl::StatusOr<std::pair<eigenmath::VectorXd, double>> HeuristicScorer(
      const eigenmath::VectorXd& candidate, const eigenmath::VectorXd& start,
      const eigenmath::VectorXd& goal, PathPlannerGraph const* start_graph,
      PathPlannerGraph const* goal_graph);

  proto::RrtScoringHeuristicStateSamplerConfig scoring_heuristic_config_;

  // Contains the known sampled configurations that have been proven to be hard
  // to connect with the graph.
  std::vector<eigenmath::VectorXd> expand_failures_;
  std::vector<eigenmath::VectorXd> expand_failures_goal_;

  // Remembers the last growing graph used in connection with expand_failures_
  eigenmath::VectorXd expand_failure_key_;

  // Weights for computing the weighted euclidean distance.
  eigenmath::VectorXd dof_distance_weights_;

  // Defines to which degree a dof is randomly sampled. Values are expected to
  // range from 0 to 1.
  eigenmath::VectorXd dof_sampling_weights_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_RRT_SCORING_HEURISTIC_STATE_SAMPLER_H_
