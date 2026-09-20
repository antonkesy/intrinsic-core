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

#include "intrinsic/motion_planning/path_planning/data_structures/rrt_scoring_heuristic_state_sampler.h"

#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/random_number_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/data_structures/planner_graph.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/uniform_state_sampler.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/util/proto/parse_text_proto.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {

namespace {
using ::intrinsic::ParseTextProtoOrDie;
// Returns the function f(x) = (width * (center - x)^2 + 1)^-1.
//
// This function always has max 1 and approaches zero for x = +-inf. It is
// useful to simulate a "local" effect around a point.
double InverseSquare(double x, double center, double width) {
  double d = (width * (center - x) * (center - x) + 1);
  return 1.0 / d;
}

// Returns a configurations that merges into the provided config unset values
// with reasonable values.
proto::RrtScoringHeuristicStateSamplerConfig SetDefaultConfig(
    const proto::RrtScoringHeuristicStateSamplerConfig& config) {
  proto::RrtScoringHeuristicStateSamplerConfig default_config =
      ParseTextProtoOrDie(R"pb(
        growing_tree_spacing: 0.1
        growing_tree_width: 5
        connecting_tree_spacing: 0
        connecting_tree_width: 0.1
        connecting_tree_weight: 0.1
        failure_spacing: 0
        failure_width: 20
        failure_weight: 0.2
        num_candidates: 100
        max_num_failures_stored: 100
        ;
      )pb");
  default_config.MergeFrom(config);
  return default_config;
}

}  // namespace

RrtScoringHeuristicStateSampler::RrtScoringHeuristicStateSampler(
    const proto::RrtScoringHeuristicStateSamplerConfig& config,
    const KinematicsSystemProxy& proxy)
    : UniformStateSampler(proto::UniformStateSamplerConfig(), proxy),
      scoring_heuristic_config_(SetDefaultConfig(config)),
      expand_failure_key_(eigenmath::VectorXd::Zero(sampling_limits_.size())) {
  // Number of candidates to generate should be greater than zero.
  CHECK_GT(scoring_heuristic_config_.num_candidates(), 0)
      << "Number of random configurations to store must be greater than zero.";

  // Number of max_num_failures_stored must be a non negative value.
  CHECK_GE(scoring_heuristic_config_.max_num_failures_stored(), 0)
      << "Number of stored failure cases needs to be a non negative value.";

  // Set the configuration validator using the proxy.
  dof_validator_fn_ = [&proxy](const eigenmath::VectorXd& q_dest) -> bool {
    return proxy.IsWithinLimits(q_dest).value();
  };

  const int ndof = sampling_limits_.size();

  // Get dof labels to differentiate between arm and tool dof and create
  // euclidean distance and sampling weights for each of the dofs depending on
  // their label.
  const std::vector<DofLabel> dof_labels = proxy.GetDofLabels();
  CHECK_EQ(sampling_limits_.size(), dof_labels.size());
  dof_distance_weights_ = eigenmath::VectorXd::Ones(ndof);
  dof_sampling_weights_ = eigenmath::VectorXd::Ones(ndof);
  constexpr double kScaleNorm = 2 * M_PI;
  for (int i = 0; i < dof_labels.size(); ++i) {
    if (dof_labels[i] != labels::ToolPart()) {
      if (scoring_heuristic_config_.arm_dof_range_scale()) {
        dof_distance_weights_[i] =
            kScaleNorm / (sampling_limits_.max_position[i] -
                          sampling_limits_.min_position[i]);
      }
    } else {
      dof_sampling_weights_[i] =
          scoring_heuristic_config_.tool_dof_sampling_weight();
      if (scoring_heuristic_config_.tool_dof_range_scale()) {
        dof_distance_weights_[i] =
            kScaleNorm / (sampling_limits_.max_position[i] -
                          sampling_limits_.min_position[i]);
      }
    }
  }

  // Overall distance weights should be scaling according to the sampling.
  dof_distance_weights_ =
      dof_distance_weights_.cwiseProduct(dof_sampling_weights_);
  dof_distance_weights_ = dof_distance_weights_.array().sqrt();
}

absl::StatusOr<eigenmath::VectorXd>
RrtScoringHeuristicStateSampler::GetRandomConfiguration() {
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
        eigenmath::VectorXd config,
        eigenmath::GetQuasiRandomVectorXd(sampling_limits_.min_position,
                                          sampling_limits_.max_position,
                                          &halton_sequence_index_));
    if (scoring_heuristic_config_.use_equidirectional_sampling_method() ||
        dof_validator_fn_(config)) {
      return config;
    }
  }
}

absl::StatusOr<eigenmath::VectorXd>
RrtScoringHeuristicStateSampler::GetRandomConfiguration(
    const eigenmath::VectorXd& start, const eigenmath::VectorXd& goal,
    const PathPlannerGraph* start_graph, const PathPlannerGraph* goal_graph) {
  // If graph contains values and option to perform equidirectional sampling
  // reduce sampling range if possible to an area of the lower and upper bound
  // of the graph samples.
  if (scoring_heuristic_config_.use_equidirectional_sampling_method() &&
      start_graph->Size() >= 1) {
    INTR_ASSIGN_OR_RETURN(auto start_lower_upper,
                          start_graph->GetLowerAndUpperValueBounds());
    JointLimitsXd new_limits;
    new_limits.SetSize(start_lower_upper.first.size());
    new_limits.min_position =
        start_lower_upper.first -
        scoring_heuristic_config_.equidirectional_sampling_buffer() *
            eigenmath::VectorXd::Ones(start_lower_upper.first.size());
    new_limits.max_position =
        start_lower_upper.second +
        scoring_heuristic_config_.equidirectional_sampling_buffer() *
            eigenmath::VectorXd::Ones(start_lower_upper.first.size());
    INTR_RETURN_IF_ERROR(SetAndClampSamplingLimtis(new_limits));
  }
  // If not more than one configuration will be generated to score against,
  // return a uniformly distributed random configuration.
  if (scoring_heuristic_config_.num_candidates() < 2) {
    return GetRandomConfiguration();
  }

  // Generate the candidates to score against. Each candidate is generated
  // via a uniform state sampler.
  std::vector<eigenmath::VectorXd> candidates(
      scoring_heuristic_config_.num_candidates());
  for (int i = 0; i < scoring_heuristic_config_.num_candidates(); i++) {
    // Generate random configuration
    INTR_ASSIGN_OR_RETURN(candidates[i], GetRandomConfiguration());
  }

  // Check if the tree was changed from the last call
  if (expand_failure_key_.isZero()) {
    expand_failure_key_ = start;
  }
  if (!expand_failure_key_.isApprox(start)) {
    // Change key and expand_failure_start_
    expand_failure_key_ = start;
    expand_failures_.swap(expand_failures_goal_);
  }

  // Set start and goal graph if only one graph is provided. It is expected that
  // in this case the start_graph contains both connected components for the
  // start and goal configurations. We are only looking for valid edges to
  // define 'connected'.
  std::optional<PathPlannerGraph> start_connected_component,
      goal_connected_component(std::nullopt);
  if (goal_graph == nullptr) {
    INTR_ASSIGN_OR_RETURN(goal_connected_component,
                          start_graph->GetConnectedGraphWithEdgeProperty(
                              goal, {proto::PathPlannerGraph::VALID}));
    goal_graph = &goal_connected_component.value();
    INTR_ASSIGN_OR_RETURN(start_connected_component,
                          start_graph->GetConnectedGraphWithEdgeProperty(
                              start, {proto::PathPlannerGraph::VALID}));
    start_graph = &start_connected_component.value();
  }

  // Find candidate with heighest score.
  double max_score = -std::numeric_limits<double>::max();
  eigenmath::VectorXd result;
  for (eigenmath::VectorXd& candidate : candidates) {
    double score = 0;
    // Score the candidates. If a tool dof sampling weight is set, the result
    // will be biased towards the closest candidate in the tree.
    INTR_ASSIGN_OR_RETURN(
        auto score_result,
        HeuristicScorer(candidate, start, goal, start_graph, goal_graph));
    score = score_result.second;
    if (score > max_score) {
      max_score = score;
      result = score_result.first;
    }
  }
  return result;
}

absl::StatusOr<std::pair<eigenmath::VectorXd, double>>
RrtScoringHeuristicStateSampler::HeuristicScorer(
    const eigenmath::VectorXd& candidate, const eigenmath::VectorXd& start,
    const eigenmath::VectorXd& goal, PathPlannerGraph const* start_graph,
    PathPlannerGraph const* goal_graph) {
  CHECK(start_graph);
  CHECK(goal_graph);

  double score = 0;
  eigenmath::VectorXd modified_candidate = candidate;

  // Reward for being close to the tree we are currently expanding.
  if (start_graph->Size() >= 1) {
    INTR_ASSIGN_OR_RETURN(auto start_tree_nearest_vertex,
                          start_graph->GetNearestNeighborForConfig(candidate));
    INTR_ASSIGN_OR_RETURN(
        eigenmath::VectorXd start_tree_nearest,
        start_graph->GetConfiguration(start_tree_nearest_vertex));
    // Scale the tool joint sampling according the sampling weights. In
    // case the weight is zero, use the value of the closest configuration in
    // the tree.
    modified_candidate =
        start_tree_nearest +
        (candidate - start_tree_nearest).cwiseProduct(dof_sampling_weights_);
    double distance_to_start_tree =
        (dof_distance_weights_.cwiseProduct(modified_candidate -
                                            start_tree_nearest))
            .norm();
    if (distance_to_start_tree <
        scoring_heuristic_config_.growing_tree_spacing()) {
      score += distance_to_start_tree /
               scoring_heuristic_config_.growing_tree_spacing();
    } else {
      score += InverseSquare(distance_to_start_tree,
                             scoring_heuristic_config_.growing_tree_spacing(),
                             scoring_heuristic_config_.growing_tree_width());
    }
  }

  // Reward for being close to the other tree.
  if (goal_graph->Size() >= 1) {
    INTR_ASSIGN_OR_RETURN(
        auto end_tree_nearest_vertex,
        goal_graph->GetNearestNeighborForConfig(modified_candidate));
    INTR_ASSIGN_OR_RETURN(
        eigenmath::VectorXd end_tree_nearest,
        goal_graph->GetConfiguration(end_tree_nearest_vertex));
    double distance_to_end_tree = (dof_distance_weights_.cwiseProduct(
                                       modified_candidate - end_tree_nearest))
                                      .norm();
    score += scoring_heuristic_config_.connecting_tree_weight() *
             InverseSquare(distance_to_end_tree,
                           scoring_heuristic_config_.connecting_tree_spacing(),
                           scoring_heuristic_config_.connecting_tree_width());
  }

  // Penalty for failing to expand to that point before.
  double penalty = 0;
  for (const eigenmath::VectorXd& failure : expand_failures_) {
    // TODO(b/131909306): Can this be improved ? norm involes sqrt (=slow)!
    double distance = (failure - modified_candidate).norm();
    penalty +=
        -scoring_heuristic_config_.failure_weight() *
        InverseSquare(distance, scoring_heuristic_config_.failure_spacing(),
                      scoring_heuristic_config_.failure_width());
  }
  score += penalty;

  return std::make_pair(modified_candidate, score);
}

void RrtScoringHeuristicStateSampler::AddAvoidanceConfiguration(
    const eigenmath::VectorXd& key, const eigenmath::VectorXd& configuration) {
  if (!expand_failure_key_.isApprox(key)) {
    // Change key and expand_failure_start_
    expand_failure_key_ = key;
    expand_failures_.swap(expand_failures_goal_);
  }

  expand_failures_.push_back(configuration);
  if (expand_failures_.size() >
      scoring_heuristic_config_.max_num_failures_stored()) {
    expand_failures_.erase(expand_failures_.begin());
  }
}

void RrtScoringHeuristicStateSampler::ResetAvoidanceConfigurations() {
  expand_failures_.clear();
}

absl::StatusOr<std::unique_ptr<StateSampler>>
RrtScoringHeuristicStateSampler::Create(
    const proto::StateSamplerSpecification& spec,
    const KinematicsSystemProxy& proxy) {
  proto::RrtScoringHeuristicStateSamplerConfig proto;
  if (!spec.config().UnpackTo(&proto)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to unpack config to specified type. Failure occurred for ",
        spec.name()));
  }
  auto state_sampler =
      std::make_unique<RrtScoringHeuristicStateSampler>(proto, proxy);
  return std::unique_ptr<StateSampler>(std::move(state_sampler));
}

REGISTER_STATE_SAMPLER(RrtScoringHeuristicStateSampler,
                       "RrtScoringHeuristicStateSampler",
                       RrtScoringHeuristicStateSampler::Create);

}  // namespace intrinsic
