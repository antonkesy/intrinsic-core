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

#include "intrinsic/motion_planning/path_planning/planners/rrt_connect.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/data_structures/planner_graph.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/path_planning_utils.h"
#include "intrinsic/motion_planning/path_planning/planners/rrt_configs.pb.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

constexpr double kDistinctConfigurationDistanceThreshold = 2.5e-5;
constexpr size_t kDefaultMaximumExtendSteps = 1000;
constexpr size_t kDefaultMaximumConnectSteps = 1000;

using VertexDescriptor = PathPlannerGraph::VertexDescriptor;

// Signature of a function to be used as a Vertex validator.
// Caller should validate q_dest, and optionally validate that q_origin (if
// present) has a valid transition to q_dest
typedef std::function<bool(const absl::optional<eigenmath::VectorXd>& q_origin,
                           const eigenmath::VectorXd& q_dest)>
    VertexValidationFunction;

// Encodes the result of the extend operation.
// REACHED means that the goal configuration of the operation was reached.
// ADVANCED means that we made progress towards the goal configuration, but did
// not reached it.
// TRAPPED means that no progress was made.
enum class ExtendCode { REACHED, ADVANCED, TRAPPED };

struct ExtendResult {
  ExtendCode extend_code;
  VertexDescriptor vertex_desc;
};

// Move q_near towards q with a step of magnitude eps
eigenmath::VectorXd NewConfig(const eigenmath::VectorXd& q,
                              const eigenmath::VectorXd& q_near, double eps) {
  eigenmath::VectorXd q_new;
  eigenmath::VectorXd distance = q - q_near;
  double dist = (distance).norm();
  if (dist <= eps) {
    q_new = q;
  } else {
    auto normalisation = (distance) / dist;
    q_new = q_near + normalisation * eps;
  }
  return q_new;
}

// Iterate from the current position to the nearby one, extending
// the graph with each successful collision check
//
// This approach ensures there aren't any duplicate collision checks
// and the graph grows as much as possible for every collision check
// that identifies the pose as free space.
absl::StatusOr<ExtendResult> Extend(
    PathPlannerGraph* graph, const eigenmath::VectorXd& configuration,
    const VertexDescriptor q_near_id, double step_size,
    const eigenmath::VectorXd q_goal,
    const VertexValidationFunction& validity_func,
    const ConstraintManifoldProjector& project_func, bool need_exact_connection,
    const size_t maximum_extend_steps = kDefaultMaximumExtendSteps) {
  // This is a variant on the original RRT extend function
  // Here, the tree continues to take more than one step towards the random
  // configuration as long it is valid and closer to the goal than the previous
  // one

  // Check if the graph is defined. This is a check as this should never happen.
  CHECK_NE(graph, nullptr);

  VertexDescriptor temp_q_near_id = q_near_id;
  eigenmath::VectorXd q_near, q_new;
  size_t num_extend_steps = 0;
  while (num_extend_steps++ < maximum_extend_steps) {
    INTR_ASSIGN_OR_RETURN(q_near, graph->GetConfiguration(temp_q_near_id));
    q_new = NewConfig(configuration, q_near, step_size);

    // Project the new configuration `q_new` onto the constraint manifold.
    INTR_ASSIGN_OR_RETURN(q_new, project_func(q_new));
    if ((q_new - q_near).norm() < kDistinctConfigurationDistanceThreshold) {
      // We consider this case as the case where the resultant new
      // configuration `q_new` being projected back to the same configuration
      // as (or a configuration which is too similar to) `q_near`. Thus, it
      // makes sense to terminate the loop here, to avoid meaningless
      // repetitive node extension and projection (which only adds the number
      // of nodes in the graph), but doesn't actually make any progress at
      // all.
      break;
    }

    if (validity_func && !validity_func(q_near, q_new)) {
      break;
    }
    double near_dist2goal = (q_goal - q_near).norm();
    double new_dist2goal = (q_goal - q_new).norm();
    INTR_ASSIGN_OR_RETURN(
        VertexDescriptor q_new_id,
        graph->AddVertex(q_new, proto::PathPlannerGraph::VALID));
    INTR_RETURN_IF_ERROR(graph->AddEdge(temp_q_near_id, q_new_id,
                                        proto::PathPlannerGraph::VALID));
    if (need_exact_connection) {
      if (q_new == configuration) {
        // Note: it is safe to check exact equality here because `q_new =
        // NewConfig(configuration, ...)` returns exactly `configuration` when
        // it is close enough.
        return ExtendResult(ExtendCode::REACHED, q_new_id);
      }
    } else {
      if ((q_new - configuration).norm() < step_size) {
        return ExtendResult{ExtendCode::ADVANCED, q_new_id};
      }
    }
    if (new_dist2goal < near_dist2goal) {
      temp_q_near_id = q_new_id;
    } else {
      return ExtendResult{ExtendCode::ADVANCED, q_new_id};
    }
  }

  if (temp_q_near_id == q_near_id) {
    ExtendResult result({ExtendCode::TRAPPED, temp_q_near_id});
    return result;
  } else {
    ExtendResult result({ExtendCode::ADVANCED, temp_q_near_id});
    return result;
  }
}

absl::StatusOr<ExtendResult> Connect(
    PathPlannerGraph* graph, const eigenmath::VectorXd& q, double step_size,
    const eigenmath::VectorXd& q_goal,
    const VertexValidationFunction& validity_func,
    const ConstraintManifoldProjector& project_func,
    const size_t maximum_connect_steps = kDefaultMaximumConnectSteps,
    const size_t maximum_extend_steps = kDefaultMaximumExtendSteps) {
  // Check if the graph is defined. This is a check as this should never happen.
  CHECK_NE(graph, nullptr);

  ExtendResult r;
  INTR_ASSIGN_OR_RETURN(r.vertex_desc, graph->GetNearestNeighborForConfig(q));
  size_t num_connect_steps = 0;
  do {
    INTR_ASSIGN_OR_RETURN(
        r, Extend(graph, q, r.vertex_desc, step_size, q_goal, validity_func,
                  project_func, /*need_exact_connection=*/true,
                  maximum_extend_steps));
  } while (r.extend_code == ExtendCode::ADVANCED &&
           (++num_connect_steps < maximum_connect_steps));
  return r;
}

}  // namespace

absl::StatusOr<PointPath> PlanBidirectionalRrtConnect(
    const eigenmath::VectorXd& start, const eigenmath::VectorXd& goal,
    const proto::RrtConnectConfig& config,
    const PointValidator& point_validator, const EdgeValidator& edge_validator,
    std::unique_ptr<StateSampler> state_sampler,
    const ConstraintManifoldProjector& projector, PathPlannerGraph* graph) {
  // Check if start and goal are valid configurations
  INTR_ASSIGN_OR_RETURN(bool start_valid, point_validator(start));
  INTR_ASSIGN_OR_RETURN(bool goal_valid, point_validator(goal));
  if (!start_valid || !goal_valid) {
    return absl::InvalidArgumentError(
        "Cannot plan with rrt connect. Start and goal configuration need to be "
        "valid, but did not pass the validity check.");
  }
  LOG(INFO) << "PlanBidirectionalRrtConnect():";
  LOG(INFO) << "  start:\t" << start.transpose();
  LOG(INFO) << "  goal:\t" << goal.transpose();
  VLOG(1) << "  start-goal dist:\t" << (goal - start).norm();

  CHECK(graph);
  CHECK(state_sampler);

  const eigenmath::VectorXd& q_start = start;
  const eigenmath::VectorXd& q_goal = goal;

  // Create PathPlannerGraph as RRT from existing graph
  auto start_exists = graph->ExistInGraph(start);
  auto goal_exists = graph->ExistInGraph(goal);
  VertexDescriptor start_vertex_descriptor = start_exists.first;
  VertexDescriptor goal_vertex_descriptor = goal_exists.first;

  // Check if path already exist between start and goal
  if (start_exists.second && goal_exists.second) {
    auto vertex_path =
        graph->AStarSearch(start_vertex_descriptor, goal_vertex_descriptor);
    if (!vertex_path.empty()) {
      return graph->GetConfigurationVector(vertex_path);
    }
  }

  if (!start_exists.second) {
    INTR_ASSIGN_OR_RETURN(
        start_vertex_descriptor,
        graph->AddVertex(start, proto::PathPlannerGraph::VALID));
  }
  if (!goal_exists.second) {
    INTR_ASSIGN_OR_RETURN(
        goal_vertex_descriptor,
        graph->AddVertex(goal, proto::PathPlannerGraph::VALID));
  }
  INTR_ASSIGN_OR_RETURN(
      PathPlannerGraph t_start,
      graph->GetConnectedTree(start, {proto::PathPlannerGraph::VALID}, true));
  INTR_ASSIGN_OR_RETURN(
      PathPlannerGraph t_goal,
      graph->GetConnectedTree(goal, {proto::PathPlannerGraph::VALID}, true));

  // Copy the start and end so that we can swap them as we grow the trees from
  // both directions.
  eigenmath::VectorXd tmp_start = q_start;
  eigenmath::VectorXd tmp_goal = q_goal;

  // Unfortunately, the legacy code uses a validator that only returns a bool
  // rather than a StatusOr<bool>. We wrap our validator with value or die for
  // now and will later correct the error handling.
  auto dying_validator = [&](const std::optional<eigenmath::VectorXd>& from,
                             const eigenmath::VectorXd& to) {
    if (!from.has_value()) {
      return point_validator(to).value();
    }
    return edge_validator(*from, to).value();
  };

  PathPlannerGraph *t_a = &t_start, *t_b = &t_goal;
  absl::Time start_time = absl::Now();
  for (int iterations = 0; iterations < config.max_iters(); ++iterations) {
    // Swap pointers to grow RRTs from both sides. We only swap if tree t_a is
    // larger than t_b ensuring we grow both trees evenly.
    // Note, it has been observed that some problems were sensitive of this
    // check to happen before the first time the sampling occurs. Moving the
    // following to the end can cause high dof problems to fail.
    if (t_a->Size() > t_b->Size()) {
      std::swap(t_a, t_b);
      tmp_start.swap(tmp_goal);
    }

    double elapsed_time_seconds =
        absl::ToDoubleSeconds(absl::Now() - start_time);
    if (elapsed_time_seconds > 0.0 &&
        elapsed_time_seconds > config.timeout_seconds()) {
      INTR_RETURN_IF_ERROR(graph->MergeGraph(*t_a));
      INTR_RETURN_IF_ERROR(graph->MergeGraph(*t_b));
      return absl::DeadlineExceededError(absl::StrFormat(
          "Planner could not find a path in the given time of %.3f seconds. "
          "Most likely reason is that there is no path that satisfies the "
          "given constraints or that the time out is set too low. "
          "Try to increase the planning time out or check if the configuration "
          "space is fully reachable.",
          config.timeout_seconds()));
    }

    VLOG_EVERY_N_SEC(1, 180)
        << "Still planning after " << iterations << " iterations. Request from "
        << toString(q_start) << " to " << toString(q_goal);

    eigenmath::VectorXd q_rand;

    // Always select random points (according to defined state sampler) if no
    // goal point  check is required. Select the goal point on every nth
    // occasion (if n >=1), where n is stored in iters_between_goal_checks.
    if (config.iters_between_goal_checks() < 1 ||
        ((iterations > 0) &&
         (iterations % config.iters_between_goal_checks() != 0))) {
      INTR_ASSIGN_OR_RETURN(q_rand, state_sampler->GetRandomConfiguration(
                                        tmp_start, tmp_goal, t_a, t_b));
    } else {
      // Try going towards the goal directly on occasion.
      q_rand = tmp_goal;
    }

    INTR_ASSIGN_OR_RETURN(VertexDescriptor closest_to_q_rand,
                          t_a->GetNearestNeighborForConfig(q_rand));
    INTR_ASSIGN_OR_RETURN(
        ExtendResult result_a,
        Extend(t_a, q_rand, closest_to_q_rand, config.step_size(), tmp_goal,
               dying_validator, projector, /*need_exact_connection=*/false));

    if (result_a.extend_code != ExtendCode::TRAPPED) {
      // Extension successful, new vertex 'result_a.vertex_desc' was added to
      // LegacyRRT 'a'.
      INTR_ASSIGN_OR_RETURN(eigenmath::VectorXd q_new,
                            t_a->GetConfiguration(result_a.vertex_desc));
      // Trying to connect LegacyRRT 'b' to new vertex 'q_new' in RRT 'a'. Since
      // we're connecting from the current 'goal' tree to the current 'start'
      // tree, we use 'tmp_start' as the 'q_goal' argument. This encourages
      // growth toward the root node of the 'a' tree.
      INTR_ASSIGN_OR_RETURN(
          ExtendResult result_b,
          Connect(t_b, q_new, config.step_size(), /*q_goal=*/tmp_start,
                  dying_validator, projector));

      if (result_b.extend_code == ExtendCode::REACHED) {
        // Save result in path planning graph
        // Each of the trees is a connected component that has at least the
        // first component (== root component) identical to the graph.
        INTR_RETURN_IF_ERROR(graph->MergeGraph(*t_a));
        INTR_RETURN_IF_ERROR(graph->MergeGraph(*t_b));
        auto connected_vertex_a = graph->ExistInGraph(q_new);
        INTR_ASSIGN_OR_RETURN(eigenmath::VectorXd q_new_b_tree,
                              t_b->GetConfiguration(result_b.vertex_desc));
        auto connected_vertex_b = graph->ExistInGraph(q_new_b_tree);
        if (!connected_vertex_a.second || !connected_vertex_b.second) {
          return absl::InternalError(
              "Planning algorithm completed, but could not find connecting "
              "vertices in joined graph.");
        }
        INTR_RETURN_IF_ERROR(graph->AddEdge(connected_vertex_a.first,
                                            connected_vertex_b.first,
                                            proto::PathPlannerGraph::VALID));

        auto vertex_path =
            graph->AStarSearch(start_vertex_descriptor, goal_vertex_descriptor);
        if (vertex_path.empty()) {
          return absl::InternalError(
              "Planning algorithm completed, but no path could be found.");
        }
        const double elapsed_time_seconds =
            absl::ToDoubleSeconds(absl::Now() - start_time);
        VLOG(1) << "RRT-Connect planning time: " << elapsed_time_seconds
                << " s";
        // TODO(b/438811921): Should we collision-check this path before
        // returning it?
        INTR_ASSIGN_OR_RETURN(PointPath config_path,
                              graph->GetConfigurationVector(vertex_path));
        if (ABSL_VLOG_IS_ON(1)) {
          const double path_length = ComputePointPathLength(config_path);
          VLOG(1) << "RRT-Connect path length: " << path_length;
          VLOG(1) << "RRT-Connect path node count: " << config_path.size();
          VLOG(1) << "RRT-Connect iterations: " << iterations;
          VLOG(1) << "RRT-Connect graph node count: " << graph->Size();
        }
        return config_path;
      }
    } else {
      INTR_ASSIGN_OR_RETURN(eigenmath::VectorXd closest_to_q_rand_config,
                            t_a->GetConfiguration(closest_to_q_rand));
      q_rand = (q_rand + closest_to_q_rand_config) * 0.5;
      state_sampler->AddAvoidanceConfiguration(tmp_start, q_rand);
    }
  }

  INTR_RETURN_IF_ERROR(graph->MergeGraph(*t_a));
  INTR_RETURN_IF_ERROR(graph->MergeGraph(*t_b));

  return absl::DeadlineExceededError(
      absl::StrFormat("Planning exceeded iteration deadline of %d. "
                      "There is most likely no solution that satisfies the "
                      "defined path constraints.",
                      config.max_iters()));
}

}  // namespace intrinsic
