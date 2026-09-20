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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNER_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/no_destructor.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "cppregpattern/registry.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/path_planner.pb.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"

namespace intrinsic {
// Main interface for planning a path.
//
// This interface is set up to support different types of planning including:
//   * traditional planner (RRT, PRM)
//   * path smoothing (shortcutting)
//   * other path modification (padding a path around collisions).
//
// Each PathPlanner takes an input path and returns an output path, with the
// only requirement being that the boundary point of the input path match the
// boundary points of the output path.
//
// PathPlanners may have some semantic expectation of its input paths. For
// example, a shortcutting planner may expect that all input paths are already
// collision free. Each PathPlanner is responsible for checking that required
// preconditions are satisfied for each input and outputting appropriate error
// status when those conditions are not met.
class PathPlanner {
 public:
  virtual ~PathPlanner() = default;

  // Makes a new path given the input `path`. The only requirement on the output
  // is that the first and last point in the output match the first and last
  // points in the input respectively.
  //
  // Implementations can do vastly different things. Some might implement a full
  // path planning algorithm like RRT and find a completely new path. Others
  // might perform shortcutting, or other modification on the given points.
  //
  // Since implementations may have different expectation about the types of
  // input paths they expect, each should carefully document its requirements
  // and check that the required preconditions hold.
  //
  // `proxy` provides access to robot and world specific information such as
  // collision checking and FK/IK.
  //
  // `graph` provides access to a graph that may store additional information
  // about points and edges checked by the implementation. Because results in
  // `graph` may depend on the `proxy` the two must be kept in sync (caller
  // knows to modify the two together) or implementation must carefully check
  // that the right preconditions hold. `graph` could be nullptr.
  //
  // The caller should guarantee the lifetime of the passed objects for the
  // duration off the call, and ensure thread safety of those objects if shared
  // between multiple threads.
  virtual absl::StatusOr<std::vector<PointPath>> Plan(
      const PointPath& path, const KinematicsSystemProxy& proxy,
      PathPlannerGraph* graph) const = 0;

  // Performs precomputation. This call is expected to be called once (if at
  // all) before long stages of planning so that the implementation can
  // construct appropriate data to speed up computation of the Plan calls. This
  // could involve adding data to `graph` or other internal data structures.
  //
  // To give an example, a PRM implementation of PathPlanner might use this call
  // to initialize `graph` with a roadmap that connects most of the points it
  // wants to plan.
  //
  // Implementations of this function should be written in a way that this is
  // NOT required for Plan (though it should presumably make subsequent calls to
  // Plan faster).
  //
  // The client can provide a list of paths if these are known ahead of time but
  // it is up to the implementation whether it wants to use this list or not.
  virtual absl::Status Precompute(const std::vector<PointPath>& paths_to_plan,
                                  const KinematicsSystemProxy& proxy,
                                  PathPlannerGraph* graph) const = 0;
};

// Classes and macros below defines a distributed way for registering subclasses
// of PathPlanner. Example usage:
//
// <To register a path planner subclass, in 'magic_planner.h' file>
//
// #include ".../path_planner.h"
//
// class MagicPlanner : public PathPlanner {
//  public:
//   explicit MagicPlanner(const MagicPlannerConfigProto& config);
// };
//
// REGISTER_PATH_PLANNER(MagicPlanner, "MagicPlanner", MagicPlannerConfigProto);
//
//
// <To use the registered class in an 'apply_magic.cc' file>
//
// #include ".../magic_planner.h"
//
// ...
//
// std::unique_ptr<PathPlanner> planner;
//
// INTR_ASSIGN_OR_RETURN(planner,
//                  PathPlannerFactory::Create("MagicPlanner", any_config));
//
// planner.Plan(...);
//
// ...
//
// <End of example>
//
// Note: 'any_config' must be of proto 'Any'.  If not, use
//
//   'google::protobuf::Any any_config; any_config.PackFrom(config);'
//
// to get 'any_config'. Initializing planners in this way probably does not
// make too much sense for normal usage, but can be convenient for composite
// planners, e.g. 'PipelinePathPlanner', which wants to be able to initialize
// arbitrary planners based on configurations of various types of protos.

class PathPlannerFactory {
 public:
  using CreateSignature = absl::StatusOr<std::unique_ptr<PathPlanner>>(
      const std::optional<google::protobuf::Any>& any_config);

  static absl::StatusOr<std::unique_ptr<PathPlanner>> Create(
      const std::string& planner_name,
      const std::optional<google::protobuf::Any>& any_config);
};

using PathPlannerRegistry = registry::Registry<
    std::string,
    absl::StatusOr<std::unique_ptr<PathPlanner>>(
        const std::optional<google::protobuf::Any>& any_config),
    registry::MissingKeyPolicy::default_construct>;

#define REGISTER_PATH_PLANNER(name, alias, FactoryName)                       \
  [[maybe_unused]] const bool kUnused##name =                                 \
      intrinsic::PathPlannerRegistry::Register(                               \
          alias, [](const std::optional<google::protobuf::Any>& any_config) { \
            return FactoryName(any_config);                                   \
          });
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNER_H_
