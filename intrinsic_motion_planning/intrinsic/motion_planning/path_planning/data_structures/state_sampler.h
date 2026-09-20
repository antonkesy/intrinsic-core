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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_STATE_SAMPLER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_STATE_SAMPLER_H_

#include <memory>
#include <string>

#include "absl/base/no_destructor.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "cppregpattern/registry.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"

namespace intrinsic {
// Main interface for state sampling used within the path planning.
//
// There exist many approaches on how to produce samples of the state space
// that increase the likelihood of the path planning algorithm to find a
// solution or to increase the quality of the found solution.
//
// This interface is set up to support different kinds of state sampling
// approaches for path planning including:
// * Traditional probability distributions, e.g., uniform state sampling.
// * Heuristics for state sampling
// * Informed state sampling.
//
// Each StateSampler is expected to generate a configuration within the joint
// limits of the robot, but no guarantees regarding the validity, or the
// distribution of the generated configurations are made.
class StateSampler {
 public:
  explicit StateSampler(const JointLimitsXd& system_limits)
      : system_limits_(system_limits), sampling_limits_(system_limits) {}

  virtual ~StateSampler() = default;
  // Returns a random configuration for which IsWithinLimits of the
  // KinematicsSystemProxy will return true but IsValid might not.
  // This call should be fast, i.e., no collision checks should be performed.
  //
  // No guarantees are made about the sampling distribution.
  //
  // A returned error indicates a problem within the generation of the
  // configuration and not the absence of a valid configuration.
  virtual absl::StatusOr<eigenmath::VectorXd> GetRandomConfiguration() = 0;

  // Returns a random configuration for which IsWithinLimits of the
  // KinematicsSystemProxy will return true, but IsValid might not. This call
  // should be fast, i.e., no collision checks should be performed.
  //
  // Interface provides additional information to the state sampling process for
  // path planning between two configurations (start and goal) using a
  // PathPlannerGraph.
  //
  // If only one PathPlannerGraph is provided, it is assumed that the goal
  // configuration is contained in the start_graph as well.
  //
  // A returned error indicates a problem within the generation of the
  // configuration and not the absence of a valid configuration.
  virtual absl::StatusOr<eigenmath::VectorXd> GetRandomConfiguration(
      const eigenmath::VectorXd& start, const eigenmath::VectorXd& goal,
      const PathPlannerGraph* start_graph,
      const PathPlannerGraph* goal_graph) = 0;

  // Returns a random configuration that is near by a seed_configuration within
  // the +/- distance and for which the IsWithinLimits of the
  // KinematicsSystemProxy will return true (though IsValid may not).
  //
  // No guarantees are made about the sampling distribution.
  //
  // A returned error indicates a problem within the generation of the
  // configuration and not the absence of a valid configuration.
  virtual absl::StatusOr<eigenmath::VectorXd> GetRandomConfigurationNearby(
      const eigenmath::VectorXd& seed_configuration, double distance) = 0;

  // Returns a random configuration that is near by a seed_configuration within
  // a given range where the distance to the configuration is defined for each
  // degree of freedom individually. The returned configuration IsWithinLimits
  // of the KinematicsSystemProxy though IsValid of the KinematicsSystemProxy
  // might not return true.
  //
  // A returned error indicates a problem within the generation of the
  // configuration and not the absence of a valid configuration.
  virtual absl::StatusOr<eigenmath::VectorXd> GetRandomConfigurationNearby(
      const eigenmath::VectorXd& seed_configuration,
      const eigenmath::VectorXd& distance) = 0;

  // Adds a configuration to the list of sampling seeds. The
  // configuration is usually expected to be in a hard to sample region that
  // we would like to focus on.
  virtual void AddSamplingSeedConfiguration(
      const eigenmath::VectorXd& configuration) = 0;

  // Adds a configuration to avoid to the state sampling process. This informs
  // the state sampler about areas that should be avoided to be
  // sampled for example because they are not relevant or known to be invalid.
  // The interface allows to specify a key in form of a configuration. This
  // key should correspond to the start or goal configuration. It allows to
  // define different avoid configuration sets for the two different trees:
  // start and goal tree.
  virtual void AddAvoidanceConfiguration(
      const eigenmath::VectorXd& key,
      const eigenmath::VectorXd& configuration) = 0;

  // Resets the set of avoidance configurations.
  virtual void ResetAvoidanceConfigurations() = 0;

  // Sets sampling configurations that are different but within the system
  // limits.
  // Some sampler needs to update their state space when the limits are changed,
  // hence the pure virtual.
  virtual absl::Status SetSamplingLimits(const JointLimitsXd& limits) = 0;

  // Sets sampling configurations that are different but within the system
  // limits. If the suggested limits exceed the system limits, the limits will
  // be automatically clamped.
  virtual absl::Status SetAndClampSamplingLimtis(
      const JointLimitsXd& limits) = 0;

 protected:
  absl::Status SetSamplingLimitsCommon(const JointLimitsXd& limits);

  JointLimitsXd system_limits_;
  JointLimitsXd sampling_limits_;
};

// Registration mechanism for factories that create the state sample.
using StateSamplerRegistry =
    registry::Registry<std::string,
                       absl::StatusOr<std::unique_ptr<StateSampler>>(
                           const proto::StateSamplerSpecification&,
                           const KinematicsSystemProxy&),
                       registry::MissingKeyPolicy::default_construct>;

#define REGISTER_STATE_SAMPLER(name, alias, FactoryName)            \
  [[maybe_unused]] const bool kUnused##name =                       \
      intrinsic::StateSamplerRegistry::Register(                    \
          alias, [](const proto::StateSamplerSpecification& config, \
                    const KinematicsSystemProxy& proxy) {           \
            return FactoryName(config, proxy);                      \
          });
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_STATE_SAMPLER_H_
