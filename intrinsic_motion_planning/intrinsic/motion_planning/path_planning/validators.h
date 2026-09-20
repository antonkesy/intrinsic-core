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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_VALIDATORS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_VALIDATORS_H_

#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/validators_config.pb.h"
#include "intrinsic/world/collision/collision_types.h"

// A library of different edge and node validation functions.

namespace intrinsic {

// Returns an edge validator with default settings.
absl::StatusOr<EdgeValidator> DefaultEdgeValidator(
    const KinematicsSystemProxy& proxy, double collision_resolution);

// Returns a point validator with default settings.
PointValidator DefaultPointValidator(const KinematicsSystemProxy& proxy);

// This function is used to instantiate an EdgeValidator from
// a generic specification and the KinematicsSystemProxy.
//
// It returns an error if any edge validator fails to instantiate,
// or the implementation cannot find a registered edge validator to instantiate
// from the configuration.
absl::StatusOr<EdgeValidator> CreateEdgeValidator(
    const KinematicsSystemProxy& proxy,
    const proto::EdgeValidatorSpecification& spec);

// This function is used to instantiate a PointValidator from
// a generic specification and the KinematicsSystemProxy.
//
// It returns an error if any point validator fails to instantiate,
// or the implementation cannot find a registered point validator to instantiate
// from the configuration.
absl::StatusOr<PointValidator> CreatePointValidator(
    const KinematicsSystemProxy& proxy,
    const proto::PointValidatorSpecification& spec);

// Contains details about an invalid joint configuration detected during path
// validation, such as whether the collision occurred on an edge, the joint
// angles at the collision point, and the index in the path.
struct InvalidJointConfigurationInfo {
  // True if the collision is on an edge between two configurations of the path.
  bool is_edge_collision = false;
  // The exact joint configuration of the invalid point (if it is an edge
  // collision, this will be the exact joint configuration on the edge).
  eigenmath::VectorXd joint_configuration;
  // The index of the invalid configuration in the path. If it is an edge
  // collision, this will be the index of the first endpoint of the edge.
  int index = -1;
};

// Finds invalid joint configurations along a piecewise linear path.
//
// Validates `config_path` using the kinematics `proxy` and the step resolution
// settings in `spec`. Returns a vector of `InvalidJointConfigurationInfo` for
// all found invalid configurations. If `max_invalid_results` is provided, the
// function returns early after finding that many invalid configurations.
absl::StatusOr<std::vector<InvalidJointConfigurationInfo>>
FindInvalidConfigurationsInPath(
    const KinematicsSystemProxy& proxy,
    absl::Span<const eigenmath::VectorXd> config_path,
    const proto::EdgeValidatorSpecification& spec,
    std::optional<int> max_invalid_results = std::nullopt);

// Holds the outcome of validating a configuration path.
struct ValidateConfigPathResult {
  // True if the path is valid.
  bool valid = false;
  // Details about an invalid configuration found, if the path is invalid.
  std::optional<InvalidJointConfigurationInfo> invalid_config_info;
};

// Validates a piecewise linear path.
//
// Checks the `config_path` using the kinematics `proxy` and the step resolution
// settings in `spec`. Returns a `ValidateConfigPathResult` on success, or an
// error if validation fails to execute.
absl::StatusOr<ValidateConfigPathResult> ValidateConfigPath(
    const KinematicsSystemProxy& proxy,
    absl::Span<const eigenmath::VectorXd> config_path,
    const proto::EdgeValidatorSpecification& spec);

// Returns an EdgeValidatorSpecification with the given validity check
// resolution.
absl::StatusOr<proto::EdgeValidatorSpecification> GetEdgeValidatorSpecification(
    double resolution);

// Returns a default PointValidatorSpecification in case it is not defined.
// The default is a hard margin validator that allows to go through soft
// margins.
proto::PointValidatorSpecification GetDefaultPointValidatorSpecification();

// For a straight line segment with endpoints q1 and q2, this generates a
// sequence of points between q1 and q2 (not inclusive) where the maximum
// distance between neighboring points is no greater than `max_step_size` and
// the ordering of the points follows a binary-search/middle-out ordering. These
// points are appended to `qs`.
absl::Status AddConfigsForBinarySearch(const eigenmath::VectorXd& q1,
                                       const eigenmath::VectorXd& q2,
                                       const double max_step_size,
                                       std::vector<eigenmath::VectorXd>& qs);

}  // namespace intrinsic
#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_VALIDATORS_H_
