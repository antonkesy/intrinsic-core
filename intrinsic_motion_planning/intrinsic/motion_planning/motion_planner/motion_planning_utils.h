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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNING_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNING_UTILS_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"
#include "intrinsic/motion_planning/motion_planner/robot_specification.h"
#include "intrinsic/motion_planning/motion_planner/trajectory_segment.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/planners/pipeline_configs.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {

// Additional options for planning pipeline that are not always set and depend
// on the planner if they are required.
struct PlannerConfigurationOptions {
  // Cartesian radius of the circular blend around a Cartesian waypoint corner
  // in m.
  std::optional<double> cart_translation_rounding_m;

  // Radius of the circular blend around a waypoint corner (max angles-axis
  // deviation) in radians.
  std::optional<double> cart_rotation_rounding_rad;

  // Step size used to sample the translational Cartesian path. Ideal values
  // ranges between 0.0005 and 0.001.
  std::optional<double> cart_translation_sampling_distance_m;

  // Step size used to sample the rotational Cartesian path. Ideal values ranges
  // between 0.001 and 0.002.
  std::optional<double> cart_rotation_sampling_distance_rad;

  // Optional distance at which the Cartesian path will be uniformly sampled in
  // joint space. If set, an approximately uniform joint sampling scheme is
  // applied. Otherwise, joint samples will be irregularly spaced according to
  // the Cartesian rotational/translational sampling distance included in this
  // configuration. An appropriate value would be in the range [0.005, 0.01].
  std::optional<double> joint_sampling_distance_rad;

  // Spacing between collision checks (in radians in the joint space of the
  // robot). A typical value is 0.01. Increasing this value will speed up
  // collision checking at the expense of possibly having small collisions on
  // the path (in between the discrete points checked).
  double collision_check_spacing_rad = 0.0;

  // Offset between robot tip (flange) and the target frame for which the motion
  // is defined.
  Pose3d tip_t_tool = Pose3d::Identity();
};

// Creates valid path planner pipelines for a motion segment. Depending on the
// properties defined in the motion segment the function returns either a linear
// Cartesian path planner, a joint interpolation path planner, or a planner
// pipeline consisting of rrt and joint shortcutter.
// Configuration options within the motion config and configuration_operations
// that are not required by the planner pipeline defined by the motion segment
// will be ignored.
absl::StatusOr<intrinsic::proto::PipelinePathPlannerConfig>
GetPathPlanningPipelineForMotionSegment(
    const intrinsic_proto::motion_planning::v1::MotionPlannerConfiguration&
        motion_config,
    const intrinsic_proto::motion_planning::v1::MotionSegment& segment,
    const PlannerConfigurationOptions& configuration_options);

// Information required to construct a proxy (along with object_world and
// robot).
struct ProxyCreateInfo {
  std::optional<
      intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
      constraints_proto;
  intrinsic_proto::RuleSet rule_set;
  bool disable_collision_checking = false;
};

// Returns a KinematicsSystemProxy based on the path properties defined in the
// motion segment. The proxy will by default enable collision checking unless
// disabled within the collision settings of the segment. If no collision
// settings are specified in the segment the function will generate a kinematics
// system proxy with the default rule set defined in the world. If
// uniform path constraints are defined in the segment, the proxy will be
// initiated with those constraints and evaluate it within the IsValid function
// together with the joint limit and collision checking evaluation. If collision
// checking is disabled and path constraints are not specified, the returned
// proxy will only check for joint limit violations within the IsValid check.
absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>>
GetKinematicSystemsProxyForSegment(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    const intrinsic_proto::motion_planning::v1::MotionSegment& segment,
    std::optional<int> maybe_concurrent_thread_count = std::nullopt);

// Just like the `GetKinematicSystemsProxyForSegment()` above, except it doesn't
// actually create the proxy: it returns the info needed to construct the proxy.
absl::StatusOr<ProxyCreateInfo> GetKinematicSystemsProxyCreateInfoForSegment(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::MotionSegment& segment);
// Creates the `CartesianKinematicsComponents` containing the forward and
// inverse kinematics functions for the given `proxy` and `tip_t_tool`.
absl::StatusOr<CartesianKinematicsComponents>
CreateCartesianKinematicsComponents(const KinematicsSystemProxy& proxy,
                                    const Pose3d& tip_t_tool);

// Helper function to check if translational rounding is set in the
// `blending_parameters`. If not set, it will return the
// `default_translational_rounding`. Otherwise the value specified in the proto.
// In any case, the value will be clamped to the minimum allowed translational
// rounding `minimum_translational_rounding`.
double GetCartesianTranslationalRoundingFromProto(
    std::optional<intrinsic_proto::motion_planning::v1::BlendingParameters>
        blending_parameters,
    double default_translational_rounding,
    double minimum_translational_rounding);

// Helper function to check if rotational rounding is set in the
// `blending_parameters`. If not set, it will return the
// `default_rotational_rounding`. Otherwise the value specified in the proto.
// In any case, the value will be clamped to the minimum allowed rotational
// rounding `minimum_rotational_rounding`.
double GetCartesianRotationalRoundingFromProto(
    std::optional<intrinsic_proto::motion_planning::v1::BlendingParameters>
        blending_parameters,
    double default_rotational_rounding, double minimum_rotational_rounding);

// Compares joint limits in the given motion_segment and the given joint_limits.
// Returns false if the given motion_segment has joint_limits that differ from
// the given joint_limits for larger than max_allowed_error.
// Raises absl::InvalidArgumentError if
// * The given joint_limit is not valid or has size of zero, or
// * The given motion_segment has joint_limits that are of different size
// from the given joint_limits.
// Otherwise, return true.
absl::StatusOr<bool> CheckIfDynamicJointLimitsAreCompatible(
    const intrinsic_proto::motion_planning::v1::MotionSegment& motion_segment,
    const JointLimits& joint_limits, double max_allowed_error);

// Compares position joint limits in the given motion_segment and the given
// joint_limits.
// Returns false if the given motion_segment min/max joint_position_limits
// differ from the given joint_limits for more than max_allowed_error.
// Otherwise, return true.
absl::StatusOr<bool> CheckIfPositionJointLimitsAreCompatible(
    const intrinsic_proto::motion_planning::v1::MotionSegment& motion_segment,
    const JointLimits& joint_limits, double max_allowed_error);

// Compares cartesian limits in the given motion_segment and the given
// cart_limits.
// Returns false if the given motion_segment cartesian limits differ from the
// given cart_limits for more than max_allowed_vel_error or
// max_allowed_accel_error.
// Otherwise, return true.
absl::StatusOr<bool> CheckIfCartesianLimitsAreCompatible(
    const intrinsic_proto::motion_planning::v1::MotionSegment& motion_segment,
    const CartesianLimits& cart_limits, double max_allowed_vel_error,
    double max_allowed_accel_error);

// Parses the motion specification into a vector of trajectory segments.
// The number of returned trajectory segments is the same or fewer than the
// number of motion segments in the motion specification.
// If there are two or more consecutive linear path motion segments with all
// compatible limits and collision settings, they are merged into a single
// trajectory segment.
absl::StatusOr<std::vector<TrajectorySegment>>
ParseTrajectorySegmentsFromMotionSpecification(
    const object_world::ObjectWorld& object_world,
    const RobotSpecification& robot_specification,
    const intrinsic_proto::motion_planning::v1::MotionSpecification&
        motion_specification);
// Returns the effective collision checking spacing given the default/minimum
// value from `flags` and a possible override in `motion_config`. Returns an
// error if the override in `motion_config` violates the maximum spacing in
// `flags`.
absl::StatusOr<double> GetCollisionCheckSpacing(
    const MotionPlannerFlags& flags,
    const intrinsic_proto::motion_planning::v1::MotionPlannerConfiguration&
        motion_config);

// Determines the number of threads to use for concurrent collision checking.
//
// Returns the thread count specified in `flags`, or `std::nullopt` (indicating
// single-threaded execution) if concurrency is explicitly disabled, the thread
// count is invalid, or collision statistics are being collected.
std::optional<int> GetConcurrentThreadCount(
    const MotionPlannerFlags& flags,
    const DistanceCheckStatistics* distance_check_statistics = nullptr);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNING_UTILS_H_
