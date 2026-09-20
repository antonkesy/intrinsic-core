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

#include "intrinsic/motion_planning/motion_planner/acceleration_limited_trajectory_parameterizer.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/dynamics/robotics_library_dynamics_creator.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/skeleton_util.h"
#include "intrinsic/math/spline/bspline_sampler.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/trajectory_planning/path_refinement/spline_based_path_refinement.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/joint_optimization_options.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/refine_path_and_compute_acceleration_limited_trajectory.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_options.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/trajectory_with_optional_path_samples.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {

AccelerationLimitedTrajectoryParameterizer::
    AccelerationLimitedTrajectoryParameterizer(const BSplineSampler& sampler)
    : sampler_(sampler) {}

absl::StatusOr<topp::TrajectoryWithOptionalPathSamples>
AccelerationLimitedTrajectoryParameterizer::ComputeTrajectory(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    absl::Span<const PathSegment> path_segments,
    const MotionPlannerFlags& flags) const {
  // Construct kinematics chain from base link to (ik solver) tool tip.
  // TODO: b/527039475 - accept Skeleton directly instead of ObjectWorld and
  // KinematicObject
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<kinematics::Skeleton> skeleton,
                        kinematics::GetSkeletonForRobot(world, robot));
  INTR_ASSIGN_OR_RETURN(kinematics::Chain chain,
                        kinematics::CreateChainFromModel(*skeleton));

  // Hardcode options for acceleration-limited trajectory.
  const OptimizationOptions options =
      topp::ToppOptionsWithJointAndCartesianConstraints(
          /*joint_acceleration=*/true, /*joint_jerk=*/false,
          /*cart_velocity=*/true, /*cart_acceleration=*/true);

  // Create a Rigid-Body dynamics interface only if the options have active
  // dynamic constraints (i.e. torque limits or torque-rate limits).
  std::unique_ptr<icon::RigidBodyInterface> dynamics = nullptr;
  if (options.has_dynamic_constraints()) {
    INTR_ASSIGN_OR_RETURN(
        dynamics, icon::CreateRoboticsLibraryDynamics(std::move(skeleton)));
  }

  INTR_ASSIGN_OR_RETURN(
      topp::PathAndTrajectory path_and_trajectory,
      topp::RefinePathAndComputeAccelerationLimitedTrajectory(
          options, path_segments, &chain, dynamics.get(), sampler_));

  return topp::TrajectoryWithOptionalPathSamples{
      .topp_trajectory_result =
          std::move(path_and_trajectory.trajectory_result),
      .path_samples = std::move(path_and_trajectory.path_result.path_samples),
  };
}

}  // namespace intrinsic
