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

#ifndef INTRINSIC_KINEMATICS_UTILS_COMPUTE_IK_UTIL_H_
#define INTRINSIC_KINEMATICS_UTILS_COMPUTE_IK_UTIL_H_

#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_newton_raphson_ik_solver.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/proto/motion_target.pb.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/transform_node.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// Timeout for numeric solver used in the proximity of kinematic singularities
// to guarantee limited time impact.
const absl::Duration kTimeoutSingularityRobustIkSolver = absl::Milliseconds(5);

// Options for ik.
struct ComputeIkOptions {
  // The starting joint configuration to use. If empty (=default), the current
  // position of a robot in the world will be used.
  eigenmath::VectorXd seed_configuration;

  // The joint limits that the solution should respect. If not set (=default),
  // the application limits of the world will be used.
  std::optional<JointLimitsXd> joint_limits;

  // The maximum number of solutions to be returned. If not set (== 0), the
  // underlying implementation has the freedom to choose. Negative values are
  // invalid.
  //
  // Choosing a smaller value may make some implementations faster, but this
  // depends on the underlying implementation and is not guaranteed.
  int max_num_solutions = 8;

  // Optional same branch IK flag. Defaults to false.
  bool ensure_same_branch = false;

  // Optional same branch Ik flag that will prefer solutions on the same
  // kinematic branch over those close to the starting_joint configuration.
  // Defaults to false;
  bool prefer_same_branch = false;

  // Optional flag to disable setting the error status when there are no valid
  // solutions due to collisions. When false, the request will fail if no
  // collision free solution is found. When true, the request will succeed even
  // if no collision free solution is found. This is useful when accessing the
  // collision debug information. Defaults to false.
  bool disable_error_on_collisions = false;
};

struct ComputeIkDebugInformation {
  struct IkSolution {
    eigenmath::VectorXd configuration;
    JointConfigurationValidationResult validation_result;
    std::optional<CollisionCheckingDebug> collision_checking_debug;
  };

  std::vector<IkSolution> ik_solutions;
};

// Compute ik solutions for the specified parameters. This function does not
// check collisions for any of the solutions and so is useful for enumerating
// all the options.
absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::CartesianMotionTarget&
        cartesian_motion_target,
    const ComputeIkOptions& options = {});

// Compute Ik solutions that satisfy the geometric constraints specified
// parameters. This function does not check collisions for any of the solutions.
absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint,
    const ComputeIkOptions& options = {});

// Compute ik solutions for the specified parameters. In contrast to other
// methods, it allows to specify if the same branch solution should be used as
// preferred solution. I.e., if the same branch solution is available this will
// be the first value in the vector of returned solution. If not the closest to
// the current configuration will be used. This function does not check
// collisions for any of the solutions and is useful for enumerating all the
// options.
absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::PoseEquality&
        cartesian_pose_constraint,
    const ComputeIkOptions& options = {});

// Compute ik solutions for the specified parameters. This function does not
// check collisions for any of the solutions and so is useful for enumerating
// all the options.
absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const World& world, RobotCollectionsEntityId robot_id,
    AttachmentEntityId reference_id, AttachmentEntityId object_id,
    const Pose3d& reference_t_object, const ComputeIkOptions& options = {});

// Compute same branch ik solution for the specified parameters. This function
// does not check collisions for the solution.
// Note: This function is currently supported for only spherical wrist robots.
absl::StatusOr<eigenmath::VectorXd> ComputeSameBranchIk(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::CartesianMotionTarget&
        cartesian_motion_target);

// Compute same branch ik solution for the specified parameters. This function
// does not check collisions for the solution.
// Note: This function is currently supported for only spherical wrist robots.
absl::StatusOr<eigenmath::VectorXd> ComputeSameBranchIk(
    const World& world, RobotCollectionsEntityId robot_id,
    AttachmentEntityId reference_id, AttachmentEntityId object_id,
    const Pose3d& reference_t_object);

// Compute same branch ik solution for the specified parameters. This function
// does not check collisions for the solution.
// Note: This function is currently supported for only spherical wrist robots.
absl::StatusOr<eigenmath::VectorXd> ComputeSameBranchIk(
    const World& world, const CartesianKinematicView& cartesian_view,
    AttachmentEntityId reference_id, AttachmentEntityId object_id,
    const Pose3d& reference_t_object,
    std::optional<eigenmath::VectorXd> seed_configuration = std::nullopt);

// Returns for the given robot in the given world the IK solution which
// results in the given reference_t_object pose and, if multiple solutions
// exist, which is closest to the current configuration of the robot. Uses the
// proxy for computing the collision checking instead of the world. Returns an
// error if either a) the goal pose is not reachable or b) the goal pose is
// reachable but there is no collision-free IK solution (see IsNotReachable()
// and IsInCollision() below).
absl::StatusOr<eigenmath::VectorXd> GetClosestIkSolution(
    const World& world, const KinematicsSystemProxy& proxy,
    RobotCollectionsEntityId robot_id, AttachmentEntityId reference_id,
    AttachmentEntityId object_id, const Pose3d& reference_t_object);

// Returns for the given robot in the given world the IK solution which
// satisfies the specified constraints and is collision free. Returns an error
// if either a) the constraint cannot be satisfied or b) non of the found
// solutions are collision free.
// Attempts to find ik_solutions_requested that are collision free, i.e., more
// ik solutions might be sampled in order to find at least this number and a
// smaller number that provided might be found and returned..
absl::StatusOr<std::vector<eigenmath::VectorXd>> GetCollisionFreeIkSolutions(
    const object_world::ObjectWorld& world, const KinematicsSystemProxy& proxy,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint,
    const ComputeIkOptions& options = {},
    ComputeIkDebugInformation* debug_information = nullptr);

// Returns for the given robot in the given world the IK solution which
// results in the given cartesian_motion_target and is on the same branch as the
// current joint state of the arm. Uses the proxy for computing the collision
// checking instead of the world. Returns an error if either a) the goal is not
// reachable or b) does not have an IK solution on the same branch as the
// current state or c) the goal is reachable but there is no collision-free IK
// solution (see IsNotReachable() and IsInCollision() below).
absl::StatusOr<eigenmath::VectorXd> GetSameBranchIkSolution(
    const object_world::ObjectWorld& world, const KinematicsSystemProxy& proxy,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::CartesianMotionTarget&
        cartesian_motion_target);

// Returns for the given robot in the given world the IK solution which
// results in the given cartesian_motion_target and is on the same branch as the
// current joint state of the arm. Uses the proxy for computing the collision
// checking instead of the world. Returns an error if either a) the goal is not
// reachable or b) does not have an IK solution on the same branch as the
// current state or c) the goal is reachable but there is no collision-free IK
// solution (see IsNotReachable() and IsInCollision() below).
absl::StatusOr<eigenmath::VectorXd> GetSameBranchIkSolution(
    const World& world, const KinematicsSystemProxy& proxy,
    RobotCollectionsEntityId robot_id, AttachmentEntityId reference_id,
    AttachmentEntityId object_id, const Pose3d& reference_t_object);

// Returns true if the status returned by GetClosestIkSolutionInWorld() above
// indicates failure because the desired goal pose is not reachable.
bool IsNotReachable(const absl::Status& status);

// Returns true if the status returned by GetClosestIkSolutionInWorld() above
// indicates failure because there is no collision-free IK solution.
bool IsInCollision(const absl::Status& status);

// Returns OK if the provided collision parameters are valid, and a status
// error otherwise.
absl::Status ValidateCollisionSettings(
    const intrinsic_proto::world::CollisionSettings& collision_settings);

// Converts a the `motion_target` to a Pose3d of the `target_node` frame
// relative to the `reference_node` frame. When moving the `target_node` frame
// to the returned pose the `motion_target` is reached.
absl::StatusOr<Pose3d> CartesianMotionTargetToPose3d(
    const world::ObjectWorldClient& object_world,
    const intrinsic_proto::motion_planning::CartesianMotionTarget&
        motion_target,
    world::TransformNode reference_node, world::TransformNode target_node);

// Resolves multiple joint position limit constraints in a given a vector of
// Geometric constraints, to provide the most restrictive set of joint
// position limits.
absl::StatusOr<intrinsic_proto::motion_planning::v1::GeometricConstraint>
ResolveJointPositionLimitsConstraints(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    std::vector<intrinsic_proto::motion_planning::v1::GeometricConstraint>&
        constraints);

// Updates the target constraints given a vector of Geometric constraints.
// TODO (b/279637923): Improve the resolution of target constraints.
// Currently we only support the resolution of joint position limits
// constraints, the rest of them are added to the target constraint as is.
absl::StatusOr<intrinsic_proto::motion_planning::v1::GeometricConstraint>
UpdateTargetConstraints(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    std::vector<intrinsic_proto::motion_planning::v1::GeometricConstraint>&
        constraints);

// Computes singularity-robust IK solutions for the given `base_t_tip` computed
// from `base_t_target` and `tip_t_target`. The initial solution is computed
// using the IK solver of the `kinematics`, given the
// `hint_joint_configuration`, `joint_limits`, and the `ensure_same_branch`
// flag. If the initial solution is close to singularity,
// `singularity_robust_ik_solver` (if provided) will be used to compute a more
// singularity-robust IK solution. The returned solutions are ranked by their
// distance to either the `closest_hint_joint_configuration` (if provided) or
// the `hint_joint_configuration`. The `closest_hint_joint_configuration` for
// example can be provided from the solution of the IK problem of an adjacent/
// closest pose in a path of poses.
// This function does not check collisions for any of the solutions.
absl::StatusOr<std::vector<eigenmath::VectorNd>> ComputeSingularityRobustIK(
    const icon::ManipulatorKinematics& kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    const Pose3d& base_t_target, const Pose3d& tip_t_target,
    const JointLimits& joint_limits,
    std::optional<eigenmath::VectorNd> closest_hint_joint_configuration =
        std::nullopt,
    kinematics::KinematicChainRandomSeedIKSolver* singularity_robust_ik_solver =
        nullptr,
    bool ensure_same_branch = false);

}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_UTILS_COMPUTE_IK_UTIL_H_
