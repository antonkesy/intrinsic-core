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

#include "intrinsic/kinematics/utils/compute_ik_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_interface.h"
#include "intrinsic/kinematics/ik/constrained/cost_functions.h"
#include "intrinsic/kinematics/ik/constrained/nlopt_constrained_ik_solver.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_ik_solver.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_newton_raphson_ik_solver.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/numopt/costfunction_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto/pose.pb.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/motion_planning/data_types/constraints/joint_position/joint_position_limits.h"
#include "intrinsic/motion_planning/data_types/constraints/utils/geometric_constraints_proto_utils.h"
#include "intrinsic/motion_planning/motion_planner/motion_planning_error_utils.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/path_planning_utils.h"
#include "intrinsic/motion_planning/proto/motion_specification_proto_utils.h"
#include "intrinsic/motion_planning/proto/motion_target.pb.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/skills/proto/motion_targets.pb.h"
#include "intrinsic/stats/scoped_span.h"
#include "intrinsic/util/aggregate_type.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_types.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/objects/transform_node.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/util/entity_search_util.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

namespace {

using ::intrinsic_proto::motion_planning::v1::GeometricConstraint;
using GeometricConstraintCase =
    ::intrinsic_proto::motion_planning::v1::GeometricConstraint::ConstraintCase;

using IKResult = kinematics::InverseKinematicsInterface::IKResult;

// Clip too long error messages to avoid gRPC issues. This can be removed once
// we have a more general solution for b/232245718.
std::string ClipTooLongErrorMessage(absl::string_view message) {
  if (message.size() > 3500) {
    return absl::StrCat(message.substr(0, 3500), "... [", message.size() - 3500,
                        " more characters]");
  } else {
    return std::string(message);
  }
}

// Sets the application limits in the robot defined by the DofKinematicView. In
// case the current joint config is out of limits it will change the current
// config in the dof view to one inside the new limits.
absl::Status SetApplicationLimits(const JointLimitsXd& joint_limits,
                                  DofKinematicView* dof_view) {
  const eigenmath::VectorNd robot_config = dof_view->GetDofValues();
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto limit_check_result,
                                IsWithinLimits(robot_config, joint_limits));
  if (!limit_check_result.p_ok) {
    INTR_RETURN_IF_ERROR(
        dof_view->SetDofValues(joint_limits.min_position, true));
  }
  return dof_view->SetDofApplicationLimits(joint_limits,
                                           /*enforce_limits=*/true);
}

absl::Status UpdateJointPositionLimits(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const GeometricConstraint& geometric_constraints,
    JointLimits& joint_limits) {
  const std::vector<GeometricConstraint> joint_limit_constraints =
      ExtractConstraintType(geometric_constraints,
                            GeometricConstraintCase::kJointPositionLimits);
  if (joint_limit_constraints.size() > 1) {
    return absl::InvalidArgumentError(
        "Provided geometric constraints contain more than one set of joint "
        "limit constraints. Please ensure that only one is provided.");
  }
  if (!joint_limit_constraints.empty()) {
    if (!joint_limit_constraints.front().has_joint_position_limits()) {
      return absl::InternalError(
          "Requested geometric constraint of type `joint_position_limit`, but "
          "received constraint does not contain requested type.");
    }

    INTR_RETURN_IF_ERROR(SetJointPositionLimitsFromProto(
        world, robot, joint_limit_constraints.front().joint_position_limits(),
        joint_limits));
  }

  return absl::OkStatus();
}

absl::StatusOr<std::optional<eigenmath::VectorNd>>
ParseJointPositionEqualityConstraints(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const JointLimits& application_limits,
    const GeometricConstraint& geometric_constraint) {
  std::queue<GeometricConstraint> queue;
  queue.push(geometric_constraint);
  std::optional<eigenmath::VectorNd> parsed_joint_configuration;
  bool joint_limit_constraint_set = false;
  JointLimits planning_limits = application_limits;
  intrinsic_proto::motion_planning::v1::ConstraintIntersection
      cartesian_constraints;
  while (!queue.empty()) {
    const GeometricConstraint constraint = queue.front();
    queue.pop();

    if (constraint.has_constraint_intersection()) {
      for (const auto& element :
           constraint.constraint_intersection().constraints()) {
        queue.push(element);
      }
      continue;
    }
    if (constraint.has_joint_position()) {
      if (parsed_joint_configuration.has_value()) {
        return absl::InvalidArgumentError(
            "Geometric constraint contains multiple joint equality "
            "constraints.");
      }
      parsed_joint_configuration =
          RepeatedDoubleToVectorXd(constraint.joint_position().joints());
      continue;
    }
    if (constraint.has_joint_position_limits()) {
      if (joint_limit_constraint_set) {
        return absl::InvalidArgumentError(
            "Geometric constraint contains multiple joint position limit "
            "constraints.");
      }
      INTR_RETURN_IF_ERROR(SetJointPositionLimitsFromProto(
          world, robot, constraint.joint_position_limits(), planning_limits));
      joint_limit_constraint_set = true;
      continue;
    }

    *cartesian_constraints.add_constraints() = constraint;
  }
  // If joint limits are parsed, check if the configuration is in limits
  if (parsed_joint_configuration.has_value()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto joint_configuration_within_limits,
        IsWithinLimits(parsed_joint_configuration.value(), planning_limits));
    if (!joint_configuration_within_limits.p_ok) {
      const std::string error_message = absl::StrCat(
          "Defined joint configuration [",
          toString(parsed_joint_configuration.value()),
          "] is outside the joint configuration "
          "limits.\n Upper limits: [",
          toString(planning_limits.max_position), "].", "\n Lower limits: [",
          toString(planning_limits.min_position), "].");
      absl::string_view kMotionPlanningErrorSuffix = "validation";
      return CreateStatusWithJointLimitError(
          error_message, parsed_joint_configuration.value(),
          kMotionPlanningErrorSuffix, ErrorContext::CONF_VALIDATION,
          absl::StatusCode::kInvalidArgument, planning_limits);
    }
  }

  bool satisfies_constraints = true;
  if (parsed_joint_configuration.has_value() &&
      cartesian_constraints.constraints_size() != 0) {
    GeometricConstraint cartesian_geometric_constraint;
    *cartesian_geometric_constraint.mutable_constraint_intersection() =
        cartesian_constraints;
    INTR_ASSIGN_OR_RETURN(
        std::vector<std::unique_ptr<ConstraintInterface>> resolved_constraints,
        GetConstraintsFromProto(world, robot, cartesian_geometric_constraint));
    for (const auto& resolved_constraint : resolved_constraints) {
      INTR_ASSIGN_OR_RETURN(
          satisfies_constraints,
          resolved_constraint->IsSatisfied(parsed_joint_configuration.value()));
      if (!satisfies_constraints) {
        break;
      }
    }
  }

  // Check if non_compatible_constraint are set
  if (parsed_joint_configuration.has_value() && !satisfies_constraints) {
    return absl::InvalidArgumentError(
        "Motion target contains conflicting constraints. The defined joint "
        "position violates at least one of the other motion constraints.");
  }
  return parsed_joint_configuration;
}

// Identifies if a given target frame is the tool frame. Background: Geometric
// constraints allow to arbitrarily assign frames. Cartesian Motion Target
// requires differentiation.
absl::StatusOr<bool> IsTargetToolFrame(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const AttachmentEntityId& reference_id,
    const AttachmentEntityId& target_id) {
  INTR_ASSIGN_OR_RETURN(auto ancestor_ref_target_id,
                        object_world.GetEntityWorld().FindCommonAncestor(
                            reference_id, target_id));
  INTR_ASSIGN_OR_RETURN(auto base_id, robot.GetRootEntityId());
  INTR_ASSIGN_OR_RETURN(
      auto ancestor_base_target_id,
      object_world.GetEntityWorld().FindCommonAncestor(base_id, target_id));

  if (target_id == reference_id) {
    // Special case.
    return true;
  }

  bool target_is_ancestor_of_ref = (ancestor_ref_target_id == target_id);
  if (target_is_ancestor_of_ref) {
    // target is an ancestor of ref or the same. I.e., it cannot be the tool
    // frame.
    return false;
  }
  // If ref is not an ancestor of target and target is not an ancestor of ref
  // then target is tool frame if it is connected to the robot base.
  bool ref_is_ancestor_of_target = (ancestor_ref_target_id == reference_id);
  bool base_is_ancestor_of_target = (ancestor_base_target_id == base_id);
  if (!ref_is_ancestor_of_target && !base_is_ancestor_of_target) {
    return false;
  }
  return true;
}

std::optional<intrinsic_proto::motion_planning::v1::PoseEquality>
ParsePoseEqualityConstraint(
    const object_world::ObjectWorld& world,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint) {
  if (geometric_constraint.has_cartesian_pose()) {
    return geometric_constraint.cartesian_pose();
  }
  if (geometric_constraint.has_constraint_intersection() &&
      geometric_constraint.constraint_intersection().constraints().size() ==
          1 &&
      geometric_constraint.constraint_intersection()
          .constraints(0)
          .has_cartesian_pose()) {
    intrinsic_proto::motion_planning::v1::PoseEquality pose_equality_proto =
        geometric_constraint.constraint_intersection()
            .constraints(0)
            .cartesian_pose();
    return pose_equality_proto;
  }
  if (geometric_constraint.has_relative_cartesian_pose()) {
    absl::StatusOr<intrinsic_proto::motion_planning::v1::PoseEquality>
        maybe_pose_equality_proto =
            ConvertRelativePoseConstraintToPoseConstraint(
                world, geometric_constraint.relative_cartesian_pose());
    if (maybe_pose_equality_proto.ok()) {
      return *maybe_pose_equality_proto;
    }
  }
  return std::nullopt;
}

// Returns true if the vector already contains a similar value up to a
// similarity threshold of `similarity_threshold`.
bool VectorContainsSimilarValues(
    const std::vector<eigenmath::VectorXd>& solution_vec,
    const eigenmath::VectorXd& value, double similarity_precision) {
  for (const auto& solution : solution_vec) {
    if (solution.isApprox(value, similarity_precision)) {
      return true;
    }
  }
  return false;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> SampleSolutionsWithNlopt(
    const kinematics::ModelInterface* model,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints,
    std::vector<std::unique_ptr<CostFunctionInterface>> costs,
    const eigenmath::VectorNd& initial_guess, int max_solutions_sampled) {
  const stats::ScopedSpan span(
      "PathPlanningUtil/SampleSolutionsWithNloptWithConstraintsAndCosts");
  // Use under constrained inverse kinematics solver to find joint configuration
  // that fulfills the specified constraints.
  INTR_ASSIGN_OR_RETURN(kinematics::Chain chain,
                        kinematics::CreateChainFromModel(*model));
  auto iksolver =
      std::make_unique<kinematics::NLOptConstrainedIKSolver>(std::move(chain));
  INTR_RETURN_IF_ERROR(
      iksolver->SetProblem(std::move(constraints), std::move(costs)));

  // Attempt to compute max_ik_solution_sampled different solutions. Only the
  // first attempt gets the initial guess as a hint, in all subsequent
  // attempts the seed is chosen deterministically using a unique Halton
  // sequence index to ensure that the initial guesses are different in every
  // iteration.
  std::vector<eigenmath::VectorXd> solutions;
  solutions.reserve(max_solutions_sampled);
  for (int i = 0; i < max_solutions_sampled; ++i) {
    kinematics::ConstrainedIKInterface::Options ik_options{
        .max_number_of_attempts = /* max attempts */ 40,
        .halton_sequence_index = i};
    if (i == 0) {
      ik_options.joint_position_initial_guess = initial_guess;
    }
    INTR_ASSIGN_OR_RETURN(const auto result, iksolver->Solve(ik_options));
    if (result.status == result.OK) {
      if (!result.q_star) {
        return absl::InternalError(
            "Solver returned ok but did not provide solution.");
      }
      if (!VectorContainsSimilarValues(solutions, result.q_star->position,
                                       /*similarity_precision=*/1e-3)) {
        solutions.push_back(result.q_star->position);
      }
    }
  }
  return solutions;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> SampleSolutionsWithNlopt(
    const kinematics::ModelInterface* model,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints,
    const eigenmath::VectorNd& initial_guess, bool set_joint_position_cost,
    int max_solutions_sampled) {
  const stats::ScopedSpan span(
      "PathPlanningUtil/SampleSolutionsWithNloptWithConstraints");
  std::vector<std::unique_ptr<CostFunctionInterface>> costs;
  if (set_joint_position_cost) {
    INTR_ASSIGN_OR_RETURN(kinematics::Chain chain,
                          kinematics::CreateChainFromModel(*model));
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<kinematics::JointPositionCost> joint_position_cost,
        kinematics::JointPositionCost::Create(&chain, initial_guess, 1));
    costs.emplace_back(std::move(joint_position_cost));
  }
  return SampleSolutionsWithNlopt(model, std::move(constraints),
                                  std::move(costs), initial_guess,
                                  max_solutions_sampled);
}

// Replaces inf values in the joint position values with `initial_guess` +/-
// restriction_range.
absl::StatusOr<JointLimits> RestrictInfJointPositionValues(
    const kinematics::ModelInterface* kinematic_model,
    const JointLimits& joint_limits, const eigenmath::VectorXd& initial_guess,
    double restriction_range) {
  if (joint_limits.min_position.size() != joint_limits.max_position.size()) {
    return absl::InternalError(absl::StrCat(
        "Minimum and maximum position limit size differ. Need to be the "
        "same. Min position limit size ",
        joint_limits.min_position.size(), ", max position limit size ",
        joint_limits.max_position.size()));
  }

  if (joint_limits.min_position.size() != initial_guess.size()) {
    return absl::InternalError(
        absl::StrCat("Size of the initial guess provided and the size of "
                     "position limits differ. Position limit size ",
                     joint_limits.min_position.size(), ", initial guess size ",
                     initial_guess.size()));
  }

  // Check for infinite position limit size and restrict it to desired range
  // around initial_guess.
  JointLimits out = joint_limits;
  for (int i = 0; i < out.min_position.size(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto joint_id,
                                  kinematic_model->GetElementIdForDofIndex(i));
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint,
                                  kinematic_model->GetJoint(joint_id));
    if (joint->GetParameters().type == kinematics::Joint::Type::REVOLUTE) {
      if (std::isinf(out.min_position[i]) ||
          out.min_position[i] == -std::numeric_limits<double>::max()) {
        out.min_position[i] = initial_guess[i] - restriction_range;
        LOG(INFO) << "Min limits for joint " << i
                  << " will be reduced from inf to " << out.min_position[i]
                  << ".";
      }
      if (std::isinf(out.max_position[i]) ||
          out.max_position[i] == std::numeric_limits<double>::max()) {
        out.max_position[i] = initial_guess[i] + restriction_range;
        LOG(INFO) << "Max limits for joint " << i
                  << " will be reduced from inf to " << out.max_position[i]
                  << ".";
      }
    }
  }
  return out;
}

// Function limits the range of motion of each joint to a max of initial guess
// +/- `max_range`. Method will not extend the range of motion in one direction
// more than max_range or the original joint limit.
absl::StatusOr<JointLimits> RestrictJointPositionValuesToMaxRange(
    const kinematics::ModelInterface* kinematic_model,
    const JointLimits& joint_limits, const eigenmath::VectorXd& initial_guess,
    double max_range) {
  if (joint_limits.min_position.size() != joint_limits.max_position.size()) {
    return absl::InternalError(absl::StrCat(
        "Minimum and maximum position limit size differ. Need to be the "
        "same. Min position limit size ",
        joint_limits.min_position.size(), ", max position limit size ",
        joint_limits.max_position.size()));
  }
  if (joint_limits.min_position.size() != initial_guess.size()) {
    return absl::InternalError(
        absl::StrCat("Size of the initial guess provided and the size of "
                     "position limits differ. Position limit size ",
                     joint_limits.min_position.size(), ", initial guess size ",
                     initial_guess.size()));
  }

  JointLimits out = joint_limits;
  for (int i = 0; i < out.min_position.size(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto joint_id,
                                  kinematic_model->GetElementIdForDofIndex(i));
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint,
                                  kinematic_model->GetJoint(joint_id));
    if (joint->GetParameters().type == kinematics::Joint::Type::REVOLUTE) {
      if (std::abs(out.max_position[i] - out.min_position[i]) > 2 * max_range) {
        out.min_position[i] =
            std::max(initial_guess[i] - max_range, out.min_position[i]);
        out.max_position[i] =
            std::min(initial_guess[i] + max_range, out.max_position[i]);
      }
    }
  }
  return out;
}

// Converts and adds `planning limits` to the set of constraints. The final set
// of constraints contains all constraints defined in `constraints` and adds or
// updates existing joint limit restriction to stay within the
// `planning_limits`.
absl::StatusOr<GeometricConstraint> AddAndResolveJointPositionLimitsConstraints(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const JointLimits& planning_limits,
    const GeometricConstraint& constraints) {
  // Convert and add planning limits to set of constraints.
  const auto joint_limit_constraint_proto = ToJointPositionLimitProto(
      robot, planning_limits.min_position, planning_limits.max_position);
  GeometricConstraint geometric_constraint_joint_limit;
  *geometric_constraint_joint_limit.mutable_joint_position_limits() =
      joint_limit_constraint_proto;
  std::vector<GeometricConstraint> constraints_with_limits;
  constraints_with_limits.emplace_back(constraints);
  constraints_with_limits.emplace_back(geometric_constraint_joint_limit);
  return ResolveJointPositionLimitsConstraints(object_world, robot,
                                               constraints_with_limits);
}

GeometricConstraint GetGeometricConstraint(
    intrinsic_proto::motion_planning::CartesianMotionTarget
        cartesian_motion_target) {
  GeometricConstraint geometric_constraint;
  *geometric_constraint.mutable_cartesian_pose()->mutable_moving_frame() =
      cartesian_motion_target.tool();
  *geometric_constraint.mutable_cartesian_pose()->mutable_target_frame() =
      cartesian_motion_target.frame();
  *geometric_constraint.mutable_cartesian_pose()
       ->mutable_target_frame_offset() = cartesian_motion_target.offset();
  return geometric_constraint;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const World& world, RobotCollectionsEntityId robot_id,
    const intrinsic_proto::skills::CartesianMotionTarget&
        cartesian_motion_target,
    const ComputeIkOptions& options) {
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId frame_id,
                        GetSingleTypedEntity<AttachmentEntityId>(
                            world, cartesian_motion_target.frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId tool_id,
                        GetSingleTypedEntity<AttachmentEntityId>(
                            world, cartesian_motion_target.tool()));
  Pose3d frame_t_tool = Pose3d::Identity();
  if (cartesian_motion_target.has_offset()) {
    INTR_ASSIGN_OR_RETURN(
        frame_t_tool, FromProtoNormalized(cartesian_motion_target.offset()));
  }
  return ComputeIk(world, robot_id, frame_id, tool_id, frame_t_tool, options);
}

absl::StatusOr<eigenmath::VectorXd> GetSameBranchIkSolution(
    const World& world, const KinematicsSystemProxy& proxy,
    RobotCollectionsEntityId robot_id,
    const intrinsic_proto::skills::CartesianMotionTarget&
        cartesian_motion_target) {
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId frame_id,
                        GetSingleTypedEntity<AttachmentEntityId>(
                            world, cartesian_motion_target.frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId tool_id,
                        GetSingleTypedEntity<AttachmentEntityId>(
                            world, cartesian_motion_target.tool()));
  Pose3d frame_t_tool = Pose3d::Identity();
  if (cartesian_motion_target.has_offset()) {
    INTR_ASSIGN_OR_RETURN(
        frame_t_tool, FromProtoNormalized(cartesian_motion_target.offset()));
  }

  return GetSameBranchIkSolution(world, proxy, robot_id, frame_id, tool_id,
                                 frame_t_tool);
}

absl::StatusOr<eigenmath::VectorXd> ComputeSameBranchIk(
    const World& world, RobotCollectionsEntityId robot_id,
    const intrinsic_proto::skills::CartesianMotionTarget&
        cartesian_motion_target) {
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId frame_id,
                        GetSingleTypedEntity<AttachmentEntityId>(
                            world, cartesian_motion_target.frame()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId tool_id,
                        GetSingleTypedEntity<AttachmentEntityId>(
                            world, cartesian_motion_target.tool()));
  Pose3d frame_t_tool = Pose3d::Identity();
  if (cartesian_motion_target.has_offset()) {
    INTR_ASSIGN_OR_RETURN(frame_t_tool, intrinsic_proto::FromProtoNormalized(
                                            cartesian_motion_target.offset()));
  }
  return ComputeSameBranchIk(world, robot_id, frame_id, tool_id, frame_t_tool);
}

}  // namespace

absl::StatusOr<intrinsic_proto::motion_planning::v1::GeometricConstraint>
UpdateTargetConstraints(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    std::vector<intrinsic_proto::motion_planning::v1::GeometricConstraint>&
        constraints) {
  // TODO (b/279637923) : Improve the resolution of the target constraints when
  // using path constraints for planning. Currently we only resolve the joint
  // position limits
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::motion_planning::v1::GeometricConstraint
          updated_constraints,
      ResolveJointPositionLimitsConstraints(object_world, robot, constraints));
  return updated_constraints;
}

absl::StatusOr<intrinsic_proto::motion_planning::v1::GeometricConstraint>
ResolveJointPositionLimitsConstraints(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    std::vector<intrinsic_proto::motion_planning::v1::GeometricConstraint>&
        constraints) {
  INTR_ASSIGN_OR_RETURN(JointLimitsXd planning_limits_xd,
                        robot.GetJointApplicationLimits());
  eigenmath::VectorXd resolved_upper_limit = planning_limits_xd.max_position;
  eigenmath::VectorXd resolved_lower_limit = planning_limits_xd.min_position;
  if (resolved_upper_limit.size() == 0 || resolved_lower_limit.size() == 0) {
    return absl::InternalError(
        "Error while retrieving the robot application limits. Position limits "
        "are invalid.");
  }
  intrinsic_proto::motion_planning::v1::ConstraintIntersection
      resolved_constraints;

  std::queue<intrinsic_proto::motion_planning::v1::GeometricConstraint> queue;
  for (auto& geom_constraint : constraints) {
    queue.push(geom_constraint);
  }
  bool contains_joint_limit_constraint = false;
  while (!queue.empty()) {
    const auto constraint = queue.front();
    queue.pop();
    switch (constraint.constraint_case()) {
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kConstraintIntersection: {
        // For an intersection we can add all constraints on the queue.
        for (const auto& element :
             constraint.constraint_intersection().constraints()) {
          queue.push(element);
        }
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          kJointPositionLimits: {
        // When constructing the geometric joint limits we ensure to stay within
        // the application limits set for the robot when creating the
        // constraint.
        INTR_ASSIGN_OR_RETURN(
            auto joint_limit_constraint,
            motion_planning::JointPositionLimitsConstraint::Create(
                object_world, constraint.joint_position_limits(), 0));
        const eigenmath::VectorNd& upper_limits =
            joint_limit_constraint->GetUpperLimits();
        const eigenmath::VectorNd& lower_limits =
            joint_limit_constraint->GetLowerLimits();
        resolved_upper_limit = resolved_upper_limit.cwiseMin(upper_limits);
        resolved_lower_limit = resolved_lower_limit.cwiseMax(lower_limits);
        contains_joint_limit_constraint = true;
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kJointPosition: {
        *resolved_constraints.add_constraints()->mutable_joint_position() =
            constraint.joint_position();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kCartesianPose: {
        *resolved_constraints.add_constraints()->mutable_cartesian_pose() =
            constraint.cartesian_pose();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kPositionEquality: {
        *resolved_constraints.add_constraints()->mutable_position_equality() =
            constraint.position_equality();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kRotationCone: {
        *resolved_constraints.add_constraints()->mutable_rotation_cone() =
            constraint.rotation_cone();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kRotationEquality: {
        *resolved_constraints.add_constraints()->mutable_rotation_equality() =
            constraint.rotation_equality();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kPositionBoundingBox: {
        *resolved_constraints.add_constraints()
             ->mutable_position_bounding_box() =
            constraint.position_bounding_box();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kJointPositionSumLimit: {
        *resolved_constraints.add_constraints()
             ->mutable_joint_position_sum_limit() =
            constraint.joint_position_sum_limit();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kPointAt: {
        *resolved_constraints.add_constraints()->mutable_point_at() =
            constraint.point_at();
        break;
      }
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kRelativePositionEquality:
        *resolved_constraints.add_constraints()
             ->mutable_relative_position_equality() =
            constraint.relative_position_equality();
        break;
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kRelativeRotationEquality:
        *resolved_constraints.add_constraints()
             ->mutable_relative_rotation_equality() =
            constraint.relative_rotation_equality();
        break;
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          ConstraintCase::kRelativeCartesianPose:
        *resolved_constraints.add_constraints()
             ->mutable_relative_cartesian_pose() =
            constraint.relative_cartesian_pose();
        break;
      case intrinsic_proto::motion_planning::v1::GeometricConstraint::
          CONSTRAINT_NOT_SET:
        break;
    }
  }
  // Only add constraint if input requires it
  if (contains_joint_limit_constraint) {
    intrinsic_proto::motion_planning::v1::JointPositionLimits
        resolved_joint_limits_proto = ToJointPositionLimitProto(
            robot, resolved_lower_limit, resolved_upper_limit);
    *resolved_constraints.add_constraints()->mutable_joint_position_limits() =
        resolved_joint_limits_proto;
  }
  intrinsic_proto::motion_planning::v1::GeometricConstraint
      resolved_constraints_result;
  *resolved_constraints_result.mutable_constraint_intersection() =
      resolved_constraints;
  return std::move(resolved_constraints_result);
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::CartesianMotionTarget&
        cartesian_motion_target,
    const ComputeIkOptions& options) {
  absl::string_view kMotionPlanningErrorSuffix = "ComputeIk";

  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::skills::CartesianMotionTarget entity_target,
      object_world::ToEntityBasedCartesianMotionTarget(cartesian_motion_target,
                                                       world));
  GeometricConstraint geometric_constraint =
      GetGeometricConstraint(cartesian_motion_target);
  INTR_ASSIGN_OR_RETURN(
      const std::vector<eigenmath::VectorXd> result,
      ComputeIk(world.GetEntityWorld(), robot.GetRobotEntityId(), entity_target,
                options),
      _.With([&geometric_constraint,
              &kMotionPlanningErrorSuffix](absl::Status status) {
        return AssignConstraintForIKError(status, geometric_constraint,
                                          kMotionPlanningErrorSuffix);
      }));
  return result;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::PoseEquality&
        cartesian_pose_constraint,
    const ComputeIkOptions& options) {
  const stats::ScopedSpan span("PathPlanningUtil/ComputeIkWithPoseEquality");
  INTR_ASSIGN_OR_RETURN(
      const AttachmentEntityId moving_frame_id,
      GetOriginEntityIdForTransformNodeReference(
          world, cartesian_pose_constraint.moving_frame(),
          "A problem occurred while retrieving the moving "
          "frame from the world:",
          "Make sure the moving frame is defined and exist in the world."));
  INTR_ASSIGN_OR_RETURN(
      const AttachmentEntityId target_frame_id,
      GetOriginEntityIdForTransformNodeReference(
          world, cartesian_pose_constraint.target_frame(),
          "A problem occurred while retrieving the target "
          "frame from the world:",
          "Make sure the target frame is defined and exist in the world."));

  Pose3d target_frame_offset = Pose3d::Identity();
  if (cartesian_pose_constraint.has_target_frame_offset()) {
    if (!cartesian_pose_constraint.target_frame_offset().has_orientation()) {
      return absl::InvalidArgumentError(
          "Target frame offset must have an explicitly set orientation.");
    }
    INTR_ASSIGN_OR_RETURN(
        target_frame_offset,
        FromProtoNormalized(cartesian_pose_constraint.target_frame_offset()));
  }

  INTR_ASSIGN_OR_RETURN(
      const bool target_is_tool_frame,
      IsTargetToolFrame(world, robot, target_frame_id, moving_frame_id));

  if (!target_is_tool_frame) {
    return ComputeIk(world.GetEntityWorld(), robot.GetRobotEntityId(),
                     moving_frame_id, target_frame_id,
                     target_frame_offset.inverse(), options);
  }
  return ComputeIk(world.GetEntityWorld(), robot.GetRobotEntityId(),
                   target_frame_id, moving_frame_id, target_frame_offset,
                   options);
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint,
    const ComputeIkOptions& options) {
  const stats::ScopedSpan span(
      "PathPlanningUtil/ComputeIkWithGeometricConstraint");
  absl::string_view kMotionPlanningErrorSuffix = "ComputeIk";

  // Check if constraint set is Cartesian Pose. Use user specified solver
  // instead.
  std::optional<intrinsic_proto::motion_planning::v1::PoseEquality>
      pose_equality = ParsePoseEqualityConstraint(world, geometric_constraint);
  if (pose_equality.has_value()) {
    INTR_ASSIGN_OR_RETURN(
        std::vector<eigenmath::VectorXd> result,
        ComputeIk(world, robot, pose_equality.value(), options),
        _.With([&geometric_constraint,
                &kMotionPlanningErrorSuffix](absl::Status status) {
          return AssignConstraintForIKError(status, geometric_constraint,
                                            kMotionPlanningErrorSuffix);
        }));
    return result;
  }

  // Get the skeleton for the robot_id (b/224589810). Required by NLOPT for
  // limits. To propagate the user defined joint limits we have to clone the
  // world.
  World planning_world = world.GetEntityWorld().Clone();
  INTR_ASSIGN_OR_RETURN(const LinkEntityId robot_base_id,
                        planning_world.GetBaseLink(robot.GetRobotEntityId()));
  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId robot_tip_id,
                        planning_world.GetFinalEntityOfRobotKinematicChain(
                            robot.GetRobotEntityId()));

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<DofKinematicView> dof_view,
      planning_world.GetDofKinematicView(robot.GetRobotEntityId()));
  if (dof_view == nullptr) {
    return absl::InternalError(
        absl::StrCat("Failed to get Cartesian view for robot: ",
                     robot.GetRobotEntityId().value()));
  }

  // Get initial seed as current configuration or user defined value.
  eigenmath::VectorNd initial_guess = dof_view->GetDofValues();
  if (options.seed_configuration.size() > 0) {
    if (options.seed_configuration.size() != initial_guess.size()) {
      return absl::InvalidArgumentError(
          "Provided seed configuration for computing the Ik is of different "
          "size than expected.");
    }
    initial_guess = options.seed_configuration;
  }

  // Set desired joint values in the world. This is important as we generate the
  // kinematic model from the world and those need to contain the correct limits
  // as well. If you require the current configuration of the robot, make sure
  // to retrieve this before the first time we call the SetApplicationLimit
  // function as this potentially changes the current config.
  if (options.joint_limits.has_value()) {
    INTR_RETURN_IF_ERROR(
        SetApplicationLimits(options.joint_limits.value(), dof_view.get()));
  }

  // Resolve the joint limits: Joint limits can be set by the user of ComputeIk
  // in which case they are handed down in the options. They effectively become
  // the new application limits. They also can be set through the target
  // constraints itself.
  // Due to NLOPT not doing well with inf values (b/276514783) and large
  // configuration spaces those values need to be further pruned.
  // 1) Get joint limits from the world. This contains the limits set through
  // the options.
  INTR_ASSIGN_OR_RETURN(JointLimits joint_limits,
                        ToJointLimits(dof_view->GetDofApplicationLimits()));

  // Check if constraint set contains joint position equality constraint and
  // handle separately.
  INTR_ASSIGN_OR_RETURN(std::optional<eigenmath::VectorNd> joint_config,
                        ParseJointPositionEqualityConstraints(
                            world, robot, joint_limits, geometric_constraint));
  if (joint_config.has_value()) {
    std::vector<eigenmath::VectorXd> solutions;
    solutions.push_back(*joint_config);
    return solutions;
  }

  // Check if same branch requirement was set. This is currently not supported
  // for this solver.
  if (options.ensure_same_branch) {
    return absl::InvalidArgumentError(
        "ComputeIk does currently not support computation with same branch "
        "ik for underconstrained targets.");
  }

  // 2) Prune limits due to existing bugs (see below) and set them in the world
  // such that the skeletons handed to the nlopt solver and constraints contain
  // the right limits. Those define the search space.
  if (robot_tip_id == kInvalidEntityId) {
    return absl::InvalidArgumentError(
        "Provided robot is not a kinematic chain. Planning with constraints "
        "is not supported yet for robots with multiple kinematic chains.");
  }
  INTR_ASSIGN_OR_RETURN(auto skeleton, planning_world.BuildChainSkeleton(
                                           robot_base_id, robot_tip_id));
  // TODO(b/276514783): Eliminate the source of the problem. Just work
  // around for quick fix.
  INTR_ASSIGN_OR_RETURN(joint_limits,
                        RestrictInfJointPositionValues(
                            skeleton.get(), joint_limits, initial_guess,
                            /*restriction_range=*/2 * M_PI));
  // Wrap joint limits to avoid excessive rotations and improve solution found.
  // TODO(b/286607620): Add function for wrapping joint limits.
  INTR_ASSIGN_OR_RETURN(joint_limits,
                        RestrictJointPositionValuesToMaxRange(
                            skeleton.get(), joint_limits, initial_guess,
                            /*max_range=*/2 * M_PI));

  // 3) Resolve joint limits constraints such that we only have one constraint
  // in the geometric constraints and that is the joint limit intersection of
  // all those limits (i.e., the most restricting set of joint limits). Those
  // define the valid solution. It is important to add the desired planning
  // limits here as the object world does not contain the correct limits at this
  // point. We cannot update the object_world due to the cost involved.
  INTR_ASSIGN_OR_RETURN(GeometricConstraint updated_geometric_constraints,
                        AddAndResolveJointPositionLimitsConstraints(
                            world, robot, joint_limits, geometric_constraint));

  // Update joint limits to use those defined in the
  // updated_geometric_constraint. Those should be the most restrictive ones.
  INTR_RETURN_IF_ERROR(UpdateJointPositionLimits(
      world, robot, updated_geometric_constraints, joint_limits));

  // Create kinematic model with updated limits to be passed to the nlopt
  // solver. Without this we might risk the solver not returning valid solution
  // or crash in case of inf limits or when we encounter large configuration
  // spaces.
  INTR_RETURN_IF_ERROR(SetApplicationLimits(JointLimitsXd::Create(joint_limits),
                                            dof_view.get()));
  // TODO(b/303742964): Check if initial guess is within limits to prevent
  // solution out of limit or crash.
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto limit_check_initial_guess,
                                IsWithinLimits(initial_guess, joint_limits));
  if (!limit_check_initial_guess.p_ok) {
    initial_guess =
        0.5 * (joint_limits.max_position + joint_limits.min_position);
    LOG(INFO) << "Resetting initial guess because it is out of joint limits "
                 "(b/303742964). Resetted to "
              << initial_guess.transpose();
  }

  INTR_ASSIGN_OR_RETURN(
      const auto skeleton_updated_limits,
      planning_world.BuildChainSkeleton(robot_base_id, robot_tip_id));
  // TODO(b/286312331): Make NLOPT IK solver more robust for GeometricConstraint
  // target resolution. Compute at least 8 IK solutions with the NLOPT solver
  // without any cost function` to increase the likelihood of finding a
  // variation of joint targets,
  constexpr int kMinSolutionSampled = 8;
  const int nlopt_max_ik_solutions_sampled =
      (options.max_num_solutions > kMinSolutionSampled)
          ? options.max_num_solutions
          : kMinSolutionSampled;
  INTR_ASSIGN_OR_RETURN(
      std::vector<std::unique_ptr<ConstraintInterface>> resolved_constraints,
      GetConstraintsFromProto(world, robot, updated_geometric_constraints));
  INTR_ASSIGN_OR_RETURN(
      std::vector<eigenmath::VectorXd> solutions,
      SampleSolutionsWithNlopt(skeleton_updated_limits.get(),
                               std::move(resolved_constraints), initial_guess,
                               /*set_joint_position_cost*/ false,
                               nlopt_max_ik_solutions_sampled),
      _.With([&geometric_constraint,
              &kMotionPlanningErrorSuffix](absl::Status status) {
        return AssignConstraintForIKError(status, geometric_constraint,
                                          kMotionPlanningErrorSuffix);
      }));  // Pass the original geometric constraint for the error.

  // Follow up with sampling an additional set of 8 solutions as close as
  // possible to the initial guess to increase the likelihood of finding on the
  // same branch or closer to the initial guess. Number was experimentally
  // determined in some experiments 2 was enough in others at least 6 to get a
  // good solution.
  INTR_ASSIGN_OR_RETURN(
      std::vector<std::unique_ptr<ConstraintInterface>>
          cost_resolved_constraints,
      GetConstraintsFromProto(world, robot, updated_geometric_constraints));
  const int nlopt_max_similar_solutions = 6;
  INTR_ASSIGN_OR_RETURN(
      std::vector<eigenmath::VectorXd> similar_solutions,
      SampleSolutionsWithNlopt(
          skeleton_updated_limits.get(), std::move(cost_resolved_constraints),
          initial_guess,
          /*set_joint_position_cost*/ true, nlopt_max_similar_solutions),
      _.With([&geometric_constraint,
              &kMotionPlanningErrorSuffix](absl::Status status) {
        return AssignConstraintForIKError(status, geometric_constraint,
                                          kMotionPlanningErrorSuffix);
      }));

  std::copy(similar_solutions.begin(), similar_solutions.end(),
            std::back_inserter(solutions));

  // Sort solutions with respect to similarity to the initial guess using
  // the L2 norm.
  std::sort(solutions.begin(), solutions.end(),
            [&initial_guess](const eigenmath::VectorNd& qa,
                             const eigenmath::VectorNd& qb) {
              return (qa - initial_guess).norm() < (qb - initial_guess).norm();
            });

  for (const auto& ik : solutions) {
    LOG(INFO) << "nlopt solution " << ik.transpose();
  }

  // Return the first max_ik_solutions_sampled as requested.
  if (solutions.size() > options.max_num_solutions) {
    solutions.resize(options.max_num_solutions);
  }
  return solutions;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> ComputeIk(
    const World& world, RobotCollectionsEntityId robot_id,
    AttachmentEntityId reference_id, AttachmentEntityId object_id,
    const Pose3d& reference_t_object, const ComputeIkOptions& options) {
  // Cloning the world to enable setting world limits.
  World planning_world = world.Clone();
  // TODO(b/259212669): Guarantee that a kinematic chain belongs to a robot
  // Check if the robot is a kinematic chain with a single final entity before
  // constructing a CartesianKinematicView.
  INTR_ASSIGN_OR_RETURN(bool is_kinematic_chain,
                        planning_world.IsChainKinematicChain(robot_id));
  if (!is_kinematic_chain) {
    return absl::InternalError(absl::StrCat(
        "Cannot compute IK for a robot with multiple final entities. Robot ",
        robot_id.value(),
        " is not a kinematic chain with a single final entity."));
  }

  INTR_ASSIGN_OR_RETURN(auto dof_view,
                        planning_world.GetDofKinematicView(robot_id));
  eigenmath::VectorXd seed_configuration = dof_view->GetDofValues();
  if (options.seed_configuration.size() > 0) {
    if (options.seed_configuration.size() != seed_configuration.size()) {
      return absl::InvalidArgumentError(
          "Provided seed configuration for computing the Ik is of different "
          "size than expected.");
    }
    seed_configuration = options.seed_configuration;
  }

  if (options.joint_limits.has_value()) {
    // Make sure the current configuration has been saved if not done yet. Set
    // Application limits can modify the current configuration in the world.
    INTR_RETURN_IF_ERROR(
        SetApplicationLimits(options.joint_limits.value(), dof_view.get()));
  }

  // TODO(b/303742964): Check if initial guess is within limits to prevent crash
  // of Ik solver.
  const JointLimitsXd joint_limits = dof_view->GetDofApplicationLimits();
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const auto limit_check_initial_guess,
      IsWithinLimits(seed_configuration, joint_limits));
  if (!limit_check_initial_guess.p_ok) {
    seed_configuration =
        0.5 * (joint_limits.max_position + joint_limits.min_position);
    LOG(INFO) << "Resetting initial guess because it is out of joint limits "
                 "(b/303742964). Resetted to "
              << seed_configuration.transpose();
  }

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<const CartesianKinematicView> cartesian_view,
      planning_world.GetCartesianKinematicView(robot_id));
  if (cartesian_view == nullptr) {
    return absl::InternalError(absl::StrCat(
        "Failed to get cartesian view for robot: ", robot_id.value()));
  }

  const PhysicalEntityId control_point_id =
      cartesian_view->ObjectsBeingControlled().second;

  const Pose3d control_root_t_reference = planning_world.GetTransform(
      cartesian_view->ObjectsBeingControlled().first, reference_id);
  const Pose3d object_t_control_tip =
      planning_world.GetTransform(object_id, control_point_id);

  const Pose3d control_root_t_control_tip =
      control_root_t_reference * reference_t_object * object_t_control_tip;

  // TODO(b/281545078): Replace with one Ik call with the option of preferred
  // same branch solution if possible.
  // Computes first the same branch solution and extends the solution set with
  // other solutions.
  std::vector<eigenmath::VectorXd> solutions;
  solutions.reserve(options.max_num_solutions);
  std::optional<eigenmath::VectorXd> same_branch_solution;
  if (options.prefer_same_branch || options.ensure_same_branch) {
    auto attempt_same_branch_solution =
        ComputeSameBranchIk(planning_world, *cartesian_view, reference_id,
                            object_id, reference_t_object, seed_configuration);
    if (attempt_same_branch_solution.ok()) {
      same_branch_solution = attempt_same_branch_solution.value();
      solutions.push_back(*same_branch_solution);
    } else {
      LOG(INFO) << "Same branch ik solution not available, error: "
                << attempt_same_branch_solution.status().message();
    }
    if (options.ensure_same_branch) {
      if (!attempt_same_branch_solution.ok()) {
        // Return failure.
        return attempt_same_branch_solution.status();
      }
      return solutions;
    }
  }

  // Compute all other solutions.
  if (options.max_num_solutions > 1 || solutions.empty()) {
    std::unique_ptr<IkSolutions> ik_solutions = cartesian_view->GetIkSolutions(
        control_root_t_control_tip, seed_configuration);
    const std::vector<eigenmath::VectorXd> additional_solutions =
        ik_solutions->GetSampledSolutions(options.max_num_solutions,
                                          std::nullopt);

    // Add solutions different from same branch solution if available.
    for (const eigenmath::VectorXd& sampled_solution : additional_solutions) {
      if (solutions.size() >= options.max_num_solutions) {
        break;
      }
      if (!same_branch_solution.has_value() ||
          !sampled_solution.isApprox(*same_branch_solution)) {
        solutions.push_back(sampled_solution);
      }
    }
  }
  return solutions;
}

absl::StatusOr<eigenmath::VectorXd> ComputeSameBranchIk(
    const object_world::ObjectWorld& world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::CartesianMotionTarget&
        cartesian_motion_target) {
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::skills::CartesianMotionTarget entity_target,
      object_world::ToEntityBasedCartesianMotionTarget(cartesian_motion_target,
                                                       world));
  return ComputeSameBranchIk(world.GetEntityWorld(), robot.GetRobotEntityId(),
                             entity_target);
}

absl::StatusOr<eigenmath::VectorXd> ComputeSameBranchIk(
    const World& world, RobotCollectionsEntityId robot_id,
    AttachmentEntityId reference_id, AttachmentEntityId object_id,
    const Pose3d& reference_t_object) {
  // TODO(b/259212669): Guarantee that a kinematic chain belongs to a robot
  // Check if the robot is a kinematic chain with a single final entity before
  // constructing a CartesianKinematicView.
  INTR_ASSIGN_OR_RETURN(bool is_kinematic_chain,
                        world.IsChainKinematicChain(robot_id));
  if (!is_kinematic_chain) {
    return absl::InternalError(absl::StrCat(
        "Cannot compute IK for a robot with multiple final entities. Robot ",
        robot_id.value(),
        " is not a kinematic chain with a single final entity."));
  }

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<const CartesianKinematicView> cartesian_view,
      world.GetCartesianKinematicView(robot_id));
  if (cartesian_view == nullptr) {
    return absl::InternalError(absl::StrCat(
        "Failed to get cartesian view for robot: ", robot_id.value()));
  }

  return ComputeSameBranchIk(world, *cartesian_view, reference_id, object_id,
                             reference_t_object);
}

absl::StatusOr<eigenmath::VectorXd> ComputeSameBranchIk(
    const World& world, const CartesianKinematicView& cartesian_view,
    AttachmentEntityId reference_id, AttachmentEntityId object_id,
    const Pose3d& reference_t_object,
    std::optional<eigenmath::VectorXd> seed_configuration) {
  absl::string_view kMotionPlanningErrorSuffix = "ComputeSameBranchIk";

  const PhysicalEntityId control_point_id =
      cartesian_view.ObjectsBeingControlled().second;

  const Pose3d control_root_t_reference = world.GetTransform(
      cartesian_view.ObjectsBeingControlled().first, reference_id);
  const Pose3d object_t_control_tip =
      world.GetTransform(object_id, control_point_id);

  const Pose3d control_root_t_control_tip =
      control_root_t_reference * reference_t_object * object_t_control_tip;

  eigenmath::VectorXd current_configuration =
      cartesian_view.GetDofView()->GetDofValues();
  if (seed_configuration.has_value()) {
    current_configuration = seed_configuration.value();
  }
  std::unique_ptr<IkSolutions> ik_solutions = cartesian_view.GetIkSolutions(
      control_root_t_control_tip, current_configuration);
  std::optional<eigenmath::VectorXd> solution =
      ik_solutions->GetSameBranchSolution();
  if (!solution.has_value()) {
    const std::string error_message =
        "IK solver returned no solutions. "
        "It seems like the target pose is not reachable or there is no IK "
        "solution on the same branch as the current state of the robot arm.";
    return CreateStatusWithIKError(error_message, kMotionPlanningErrorSuffix,
                                   absl::StatusCode::kInternal,
                                   intrinsic_proto::motion_planning::v1::
                                       ErrorContext::IK_NO_SOLUTIONS_FOUND);
  }

  return solution.value();
}

absl::StatusOr<eigenmath::VectorXd> GetClosestIkSolution(
    const World& world, const KinematicsSystemProxy& proxy,
    RobotCollectionsEntityId robot_id, AttachmentEntityId reference_id,
    AttachmentEntityId object_id, const Pose3d& reference_t_object) {
  absl::string_view kMotionPlanningErrorSuffix = "GetClosestIkSolution";

  INTR_ASSIGN_OR_RETURN(
      std::vector<eigenmath::VectorXd> ik_solutions,
      ComputeIk(world, robot_id, reference_id, object_id, reference_t_object));
  if (ik_solutions.empty()) {
    const std::string error_message =
        "IK solver couldn't find any solutions for the given constraints. "
        "Typically this means the Cartesian constraint region was"
        " either outside the robot's reachable envelope or that"
        " it led to limit violations.";
    return CreateStatusWithIKError(error_message, kMotionPlanningErrorSuffix,
                                   absl::StatusCode::kNotFound,
                                   intrinsic_proto::motion_planning::v1::
                                       ErrorContext::IK_NO_SOLUTIONS_FOUND);
  }

  // The solution are already ordered so that the first one is the closest to
  // the current configuration.
  std::vector<CollisionCheckingDebug> collision_debugs(ik_solutions.size());
  for (int i = 0; i < ik_solutions.size(); ++i) {
    const eigenmath::VectorXd& ik_solution = ik_solutions[i];
    VLOG(1) << "Evaluating ik solution " << ik_solution.transpose();
    INTR_ASSIGN_OR_RETURN(JointConfigurationValidationResult
                              joint_configuration_validation_result,
                          proxy.IsValid(ik_solution, &collision_debugs[i]));
    if ((joint_configuration_validation_result.collision_status ==
         MarginPairConflictStatus::kClear) &&
        (joint_configuration_validation_result.within_limits_status ==
         WithinLimitsStatus::kWithinLimits) &&
        (joint_configuration_validation_result.constraint_satisfaction_status ==
         ConstraintSatisfactionStatus::kAllConstraintsSatisfied)) {
      return ik_solution;
    }
  }

  // Build collision report for debugging purposes.
  std::string collision_report;
  intrinsic_proto::motion_planning::v1::CollisionError collision_error;
  // We limit the collision report to show 4 configurations maximum.
  int kMessageClipLimit = std::min<int>(ik_solutions.size(), 4);
  for (int i = 0; i < kMessageClipLimit; ++i) {
    const eigenmath::VectorXd& ik_solution = ik_solutions[i];
    const CollisionCheckingDebug& collision_debug = collision_debugs[i];
    INTR_ASSIGN_OR_RETURN(std::string collision_debug_string,
                          proxy.PrintCollisionCheckingDebug(collision_debug));
    std::string message =
        absl::StrFormat("\n\tConfig %d: [%s] in radians: %s", i,
                        toString(ik_solution), collision_debug_string);
    absl::StrAppend(&collision_report, message);
    INTR_ASSIGN_OR_RETURN(
        collision_error,
        CreateCollisionError(
            message, collision_debug, ik_solution, proxy,
            intrinsic_proto::motion_planning::v1::ErrorContext::IK_COLLISION,
            collision_error));
  }

  std::string error_message;
  if (collision_report.empty()) {
    return absl::NotFoundError(
        "IK solver couldn't find any solutions for the given constraints. "
        "Typically this means the Cartesian constraint region was"
        " either outside the robot's reachable envelope or that"
        " it led to limit violations.");
  } else {
    error_message = absl::StrFormat(
        "\nIk could not find a collision free configuration.\n"
        "See below for more information about the collisions detected"
        " for the solutions found.\n"
        "\nReport:\n %s",
        collision_report);
    absl::Status status =
        absl::NotFoundError(ClipTooLongErrorMessage(error_message));
    intrinsic_proto::motion_planning::v1::MotionPlanningError
        motion_planning_error;
    *motion_planning_error.mutable_collision_error() = collision_error;
    status.SetPayload(absl::StrCat(kMotionPlanningCollisionErrorPrefix,
                                   kMotionPlanningErrorSuffix),
                      absl::Cord(motion_planning_error.SerializeAsString()));
    return status;
  }
}

absl::StatusOr<eigenmath::VectorXd> GetSameBranchIkSolution(
    const object_world::ObjectWorld& world, const KinematicsSystemProxy& proxy,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::CartesianMotionTarget&
        cartesian_motion_target) {
  absl::string_view kMotionPlanningErrorSuffix = "GetSameBranchIkSolution";

  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::skills::CartesianMotionTarget entity_target,
      object_world::ToEntityBasedCartesianMotionTarget(cartesian_motion_target,
                                                       world));
  const GeometricConstraint geometric_constraint =
      GetGeometricConstraint(cartesian_motion_target);
  INTR_ASSIGN_OR_RETURN(
      eigenmath::VectorXd result,
      GetSameBranchIkSolution(world.GetEntityWorld(), proxy,
                              robot.GetRobotEntityId(), entity_target),
      _.With([&geometric_constraint,
              &kMotionPlanningErrorSuffix](absl::Status status) {
        return AssignConstraintForIKError(status, geometric_constraint,
                                          kMotionPlanningErrorSuffix);
      }));
  return result;
}

absl::StatusOr<eigenmath::VectorXd> GetSameBranchIkSolution(
    const World& world, const KinematicsSystemProxy& proxy,
    RobotCollectionsEntityId robot_id, AttachmentEntityId reference_id,
    AttachmentEntityId object_id, const Pose3d& reference_t_object) {
  absl::string_view kMotionPlanningErrorSuffix = "GetSameBranchIkSolution";

  INTR_ASSIGN_OR_RETURN(auto ik_solution,
                        ComputeSameBranchIk(world, robot_id, reference_id,
                                            object_id, reference_t_object));

  CollisionCheckingDebug collision_debug;
  VLOG(1) << "Evaluating ik solution " << ik_solution.transpose();
  INTR_ASSIGN_OR_RETURN(
      JointConfigurationValidationResult joint_configuration_validation_result,
      proxy.IsValid(ik_solution, &collision_debug));
  if (static_cast<bool>(joint_configuration_validation_result)) {
    return ik_solution;
  }

  // Build collision report for debugging purposes.
  INTR_ASSIGN_OR_RETURN(std::string collision_debug_string,
                        proxy.PrintCollisionCheckingDebug(collision_debug));
  std::string collision_report =
      absl::StrFormat("\nSame branch config: [%s] in radians: %s",
                      toString(ik_solution), collision_debug_string);

  std::string error_message;
  if (collision_report.empty()) {
    error_message =
        "IK solver couldn't find any solutions for the given constraints. "
        "Typically this means the Cartesian constraint region was"
        " either outside the robot's reachable envelope or that"
        " it led to limit violations.";
    return CreateStatusWithIKError(error_message, kMotionPlanningErrorSuffix,
                                   absl::StatusCode::kNotFound,
                                   intrinsic_proto::motion_planning::v1::
                                       ErrorContext::IK_NO_SOLUTIONS_FOUND);
  } else {
    error_message = absl::StrFormat(
        "\nIK could not find a collision free configuration.\n"
        "See below for more information about the collisions detected"
        " for the solutions found."
        "\nReport:\n %s",
        collision_report);
    return CreateStatusWithCollisionError(
        error_message, collision_debug, ik_solution, proxy,
        kMotionPlanningErrorSuffix, absl::StatusCode::kNotFound,
        intrinsic_proto::motion_planning::v1::ErrorContext::IK_COLLISION);
  }
}

bool IsNotReachable(const absl::Status& status) {
  return (absl::IsInternal(status) || absl::IsNotFound(status)) &&
         (absl::StrContains(status.message(), "not reachable") ||
          absl::StrContains(status.message(),
                            "outside the robot's reachable envelope"));
}

bool IsInCollision(const absl::Status& status) {
  return (absl::IsInternal(status) || absl::IsInvalidArgument(status) ||
          absl::IsNotFound(status)) &&
         absl::StrContains(status.message(), "collision");
}

absl::Status ValidateCollisionSettings(
    const intrinsic_proto::world::CollisionSettings& collision_settings) {
  if (collision_settings.disable_collision_checking() &&
      collision_settings.has_minimum_margin()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Collision checking set as disabled, but minimum "
                        "margin specified (as %.5f).",
                        collision_settings.minimum_margin()));
  }
  if (collision_settings.minimum_margin() < 0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Minimum margin has to be non-negative but is: %.5f ",
                        collision_settings.minimum_margin()));
  }
  return absl::OkStatus();
}

absl::StatusOr<Pose3d> CartesianMotionTargetToPose3d(
    const world::ObjectWorldClient& object_world,
    const intrinsic_proto::motion_planning::CartesianMotionTarget&
        motion_target,
    world::TransformNode reference_node, world::TransformNode target_node) {
  INTR_ASSIGN_OR_RETURN(const world::TransformNode tool_node,
                        object_world.GetTransformNode(motion_target.tool()));
  INTR_ASSIGN_OR_RETURN(const world::TransformNode frame_node,
                        object_world.GetTransformNode(motion_target.frame()));

  INTR_ASSIGN_OR_RETURN(const Pose3d reference_t_frame,
                        object_world.GetTransform(reference_node, frame_node));
  INTR_ASSIGN_OR_RETURN(const Pose3d tool_t_target,
                        object_world.GetTransform(tool_node, target_node));

  // Extract pose properties for constraint conversion into kinematics
  // setting.
  Pose3d frame_t_tool_target = Pose3d::Identity();
  if (motion_target.has_offset()) {
    INTR_ASSIGN_OR_RETURN(
        frame_t_tool_target,
        intrinsic_proto::FromProtoNormalized(motion_target.offset()));
  }

  return reference_t_frame * frame_t_tool_target * tool_t_target;
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> GetCollisionFreeIkSolutions(
    const object_world::ObjectWorld& world, const KinematicsSystemProxy& proxy,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        geometric_constraint,
    const ComputeIkOptions& options,
    ComputeIkDebugInformation* debug_information) {
  const stats::ScopedSpan span("PathPlanningUtil/GetCollisionFreeIkSolutions");
  absl::string_view kMotionPlanningErrorSuffix = "GetCollisionFreeIkSolutions";

  // Approximation: Sample more than requested to allow to find at least one
  // collision free solution. Differentiate between computing Pose3d and less
  // constraint motion targets.
  std::optional<intrinsic_proto::motion_planning::v1::PoseEquality>
      pose_equality = ParsePoseEqualityConstraint(world, geometric_constraint);
  std::vector<eigenmath::VectorXd> ik_solutions;
  ComputeIkOptions updated_options = options;
  if (pose_equality.has_value()) {
    // Invoke analytical or iterative solver.
    // In case of a Pose3d motion target we use the user specified analytical or
    // iterative ik solver that does not require to sample that much more to get
    // at least one collision free solution. While analytical solver are quite
    // fast and we could always request a higher number, in case of an iterative
    // solver this can quickly lead to long computation times.
    const int kMaxIkSolutionSampled = std::max(options.max_num_solutions, 8);
    updated_options.max_num_solutions = kMaxIkSolutionSampled;
    INTR_ASSIGN_OR_RETURN(
        ik_solutions,
        ComputeIk(world, robot, pose_equality.value(), updated_options),
        _.LogError());
  } else {
    // Approximation: For many constraints the number of samples in collision is
    // quite high. Sample more than requested to allow to find  at least one
    // collision free solution in case of an under constraint problem. Sampling
    // is relatively fast.
    const int kMaxIkSolutionSampled =
        std::max(2 * options.max_num_solutions, 12);
    updated_options.max_num_solutions = kMaxIkSolutionSampled;
    // Compute all configurations that satisfy the constraints.
    INTR_ASSIGN_OR_RETURN(
        ik_solutions,
        ComputeIk(world, robot, geometric_constraint, updated_options),
        _.LogError());
  }

  if (ik_solutions.empty()) {
    const std::string error_message =
        "IK solver couldn't find any solutions for the given constraints. "
        "Typically this means the Cartesian constraint region was"
        " either outside the robot's reachable envelope or that"
        " it led to limit violations.";
    return CreateStatusWithIKError(error_message, kMotionPlanningErrorSuffix,
                                   absl::StatusCode::kNotFound,
                                   intrinsic_proto::motion_planning::v1::
                                       ErrorContext::IK_NO_SOLUTIONS_FOUND,
                                   geometric_constraint);
  }

  std::vector<eigenmath::VectorXd> valid_ik_configurations;
  {
    const stats::ScopedSpan span_validation(
        "PathPlanningUtil/GetCollisionFreeIkSolutions/Validation");
    // The solutions are already ordered so that the first one is the closest
    // to the current configuration.
    std::string collision_report;
    const int kMessageClipLimit = 4;
    int num_collision_configs = 0;
    intrinsic_proto::motion_planning::v1::CollisionError collision_error;
    for (int i = 0; i < ik_solutions.size(); ++i) {
      const eigenmath::VectorXd& ik_solution = ik_solutions[i];
      CollisionCheckingDebug collision_debugs;
      INTR_ASSIGN_OR_RETURN(JointConfigurationValidationResult
                                joint_configuration_validation_result,
                            proxy.IsValid(ik_solution, &collision_debugs));
      ComputeIkDebugInformation::IkSolution solution_debug_info;
      solution_debug_info.configuration = ik_solutions[i];
      solution_debug_info.validation_result =
          joint_configuration_validation_result;
      if (!static_cast<bool>(joint_configuration_validation_result)) {
        // Only report solutions that have a valid collision in
        // collision_debugs. The reason we need to check this is the NLOpt
        // solver might produce solutions outside the joint limits if the
        // constraints to the solver aren't set correctly.
        if (!collision_debugs.collisions.empty()) {
          // Populate the Collision debug if available;
          solution_debug_info.collision_checking_debug.emplace(
              collision_debugs);

          // Populate error message
          INTR_ASSIGN_OR_RETURN(
              std::string collision_debug_string,
              proxy.PrintCollisionCheckingDebug(collision_debugs));
          INTR_ASSIGN_OR_RETURN(
              auto collision_debug_proto,
              proxy.GetCollisionDebugMessage(collision_debugs));
          *collision_debug_proto.mutable_joint_positions() =
              icon::ToJointVecProto(ik_solution);
          if (num_collision_configs < kMessageClipLimit) {
            // We only append 4 configurations to the collision report.
            const std::string message =
                absl::StrFormat("\nConfig %d: [%s] in radians: %s", i,
                                toString(ik_solution), collision_debug_string);
            absl::StrAppend(&collision_report, message);
            INTR_ASSIGN_OR_RETURN(
                collision_error,
                CreateCollisionError(message, collision_debugs, ik_solution,
                                     proxy,
                                     intrinsic_proto::motion_planning::v1::
                                         ErrorContext::IK_COLLISION,
                                     collision_error));
            num_collision_configs++;
          }
        }
        if (debug_information != nullptr) {
          debug_information->ik_solutions.push_back(
              std::move(solution_debug_info));
        }
        continue;
      }
      // Need to do an additional limit check here because the proxy might not
      // contain the correct joint limits set in the options.
      if (options.joint_limits.has_value()) {
        INTRINSIC_RT_ASSIGN_OR_RETURN(
            LimitCheckResult limit_result,
            IsWithinLimits(ik_solution, options.joint_limits.value()));
        if (!limit_result.p_ok) {
          solution_debug_info.validation_result.within_limits_status =
              WithinLimitsStatus::kViolatedLimits;
          if (debug_information != nullptr) {
            debug_information->ik_solutions.push_back(
                std::move(solution_debug_info));
          }
          continue;
        }
      }
      valid_ik_configurations.push_back(ik_solution);
      if (debug_information != nullptr) {
        debug_information->ik_solutions.push_back(
            std::move(solution_debug_info));
      }
      if (valid_ik_configurations.size() >= options.max_num_solutions) {
        break;
      }
    }

    if (valid_ik_configurations.empty()) {
      std::string error_message;
      if (collision_report.empty()) {
        error_message =
            "IK solver couldn't find any solutions for the given constraints. "
            "Typically this means the Cartesian constraint region was"
            " either outside the robot's reachable envelope or that"
            " it led to limit violations.";
        return CreateStatusWithIKError(error_message,
                                       kMotionPlanningErrorSuffix,
                                       absl::StatusCode::kNotFound,
                                       intrinsic_proto::motion_planning::v1::
                                           ErrorContext::IK_NO_SOLUTIONS_FOUND,
                                       geometric_constraint);

      } else {
        if (!options.disable_error_on_collisions) {
          error_message = absl::StrFormat(
              "\nIK could not find a collision free configuration.\n"
              "See below for more information about the collisions detected"
              " for the solutions found.\n"
              "\nReport:\n %s",
              collision_report);
          absl::Status status = CreateStatusWithCollisionError(
              error_message, kMotionPlanningErrorSuffix, collision_error,
              absl::StatusCode::kNotFound);
          return status;
        }
      }
    }
  }
  return valid_ik_configurations;
}

absl::StatusOr<std::vector<eigenmath::VectorNd>> ComputeSingularityRobustIK(
    const icon::ManipulatorKinematics& kinematics,
    const eigenmath::VectorNd& hint_joint_configuration,
    const Pose3d& base_t_target, const Pose3d& tip_t_target,
    const JointLimits& joint_limits,
    const std::optional<eigenmath::VectorNd> closest_hint_joint_configuration,
    kinematics::KinematicChainRandomSeedIKSolver* singularity_robust_ik_solver,
    bool ensure_same_branch) {
  const Pose3d base_t_tip_desired = base_t_target * tip_t_target.inverse();

  // A vector used to store all valid ik solutions found (two solutions might
  // be computed in the proximity of kinematic singularities).
  std::vector<eigenmath::VectorNd> ik_solutions;
  ik_solutions.reserve(2);

  IKResult ik_result;
  JointStateP ik_solution;
  INTR_ASSIGN_OR_RETURN(
      ik_result, kinematics.GetInverseKinematicsSolver().ComputeIK(
                     hint_joint_configuration, base_t_tip_desired, joint_limits,
                     ensure_same_branch, &ik_solution));

  if (ik_result.status != IKResult::OK) {
    return absl::InternalError(
        absl::StrCat("Could not compute the IK for Cartesian path sample "
                     "with transform base_t_target=",
                     toString(base_t_target), "."));
  }
  ik_solutions.push_back(ik_solution.position);

  // Check whether the current solution is in the proximity of a kinematic
  // singularity and, if so, recompute the solution using the singularity
  // robust solver.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double min_singular_value,
      GetJacobianSmallestSingularValue(&kinematics, ik_solutions.front()));

  // Used as a threshold on the minimum singular value of the robot Jacobian to
  // detect proximity to kinematic singularities. The threshold was tuned on
  // different supported robots (ur5, mh12, mh24) to be rather conservative.
  // Analytical ik solver should all succeed for minimum singular values greater
  // than this threshold.
  constexpr double kSingularityThreshold = 0.005;

  if (singularity_robust_ik_solver &&
      min_singular_value < kSingularityThreshold) {
    singularity_robust_ik_solver->setSeed(hint_joint_configuration);

    // Joint wrapping and checking of joint limits is already performed by the
    // solver, so no special IKValidator is needed.
    const std::vector<eigenmath::VectorXd> singularity_robust_ik_solutions =
        singularity_robust_ik_solver->compute(base_t_tip_desired,
                                              kinematics::IKValidator());
    if (!singularity_robust_ik_solutions.empty()) {
      ik_solutions.push_back(singularity_robust_ik_solutions.front());

      // As a further measure of robustness, compare and rank the ik_solutions
      // based on the L2 norm distance to either the
      // `closest_hint_joint_configuration` (if provided, e.g. the IK solution
      // of an adjacent pose in a path of poses) or the
      // `hint_joint_configuration`.
      const eigenmath::VectorNd& nearby_q =
          closest_hint_joint_configuration.has_value()
              ? closest_hint_joint_configuration.value()
              : hint_joint_configuration;
      std::sort(ik_solutions.begin(), ik_solutions.end(),
                [nearby_q](const eigenmath::VectorNd& solution_a,
                           const eigenmath::VectorNd& solution_b) {
                  return (solution_a - nearby_q).squaredNorm() <
                         (solution_b - nearby_q).squaredNorm();
                });
    }
  }
  return ik_solutions;
}

}  // namespace intrinsic
