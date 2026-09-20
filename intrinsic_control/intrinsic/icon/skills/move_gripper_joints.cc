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

#include "intrinsic/icon/skills/move_gripper_joints.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/icon/equipment/equipment_utils.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/proto/joint_trajectory_conversion.h"
#include "intrinsic/icon/skills/move_gripper_joints.pb.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/motion_planner_client.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.pb.h"
#include "intrinsic/motion_planning/service/motion_planner_service_asset_utils.h"
#include "intrinsic/motion_planning/trajectory_execution_utils.h"
#include "intrinsic/motion_planning/trajectory_planning/path_refinement/spline_based_path_refinement.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/refine_path_and_compute_acceleration_limited_trajectory.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_options.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_trajectory_result.h"
#include "intrinsic/skills/cc/equipment_pack.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/cc/skill_utils.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/skills/proto/skills.pb.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/get_extended_status.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/proto/collision_settings.pb.h"

namespace intrinsic::skills {

namespace {

using ::intrinsic::motion_planning::MotionPlannerClient;
using ::intrinsic_proto::skills::MoveGripperJointsParams;
using ::intrinsic_proto::world::CollisionSettings;

constexpr absl::string_view kDefaultGripperPartName = "gripper";
constexpr double kJointPositionTolerance = 1.0e-3;

constexpr double kDefaultSettlingTimeoutSeconds = 2.0;
constexpr double kPathRefinerJointBlendingDeviationRad = 0.01;

// Wraps a non-OK status with a fallback extended status code if it does not
// already have one.
//
// This is used to ensure all unexpected errors from lower-level dependencies
// are reported to the skill host with a valid extended status code specified in
// the skill's manifest (i.e. `kICONErrorCode`), rather than raw canonical
// status codes which are not declared.
//
// Skipping is done if:
// - The status is OK.
// - The status represents cancellation (which is an expected flow).
// - The status already has an extended status code attached (indicating a more
// specific error was already handled and constructed).
absl::Status WrapError(const absl::Status& status) {
  if (status.ok() || status.code() == absl::StatusCode::kCancelled ||
      GetExtendedStatus(status).has_value()) {
    return status;
  }
  StatusBuilder s(status);
  s.SetExtendedStatusCode(kExtendedStatusComponent, kICONErrorCode);
  s.SetExtendedStatusDebugMessage(status.message());
  return s;
}

// Overload of WrapError to support StatusOr types.
template <typename T>
absl::StatusOr<T> WrapError(absl::StatusOr<T> status_or) {
  if (status_or.ok()) {
    return status_or;
  }
  return WrapError(status_or.status());
}

absl::Status ValidateParams(const MoveGripperJointsParams& params,
                            const world::KinematicObject& gripper_object) {
  if (params.goal_position().value_size() !=
      gripper_object.JointPositions().size()) {
    std::string error_message =
        absl::StrCat("Wrong number of values in goal_position. Expected: ",
                     gripper_object.JointPositions().size(),
                     " actual: ", params.goal_position().value_size(), ".");
    StatusBuilder s(absl::InvalidArgumentError(error_message));
    s.SetExtendedStatusCode(kExtendedStatusComponent,
                            kInvalidParametersErrorCode);
    s.SetExtendedStatusDebugMessage(error_message);
    return s;
  }

  const eigenmath::VectorXd goal_position =
      RepeatedDoubleToVectorXd(params.goal_position().value());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LimitCheckResult is_within_limits,
      IsWithinLimits<JointLimitsXd>(goal_position,
                                    gripper_object.JointApplicationLimits()));

  if (!is_within_limits.p_ok) {
    std::string error_message = absl::StrCat(
        "The goal position [", toString(goal_position), "] must be between [",
        toString(gripper_object.JointApplicationLimits().min_position),
        "] and [",
        toString(gripper_object.JointApplicationLimits().max_position),
        "] for all joints.");
    StatusBuilder s(absl::InvalidArgumentError(error_message));
    s.SetExtendedStatusCode(kExtendedStatusComponent,
                            kInvalidParametersErrorCode);
    s.SetExtendedStatusDebugMessage(error_message);
    return s;
  }
  return absl::OkStatus();
}

absl::StatusOr<eigenmath::VectorXd> CheckForCollisionsAndLimits(
    const MoveGripperJointsParams& params,
    const world::KinematicObject& gripper_object,
    MotionPlannerClient& motion_planner) {
  // Default to use the application global collision settings.
  CollisionSettings collision_settings;
  if (params.has_collision_settings()) {
    collision_settings = params.collision_settings();
  }

  // Check if the initial position is within limits.
  // Due to numerical imprecision, we allow the initial position to be within
  // kJointPositionTolerance of the limits. But since ICON does not allow
  // joint positions that are outside the limits, we clip the initial
  // position to the limits.
  eigenmath::VectorXd initial_position = gripper_object.JointPositions();

  const eigenmath::VectorXd& joint_limit_lower =
      gripper_object.JointApplicationLimits().min_position;
  const eigenmath::VectorXd& joint_limit_upper =
      gripper_object.JointApplicationLimits().max_position;

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const LimitCheckResult is_within_limits,
      IsWithinLimits<JointLimitsXd>(initial_position,
                                    gripper_object.JointApplicationLimits(),
                                    kJointPositionTolerance));

  if (!is_within_limits.p_ok) {
    const std::string error_message = absl::StrCat(
        "The initial position [", toString(initial_position),
        "] must be between or close to [", toString(joint_limit_lower),
        "] and [", toString(joint_limit_upper), "] for all joints.");
    StatusBuilder s(absl::InternalError("Initial position is out of limits"));
    s.SetExtendedStatusCode(kExtendedStatusComponent, kOutOfLimitsErrorCode);
    s.SetExtendedStatusDebugMessage(error_message);
    return s;
  }

  LOG(INFO) << "Clipping initial position " << toString(initial_position)
            << " to limits " << toString(joint_limit_lower) << " and "
            << toString(joint_limit_upper) << " for all joints.";
  const eigenmath::VectorXd clamped_initial_position =
      initial_position.cwiseMax(joint_limit_lower).cwiseMin(joint_limit_upper);

  std::vector<eigenmath::VectorXd> waypoints;
  waypoints.reserve(2);
  waypoints.push_back(clamped_initial_position);
  waypoints.push_back(RepeatedDoubleToVectorXd(params.goal_position().value()));

  INTR_ASSIGN_OR_RETURN(
      const intrinsic_proto::motion_planning::v1::CheckCollisionsResponse
          response,
      motion_planner.CheckCollisions(
          gripper_object, waypoints,
          MotionPlannerClient::CheckCollisionsOptions{.collision_settings =
                                                          collision_settings}));

  if (response.has_collision()) {
    StatusBuilder s(absl::InternalError("Collision detected"));
    s.SetExtendedStatusCode(kExtendedStatusComponent, kCollisionErrorCode);
    s.SetExtendedStatusDebugMessage(response.collision_debug_msg());
    return s;
  }
  // Return the updated initial position.
  return clamped_initial_position;
}

}  // namespace

std::unique_ptr<SkillInterface> MoveGripperJoints::CreateSkill() {
  return std::unique_ptr<MoveGripperJoints>(
      new MoveGripperJoints(std::make_unique<icon::DefaultChannelFactory>()));
}

std::unique_ptr<SkillInterface> MoveGripperJoints::Create(
    std::unique_ptr<icon::ChannelFactory> icon_channel_factory) {
  return std::unique_ptr<MoveGripperJoints>(
      new MoveGripperJoints(std::move(icon_channel_factory)));
}

MoveGripperJoints::MoveGripperJoints(
    std::unique_ptr<icon::ChannelFactory> icon_channel_factory)
    : icon_channel_factory_(std::move(icon_channel_factory)) {}

absl::StatusOr<intrinsic_proto::skills::Footprint>
MoveGripperJoints::GetFootprint(const GetFootprintRequest& request,
                                GetFootprintContext& context) const {
  INTR_ASSIGN_OR_RETURN(
      const intrinsic_proto::skills::MoveGripperJointsParams params,
      request.params<intrinsic_proto::skills::MoveGripperJointsParams>());

  INTR_ASSIGN_OR_RETURN(
      const world::KinematicObject gripper_object,
      context.object_world().GetKinematicObject(params.gripper()));

  return ::intrinsic::skills::CreateObjectReservationFootprint(
      gripper_object.Name().value(),
      intrinsic_proto::skills::ObjectWorldReservation::WRITE);
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
MoveGripperJoints::Preview(const PreviewRequest& request,
                           PreviewContext& context) {
  return WrapError(PreviewInternal(request, context));
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
MoveGripperJoints::PreviewInternal(const PreviewRequest& request,
                                   PreviewContext& context) {
  INTR_ASSIGN_OR_RETURN(
      const intrinsic_proto::skills::MoveGripperJointsParams params,
      request.params<intrinsic_proto::skills::MoveGripperJointsParams>());

  INTR_ASSIGN_OR_RETURN(
      const world::KinematicObject gripper_object,
      context.object_world().GetKinematicObject(params.gripper()));

  INTR_RETURN_IF_ERROR(ValidateParams(params, gripper_object));
  auto& world = context.object_world();
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<motion_planning::MotionPlannerClient>
          motion_planner_service_asset_client,
      GetMotionPlannerServiceAssetClient(world.GetWorldID(),
                                         params.motion_planner_service()));
  // TODO(b/524634328): Remove the Fallback Logic from the Skills.
  motion_planning::MotionPlannerClient& motion_planner =
      (motion_planner_service_asset_client != nullptr)
          ? *motion_planner_service_asset_client
          : context.motion_planner();

  INTR_ASSIGN_OR_RETURN(
      const eigenmath::VectorXd initial_position,
      CheckForCollisionsAndLimits(params, gripper_object, motion_planner));

  // Update the world with the goal position.
  INTR_RETURN_IF_ERROR(context.object_world().UpdateJointPositions(
      gripper_object,
      RepeatedDoubleToVectorXd(params.goal_position().value())));

  return nullptr;
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
MoveGripperJoints::Execute(const ExecuteRequest& request,
                           ExecuteContext& context) {
  return WrapError(ExecuteInternal(request, context));
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
MoveGripperJoints::ExecuteInternal(const ExecuteRequest& request,
                                   ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(
      const intrinsic_proto::skills::MoveGripperJointsParams params,
      request.params<intrinsic_proto::skills::MoveGripperJointsParams>());

  INTR_ASSIGN_OR_RETURN(
      const world::KinematicObject gripper_object,
      context.object_world().GetKinematicObject(params.gripper()));

  INTR_RETURN_IF_ERROR(ValidateParams(params, gripper_object));
  auto& world = context.object_world();
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<motion_planning::MotionPlannerClient>
          motion_planner_service_asset_client,
      GetMotionPlannerServiceAssetClient(world.GetWorldID(),
                                         params.motion_planner_service()));
  // TODO(b/524634328): Remove the Fallback Logic from the Skills.
  motion_planning::MotionPlannerClient& motion_planner =
      (motion_planner_service_asset_client != nullptr)
          ? *motion_planner_service_asset_client
          : context.motion_planner();

  INTR_ASSIGN_OR_RETURN(
      const eigenmath::VectorXd initial_position,
      CheckForCollisionsAndLimits(params, gripper_object, motion_planner));

  // Check if the initial position is close to the goal position.
  const eigenmath::VectorXd goal_position =
      RepeatedDoubleToVectorXd(params.goal_position().value());
  if ((initial_position - goal_position).norm() < kJointPositionTolerance) {
    LOG(INFO) << "Initial position is close to the goal position. No motion "
                 "needed.";
    return nullptr;
  }

  // Use the resource handle data to connect to ICON and get the part name.
  const EquipmentPack& equipment_pack = context.equipment();
  INTR_ASSIGN_OR_RETURN(
      const icon::IconEquipment icon_equipment,
      icon::ConnectToIconEquipment(equipment_pack, kEquipmentSlot,
                                   *icon_channel_factory_));

  std::string gripper_part(kDefaultGripperPartName);
  if (params.has_gripper_part()) {
    LOG(INFO) << "Using part '" << params.gripper_part() << "' for ICON";
    gripper_part = params.gripper_part();
  }

  const OptimizationOptions options =
      topp::ToppOptionsWithJointAndCartesianConstraints(
          /*joint_acceleration=*/true, /*joint_jerk=*/false,
          /*cart_velocity=*/false, /*cart_acceleration=*/false);

  const std::vector<eigenmath::VectorNd> joint_configurations{initial_position,
                                                              goal_position};

  INTR_ASSIGN_OR_RETURN(const JointLimits joint_limits,
                        ToJointLimits(gripper_object.JointApplicationLimits()));
  INTR_ASSIGN_OR_RETURN(const CartesianLimits cartesian_limits,
                        gripper_object.GetCartesianLimits());

  const PathSegment path_segment{
      .joint_configurations = joint_configurations,
      .joint_blending_parameter_rad = std::vector<double>(
          joint_configurations.size(), kPathRefinerJointBlendingDeviationRad),
      .joint_limits = joint_limits,
      .cartesian_limits = cartesian_limits};

  const absl::Time trajectory_generation_start_time = absl::Now();
  INTR_ASSIGN_OR_RETURN(const topp::PathAndTrajectory path_and_trajectory,
                        topp::RefinePathAndComputeAccelerationLimitedTrajectory(
                            options, {path_segment}));
  LOG(INFO) << "Trajectory generation took "
            << absl::ToDoubleSeconds(absl::Now() -
                                     trajectory_generation_start_time)
            << " s.";
  LOG(INFO) << "Planned trajectory with duration "
            << absl::ToDoubleSeconds(
                   path_and_trajectory.trajectory_result.trajectory.Duration())
            << " s.";

  INTR_ASSIGN_OR_RETURN(
      const intrinsic_proto::icon::JointTrajectoryPVA trajectory_proto,
      ToProto(path_and_trajectory.trajectory_result.trajectory));

  INTR_RETURN_IF_ERROR(
      motion_planning::ExecuteJointTrajectory(
          context.logging_context().data_logger_context, icon_equipment.channel,
          gripper_part, trajectory_proto, kDefaultSettlingTimeoutSeconds,
          /*use_is_settled_as_condition=*/true, context.canceller()))
      .LogError();

  LOG(INFO) << "Move Gripper Joints ran successfully.";
  return nullptr;
}

}  // namespace intrinsic::skills
