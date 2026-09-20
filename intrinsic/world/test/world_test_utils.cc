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

#include "intrinsic/world/test/world_test_utils.h"

#include <algorithm>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/io.h"
#include "intrinsic/geometry/api/shape_factory.h"
#include "intrinsic/geometry/storage/dummy_storage.h"
#include "intrinsic/kinematics/ik/chain_inverse_kinematics.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/proto/parse_text_proto.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/collections_member_component.h"
#include "intrinsic/world/component/collision_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/component/physics_component.h"
#include "intrinsic/world/component/ppr_component.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/component/sensor_component.h"
#include "intrinsic/world/component/simulation_component.h"
#include "intrinsic/world/component/user_data_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/grouping.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/proto/collections_component.pb.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/proto/sensor_component.pb.h"
#include "intrinsic/world/world.h"

using ::intrinsic::ParseTextProtoOrDie;
using ::intrinsic::eigenmath::Quaterniond;
using ::intrinsic::eigenmath::Vector3d;

namespace intrinsic {

absl::StatusOr<LinearChainRobotData> CreateLinearChainRobot(
    World* world, const LinearChainRobotParams& params) {
  if (params.robot_name.empty()) {
    return absl::InvalidArgumentError("robot_name must not be empty");
  }
  if (params.link_params.size() != params.joint_params.size() + 1) {
    return absl::InvalidArgumentError(
        "link_params.size() must equal joint_params.size() + 1");
  }

  LabelId robot_label(params.robot_name);
  if (params.create_group_id) {
    // TODO(b/183439958): Remove robot_group_id once GroupIds are fully
    // deprecated.
    GroupId robot_group_id(params.robot_name);
    INTR_RETURN_IF_ERROR(world->AddGroupId(robot_group_id));
  }

  // Initialize the robot and base link.
  INTR_ASSIGN_OR_RETURN(auto robot_and_base_link_ids,
                        world->CreateRobotAndBaseLink(params.parent_id),
                        _ << "when calling CreateRobotAndBaseLink("
                          << params.parent_id.value() << ")");
  LinkEntityId current_link_id = robot_and_base_link_ids.second;
  INTR_ASSIGN_OR_RETURN(auto link_ent, world->GetEntityById(current_link_id));
  INTR_RETURN_IF_ERROR(link_ent->SetLocalName(params.link_params.front().name));
  INTR_RETURN_IF_ERROR(link_ent->AddLabels({robot_label}));
  INTR_ASSIGN_OR_RETURN(auto link_attachment,
                        link_ent->GetComponent<AttachmentComponent>());
  link_attachment->SetParentTThis(params.parent_t_base_link);

  for (int i = 0; i < params.joint_params.size(); i++) {
    const LinearChainRobotParams::JointParams& joint_params =
        params.joint_params.at(i);
    INTR_ASSIGN_OR_RETURN(
        auto joint_and_link_ids,
        world->AddJoint(current_link_id, joint_params.parent_t_inboard,
                        joint_params.motion_type, joint_params.axis,
                        joint_params.initial_value,
                        joint_params.lower_value_limit,
                        joint_params.upper_value_limit),
        _ << "when calling AddJoint()");
    current_link_id = joint_and_link_ids.second;

    // Set the link local name.
    INTR_ASSIGN_OR_RETURN(link_ent, world->GetEntityById(current_link_id));
    INTR_RETURN_IF_ERROR(
        link_ent->SetLocalName(params.link_params.at(i + 1).name));
    INTR_RETURN_IF_ERROR(link_ent->AddLabels({robot_label}));

    // Set the joint local name.
    INTR_ASSIGN_OR_RETURN(auto joint_ent,
                          world->GetEntityById(joint_and_link_ids.first));
    INTR_RETURN_IF_ERROR(joint_ent->SetLocalName(joint_params.name));
    INTR_RETURN_IF_ERROR(joint_ent->AddLabels({robot_label}));

    // Set additional joint limits.
    INTR_ASSIGN_OR_RETURN(auto joint_kinematics,
                          joint_ent->GetComponent<KinematicsComponent>());
    INTR_RETURN_IF_ERROR(joint_kinematics->SetApplicationVelocityLimit(
        joint_params.velocity_limit, /*enforce_limits=*/false));
    INTR_RETURN_IF_ERROR(joint_kinematics->SetApplicationAccelerationLimit(
        joint_params.acceleration_limit, /*enforce_limits=*/false));
    INTR_RETURN_IF_ERROR(joint_kinematics->SetApplicationJerkLimit(
        joint_params.jerk_limit, /*enforce_limits=*/false));
    INTR_RETURN_IF_ERROR(joint_kinematics->SetApplicationEffortLimit(
        joint_params.effort_limit, /*enforce_limits=*/false));

    INTR_RETURN_IF_ERROR(joint_kinematics->SetSystemVelocityLimit(
        joint_params.velocity_limit, /*enforce_limits=*/true));
    INTR_RETURN_IF_ERROR(joint_kinematics->SetSystemAccelerationLimit(
        joint_params.acceleration_limit, /*enforce_limits=*/true));
    INTR_RETURN_IF_ERROR(joint_kinematics->SetSystemJerkLimit(
        joint_params.jerk_limit, /*enforce_limits=*/true));
    INTR_RETURN_IF_ERROR(joint_kinematics->SetSystemEffortLimit(
        joint_params.effort_limit, /*enforce_limits=*/true));
  }

  // Last link gets tip label
  INTR_RETURN_IF_ERROR(link_ent->AddLabels({labels::Tip()}));

  // If necessary, add the coordinate frame.
  if (params.add_coordinate_frame) {
    INTR_ASSIGN_OR_RETURN(
        RobotCoordinateFrameEntityId frame_id,
        world->AddCoordinateFrameToRobot(current_link_id, Pose3d()));

    INTR_ASSIGN_OR_RETURN(auto frame_entity, world->GetEntityById(frame_id));
    INTR_RETURN_IF_ERROR(frame_entity->SetLocalName(
        absl::StrCat(link_ent->GetLocalName(), "_frame")));
  }

  const RobotCollectionsEntityId robot_id = robot_and_base_link_ids.first;
  // Enable the robot's LabelId to be addressed through a GroupId.
  // TODO(b/183439958): Remove robot_group_id once GroupIds are fully
  // deprecated.
  if (params.create_group_id) {
    GroupId robot_group_id(params.robot_name);
    world->SetRobotCollectionsEntityIdsForRobotGroupId(robot_group_id,
                                                       {robot_id});
  }

  INTR_ASSIGN_OR_RETURN(auto robot_entity, world->GetEntityById(robot_id));
  INTR_RETURN_IF_ERROR(robot_entity->SetLocalName(params.robot_name));
  INTR_RETURN_IF_ERROR(robot_entity->SetAlias(params.robot_name));
  INTR_RETURN_IF_ERROR(robot_entity->AddLabels({robot_label}));
  if (!params.robot_alias.empty()) {
    INTR_RETURN_IF_ERROR(robot_entity->SetAlias(params.robot_alias));
  }

  // Setup the kinematic solver key for the new robot
  if (params.add_solver_key) {
    INTR_ASSIGN_OR_RETURN(AttachmentEntityId tip_id,
                          world->GetFinalEntityOfRobotKinematicChain(robot_id));
    INTR_ASSIGN_OR_RETURN(
        auto* robot, world->GetComponentByEntityId<RobotComponent>(robot_id));
    INTR_RETURN_IF_ERROR(
        robot->AddSolvableFrames(robot_and_base_link_ids.second, tip_id,
                                 LinearChainRobotParams::kSolverName));
  }

  // Setup the resource component
  if (!params.resource_name.empty()) {
    INTR_ASSIGN_OR_RETURN(auto* resource,
                          robot_entity->GetOrCreateComponent<PPRComponent>());
    resource->SetResourceName(params.resource_name);

    INTR_ASSIGN_OR_RETURN(auto* collections,
                          robot_entity->GetComponent<CollectionsComponent>());
    for (const auto member_id : collections->GetAllCollectionMembers()) {
      if (!world->ValidateEntity<AttachmentEntityId>(member_id).ok()) {
        continue;
      }
      INTR_ASSIGN_OR_RETURN(
          auto* member_ppr,
          world->GetOrCreateComponentByEntityId<PPRComponent>(member_id));
      member_ppr->SetResourceName(params.resource_name);
    }
  }

  LinearChainRobotData ret;
  ret.robot_id = robot_id;
  INTR_ASSIGN_OR_RETURN(ret.link_ids,
                        world->ValidateCollectionMembers<LinkEntityId>(
                            robot_id, CollectionsComponent::kLinks));
  INTR_ASSIGN_OR_RETURN(ret.joint_ids,
                        world->ValidateCollectionMembers<JointEntityId>(
                            robot_id, CollectionsComponent::kJoints));
  INTR_ASSIGN_OR_RETURN(
      ret.frame_ids,
      world->ValidateCollectionMembers<RobotCoordinateFrameEntityId>(
          robot_id, CollectionsComponent::kCoordinateFrames));
  return ret;
}

LinearChainRobotParams CreateSimpleAssemblyLinearChainRobotParams(
    absl::string_view robot_name) {
  LinearChainRobotParams ret = {
      .robot_name = std::string(robot_name),
      .link_params = {{.name = "base_link"},
                      {.name = "A1_link"},
                      {.name = "A2_link"},
                      {.name = "A3_link"}},
      .joint_params = {{.name = "A1",
                        .parent_t_inboard = Pose3d(Vector3d(0, 0, 0.4)),
                        .axis = -Vector3d::UnitZ(),
                        .lower_value_limit = -10,
                        .upper_value_limit = 12,
                        .velocity_limit = 1.1,
                        .acceleration_limit = 2.2,
                        .jerk_limit = 20},
                       {.name = "A2",
                        .parent_t_inboard =
                            Pose3d(Quaterniond(0.7071, 0, 0.7071, 0),
                                   Vector3d(0.025, 0, 0)),
                        .axis = Vector3d::UnitY(),
                        .lower_value_limit = -4,
                        .upper_value_limit = 5.2,
                        .velocity_limit = 2.1,
                        .acceleration_limit = 7.2,
                        .jerk_limit = 15},
                       {.name = "A3",
                        .parent_t_inboard =
                            Pose3d(Quaterniond(0.7071, 0, -0.7071, 0),
                                   Vector3d(0, 0, 0.455)),
                        .axis = Vector3d::UnitZ(),
                        .lower_value_limit = -2,
                        .upper_value_limit = 2.2,
                        .velocity_limit = 6.1,
                        .acceleration_limit = 3.2,
                        .jerk_limit = 10}},
      .add_solver_key = true,
      .resource_name = std::string(robot_name),
  };

  return ret;
}

absl::StatusOr<RobotCollectionsEntityId> CreateLinearChainRobot(
    World* world, AttachmentEntityId parent_id, absl::string_view robot_name,
    bool create_group_id, int num_joints, bool add_coordinate_frame,
    double link_length) {
  LinearChainRobotParams params = {
      .robot_name = std::string(robot_name),
      .parent_id = parent_id,
      .link_params = {{.name = "base"}},
      .joint_params = {},
      .add_coordinate_frame = add_coordinate_frame,
      .add_solver_key = false,
      .create_group_id = create_group_id,
  };
  for (int i = 0; i < num_joints; i++) {
    params.joint_params.emplace_back(LinearChainRobotParams::JointParams(
        {.name = absl::StrCat("joint_", i),
         .parent_t_inboard =
             Pose3d(eigenmath::Vector3d(0, std::max(link_length, 0.), 0)),
         .lower_value_limit = -6.28,
         .upper_value_limit = 6.28}));
    params.link_params.emplace_back(
        LinearChainRobotParams::LinkParams({.name = absl::StrCat("link_", i)}));
  }
  INTR_ASSIGN_OR_RETURN(auto data, CreateLinearChainRobot(world, params));
  return data.robot_id;
}

absl::StatusOr<AttachmentEntityId> CreateEntityAndAttachToParent(
    World* world, absl::string_view local_name, AttachmentEntityId parent) {
  INTR_ASSIGN_OR_RETURN(
      EntityId entity_id,
      CreateEntity(world, {.local_name = local_name, .parent = parent}));
  return AttachmentEntityId(entity_id);
}

bool IsSet(const CollectionMemberParams& params) {
  return !params.links.empty() || !params.joints.empty() ||
         !params.sensors.empty() || !params.attachment_frames.empty() ||
         !params.coordinate_frames.empty();
}

absl::Status AddCollectionMembersSymmetric(
    EntityId collections_id, const std::vector<EntityId>& member_ids,
    intrinsic_proto::world::CollectionsComponent::CollectionType
        collection_type,
    World& world) {
  INTR_ASSIGN_OR_RETURN(WorldEntity * collections_entity,
                        world.GetEntityById(collections_id));
  INTR_ASSIGN_OR_RETURN(
      CollectionsComponent * collections,
      collections_entity->GetOrCreateComponent<CollectionsComponent>());

  INTR_RETURN_IF_ERROR(collections->SetCollectionMembers(
      collection_type, std::vector<CollectionsMemberEntityId>(
                           member_ids.begin(), member_ids.end())));

  for (EntityId member_id : member_ids) {
    INTR_ASSIGN_OR_RETURN(WorldEntity * member_entity,
                          world.GetEntityById(member_id));
    INTR_ASSIGN_OR_RETURN(
        CollectionsMemberComponent * collections_member,
        member_entity->GetOrCreateComponent<CollectionsMemberComponent>());
    INTR_RETURN_IF_ERROR(collections_member->AddParentCollection(
        CollectionsEntityId(collections_id), collection_type));
  }

  return absl::OkStatus();
}

namespace internal {

absl::StatusOr<EntityId> CreateEntityImpl(World* world,
                                          const CreateEntityParams& params) {
  EntityId entity_id = world->CreateEntity();
  INTR_ASSIGN_OR_RETURN(WorldEntity * entity, world->GetEntityById(entity_id));

  if (!params.local_name.empty()) {
    INTR_RETURN_IF_ERROR(entity->SetLocalName(params.local_name));
  }
  if (!params.alias.empty()) {
    INTR_RETURN_IF_ERROR(entity->SetAlias(params.alias));
  }
  if (!params.labels.empty()) {
    INTR_RETURN_IF_ERROR(entity->AddLabels(params.labels));
  }

  if (params.parent.has_value()) {
    INTR_ASSIGN_OR_RETURN(AttachmentComponent * attachment,
                          entity->GetOrCreateComponent<AttachmentComponent>());
    attachment->SetParentId(*params.parent);
    attachment->SetParentTThis(params.parent_t_this);
  }

  const int num_geometry_specs =
      params.add_geometry +
      (!params.visual_geo.empty() || !params.collision_geo.empty()) +
      !params.geometry.empty();
  if (num_geometry_specs > 1) {
    return absl::InvalidArgumentError(
        "Do not use more than one of 'add_default_geometry', {'visual_geo', "
        "'collision_geo'} and 'geometry'.");
  } else if (num_geometry_specs == 1) {
    NamedGeometrySet collision_geo;
    NamedGeometrySet visual_geo;
    if (params.add_geometry) {
      NamedGeometrySet geo = {{"0", MakeTransformedCenteredBox(0.1, 0.1, 0.1)}};
      collision_geo = geo;
      visual_geo = geo;
    } else if (!params.geometry.empty()) {
      collision_geo = params.geometry;
      visual_geo = params.geometry;
    } else {
      if (!params.visual_geo.empty()) {
        visual_geo = params.visual_geo;
      }
      if (!params.collision_geo.empty()) {
        collision_geo = params.collision_geo;
      }
    }

    // Deserialize from proto so that we have both proto and geometry set.
    ::intrinsic_proto::world::GeometryComponent geo_component_proto;
    auto& named_geos = *geo_component_proto.mutable_named_geometries();
    for (const auto& [name, geo] : collision_geo) {
      INTR_ASSIGN_OR_RETURN(auto geo_proto, ToProto(geo, nullptr));
      (*named_geos[kKindCollisionGeometry].mutable_named_geometries())[name] =
          std::move(geo_proto);
    }

    for (const auto& [name, geo] : visual_geo) {
      INTR_ASSIGN_OR_RETURN(auto geo_proto, ToProto(geo, nullptr));
      (*named_geos[kKindVisualGeometry].mutable_named_geometries())[name] =
          std::move(geo_proto);
    }

    // Trick the geometry component into immediately deserializing geos to
    // support old tests.
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<GeometryComponent> geo_component,
        GeometryComponent::FromProto(
            geo_component_proto, GetDummyGeometryLibrary()->Deserializer()));

    INTR_RETURN_IF_ERROR(
        entity->SetComponent<GeometryComponent>(std::move(geo_component)));
  }

  if (num_geometry_specs > 0 || params.collision_exclusions.has_value()) {
    INTR_ASSIGN_OR_RETURN(CollisionComponent * collision,
                          entity->GetOrCreateComponent<CollisionComponent>());
    for (EntityId other_id :
         params.collision_exclusions.value_or(std::vector<EntityId>{})) {
      INTR_ASSIGN_OR_RETURN(WorldEntity * other_entity,
                            world->GetEntityById(other_id));
      INTR_ASSIGN_OR_RETURN(
          CollisionComponent * other_collision,
          other_entity->GetOrCreateComponent<CollisionComponent>());
      collision->AddExclusionId(PhysicalEntityId(other_id));
      other_collision->AddExclusionId(PhysicalEntityId(entity_id));
    }
  }

  if ((params.make_fixed_joint + params.make_prismatic_joint +
       params.make_revolute_joint) > 1) {
    return absl::InvalidArgumentError(
        "Do not use more than one of {'make_fixed_joint', "
        "'make_prismatic_joint', 'make_revolute_joint'} simultaneously.");
  }

  if (params.make_fixed_joint) {
    INTR_ASSIGN_OR_RETURN(KinematicsComponent * kinematics,
                          entity->GetOrCreateComponent<KinematicsComponent>());
    kinematics->SetMotionType(
        intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_FIXED);
  }

  if (params.make_revolute_joint) {
    INTR_ASSIGN_OR_RETURN(KinematicsComponent * kinematics,
                          entity->GetOrCreateComponent<KinematicsComponent>());
    kinematics->SetMotionType(
        intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_REVOLUTE);
  }

  if (params.make_prismatic_joint) {
    INTR_ASSIGN_OR_RETURN(KinematicsComponent * kinematics,
                          entity->GetOrCreateComponent<KinematicsComponent>());
    kinematics->SetMotionType(
        intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_PRISMATIC);
  }

  if (params.make_camera_sensor && params.make_depth_camera_sensor) {
    return absl::InvalidArgumentError(
        "Do not use more than one of {'make_camera_sensor', "
        "'make_depth_camera_sensor'} simultaneously.");
  }

  if (params.make_camera_sensor) {
    INTR_ASSIGN_OR_RETURN(SensorComponent * sensor,
                          entity->GetOrCreateComponent<SensorComponent>());
    intrinsic_proto::world::SensorComponent proto = ParseTextProtoOrDie(R"pb(
      update_rate: 10
      camera {
        properties {
          horizontal_fov: 1.2
          image { width: 1920 height: 1200 format: FORMAT_R8G8B8 }
          clip { near: 0.02 far: 2 }
          noise { type: TYPE_GAUSSIAN stddev: 0.007 }
          intrinsics { fx: 1400 fy: 1400 cx: 960 cy: 600 }
        }
      }
    )pb");
    INTR_RETURN_IF_ERROR(sensor->UpdateFromProto(proto));
  }

  if (params.make_depth_camera_sensor) {
    INTR_ASSIGN_OR_RETURN(SensorComponent * sensor,
                          entity->GetOrCreateComponent<SensorComponent>());
    intrinsic_proto::world::SensorComponent proto = ParseTextProtoOrDie(R"pb(
      update_rate: 10
      depth_camera {
        properties {
          horizontal_fov: 1.2
          image { width: 1920 height: 1200 format: FORMAT_R8G8B8 }
          clip { near: 0.02 far: 2 }
          noise { type: TYPE_GAUSSIAN stddev: 0.007 }
          intrinsics { fx: 1400 fy: 1400 cx: 960 cy: 600 }
        }
      }
    )pb");
    INTR_RETURN_IF_ERROR(sensor->UpdateFromProto(proto));
  }

  if (!params.add_as_link_to_collections.empty() ||
      !params.add_as_joint_to_collections.empty() ||
      !params.add_as_sensor_to_collections.empty() ||
      !params.add_as_attachment_frame_to_collections.empty() ||
      !params.add_as_coordinate_frame_to_collections.empty()) {
    std::vector<
        std::pair<CollectionsEntityId,
                  intrinsic_proto::world::CollectionsComponent::CollectionType>>
        collections;
    collections.reserve(params.add_as_link_to_collections.size() +
                        params.add_as_joint_to_collections.size() +
                        params.add_as_sensor_to_collections.size() +
                        params.add_as_attachment_frame_to_collections.size() +
                        params.add_as_coordinate_frame_to_collections.size());
    for (CollectionsEntityId id : params.add_as_link_to_collections) {
      collections.emplace_back(id, CollectionsComponent::kLinks);
    }
    for (CollectionsEntityId id : params.add_as_sensor_to_collections) {
      collections.emplace_back(id, CollectionsComponent::kSensors);
    }
    for (CollectionsEntityId id : params.add_as_joint_to_collections) {
      collections.emplace_back(id, CollectionsComponent::kJoints);
    }
    for (CollectionsEntityId id :
         params.add_as_attachment_frame_to_collections) {
      collections.emplace_back(id, CollectionsComponent::kAttachmentFrames);
    }
    for (CollectionsEntityId id :
         params.add_as_coordinate_frame_to_collections) {
      collections.emplace_back(id, CollectionsComponent::kCoordinateFrames);
    }

    INTR_ASSIGN_OR_RETURN(
        CollectionsMemberComponent * collections_member,
        entity->GetOrCreateComponent<CollectionsMemberComponent>());
    for (const auto& [parent_collection_id, member_type] : collections) {
      INTR_RETURN_IF_ERROR(collections_member->AddParentCollection(
          parent_collection_id, member_type));

      INTR_ASSIGN_OR_RETURN(CollectionsComponent * collections,
                            world->GetComponentByEntityId<CollectionsComponent>(
                                parent_collection_id));
      std::vector<CollectionsMemberEntityId> members =
          collections->GetCollectionMembers(member_type);
      members.push_back(CollectionsMemberEntityId(entity_id));
      INTR_RETURN_IF_ERROR(
          collections->SetCollectionMembers(member_type, std::move(members)));
    }
  }

  if ((!params.create_collection_with_link_members.empty() +
       params.create_empty_collection +
       IsSet(params.create_collection_with_members)) > 1) {
    return absl::InvalidArgumentError(
        "Do not use more than one of {'create_empty_collection', "
        "'create_collection_with_link_members', "
        "'create_collection_with_members'} simultaneously.");
  }

  if (params.create_empty_collection) {
    INTR_RETURN_IF_ERROR(entity->CreateComponent<CollectionsComponent>());
  }

  if (!params.create_collection_with_link_members.empty()) {
    INTR_RETURN_IF_ERROR(AddCollectionMembersSymmetric(
        entity_id, params.create_collection_with_link_members,
        intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_LINKS,
        *world));
  }

  if (IsSet(params.create_collection_with_members)) {
    INTR_RETURN_IF_ERROR(AddCollectionMembersSymmetric(
        entity_id, params.create_collection_with_members.links,
        intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_LINKS,
        *world));
    INTR_RETURN_IF_ERROR(AddCollectionMembersSymmetric(
        entity_id, params.create_collection_with_members.joints,
        intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_JOINTS,
        *world));
    INTR_RETURN_IF_ERROR(AddCollectionMembersSymmetric(
        entity_id, params.create_collection_with_members.sensors,
        intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_SENSORS,
        *world));
    INTR_RETURN_IF_ERROR(AddCollectionMembersSymmetric(
        entity_id, params.create_collection_with_members.attachment_frames,
        intrinsic_proto::world::CollectionsComponent::
            COLLECTION_TYPE_ATTACHMENT_FRAMES,
        *world));
    INTR_RETURN_IF_ERROR(AddCollectionMembersSymmetric(
        entity_id, params.create_collection_with_members.coordinate_frames,
        intrinsic_proto::world::CollectionsComponent::
            COLLECTION_TYPE_COORDINATE_FRAMES,
        *world));
  }

  if (num_geometry_specs > 0 || params.make_default_physics) {
    INTR_RETURN_IF_ERROR(
        entity->GetOrCreateComponent<PhysicsComponent>().status());
  }

  if (params.make_default_robot) {
    INTR_ASSIGN_OR_RETURN(RobotComponent * robot,
                          entity->GetOrCreateComponent<RobotComponent>());
    INTR_RETURN_IF_ERROR(robot->AddSolvableFrames(
        AttachmentEntityId(kInvalidEntityId),
        AttachmentEntityId(kInvalidEntityId),
        kinematics::ChainInverseKinematics::kSolverName));
  }

  if (!params.resource_name.empty()) {
    INTR_ASSIGN_OR_RETURN(PPRComponent * ppr,
                          entity->GetOrCreateComponent<PPRComponent>());
    ppr->SetResourceName(params.resource_name);

    // If we're creating a collections entity then also set all of the member
    // entities to have the same PPR component.
    if (entity->HasComponent<CollectionsComponent>()) {
      INTR_ASSIGN_OR_RETURN(CollectionsComponent * collection,
                            entity->GetComponent<CollectionsComponent>());
      for (auto child_id : collection->GetAllCollectionMembers()) {
        INTR_ASSIGN_OR_RETURN(
            PPRComponent * child_ppr,
            world->GetOrCreateComponentByEntityId<PPRComponent>(child_id));
        child_ppr->SetResourceName(params.resource_name);
      }
    }
  }

  if (params.make_static) {
    INTR_ASSIGN_OR_RETURN(SimulationComponent * simulation,
                          entity->GetOrCreateComponent<SimulationComponent>());
    simulation->SetIsStatic(true);
  }

  if (!params.user_data_protos.empty()) {
    INTR_ASSIGN_OR_RETURN(UserDataComponent * user_data_comp,
                          entity->GetOrCreateComponent<UserDataComponent>());
    user_data_comp->MutableUserDataProtos() = params.user_data_protos;
  }

  return entity_id;
}

}  // namespace internal

}  // namespace intrinsic
