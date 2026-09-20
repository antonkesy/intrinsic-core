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

#ifndef INTRINSIC_WORLD_TEST_WORLD_TEST_UTILS_H_
#define INTRINSIC_WORLD_TEST_WORLD_TEST_UTILS_H_

#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// Argument to CreateLinearChainRobot() used to specify how the robot should be
// constructed.
//
// This struct and its sub-structs are designed to have reasonable defaults that
// can be overridden through designated initializers (see go/totw/173).
struct LinearChainRobotParams {
  static constexpr absl::string_view kSolverName = "kinematic_chain";
  // Name given to the created RobotCollectionsEntity and robot GroupId (which
  // is used to support legacy APIs).
  std::string robot_name = "robot";

  // Additional alias given to the created RobotCollectionsEntity.
  std::string robot_alias;

  // Entity to which the robot's base link is parented.
  AttachmentEntityId parent_id = kRootEntityId;

  // Parent-relative pose of the robot's base link.
  Pose3d parent_t_base_link = Pose3d();

  // Link and joint parameters are specified below. link_params.size() must
  // equal joint_params.size() + 1.
  struct LinkParams {
    // Name given to the link Entity.
    std::string name;
  };
  std::vector<LinkParams> link_params = {{.name = "base_link"}};

  struct JointParams {
    // Name given to the joint Entity.
    std::string name;

    // Parent link-relative pose of the joint's axis.
    Pose3d parent_t_inboard = Pose3d();

    // Type of joint.
    intrinsic_proto::world::KinematicsComponent::MotionType motion_type =
        intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_REVOLUTE;

    // Joint axis in the inboard coordinate frame.
    eigenmath::Vector3d axis = eigenmath::Vector3d::UnitX();

    // Initial value. This value will be used to compute the joint's
    // inboard_t_outboard transform.
    double initial_value = 0.0;

    // Limits.
    double lower_value_limit = std::numeric_limits<double>::lowest();
    double upper_value_limit = std::numeric_limits<double>::max();
    double velocity_limit = std::numeric_limits<double>::max();
    double acceleration_limit = std::numeric_limits<double>::max();
    double jerk_limit = std::numeric_limits<double>::max();
    double effort_limit = std::numeric_limits<double>::max();
  };
  std::vector<JointParams> joint_params = {};

  // If true, a coordinate frame entity will be added as a child of the final
  // link.
  bool add_coordinate_frame = false;

  // If true, a solver key will be added to the robot from base to tip.
  bool add_solver_key = false;

  // If non empty, the robot entity will have a resource component with the
  // given name.
  std::string resource_name;

  // If set we will create a corresponding group for the given robot.
  // TODO(b/183439958): Remove once GroupIds are fully deprecated.
  bool create_group_id = false;

  // Control frequency of the robot.
  std::optional<double> control_frequency_hz = std::nullopt;
};

struct LinearChainRobotData {
  // ID of the created robot.
  RobotCollectionsEntityId robot_id;

  // The following are copies of the lists found in the robot's
  // CollectionsComponent, provided for convenience.
  std::vector<LinkEntityId> link_ids;
  std::vector<JointEntityId> joint_ids;
  std::vector<RobotCoordinateFrameEntityId> frame_ids;
};

absl::StatusOr<LinearChainRobotData> CreateLinearChainRobot(
    World* world, const LinearChainRobotParams& params);

// Creates parameters for a 4-link, 3-joint robot configured in the way as
// described by various "simple_assembly.pbtxt" files that were branched from
// each other but never modified. See:
// intrinsic/perception/tools/test_data/simple_assembly.pbtxt;rcl=275805374
// intrinsic/skills/test_data/simple_assembly.pbtxt;rcl=277099208
// intrinsic/motion_planning/trajectory_planning/test_data/simple_assembly.pbtxt;rcl=330794014
LinearChainRobotParams CreateSimpleAssemblyLinearChainRobotParams(
    absl::string_view robot_name);

// Creates a new robot with the following properties:
// - num_joints revolute joints in a linear chain
// - (num_joints + 1) links
// - base link parented to parent_id
// - if add_coordinate_frame is true, a coordinate frame as a child of the final
//   link
// - By default link length is 0, unless specified otherwise.
ABSL_DEPRECATED(
    "Use the version that takes CreateLinearChainRobotParams instead.")
absl::StatusOr<RobotCollectionsEntityId> CreateLinearChainRobot(
    World* world, AttachmentEntityId parent_id, absl::string_view robot_name,
    bool create_group_id, int num_joints, bool add_coordinate_frame,
    double link_length = 0);

absl::StatusOr<AttachmentEntityId> CreateEntityAndAttachToParent(
    World* world, absl::string_view local_name, AttachmentEntityId parent);

// Option struct for adding collections members as part of CreateEntityParams
// below.
struct CollectionMemberParams {
  // Entities to be added to a collections with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_LINKS.
  std::vector<EntityId> links;

  // Entities to be added to a collections with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_JOINTS.
  std::vector<EntityId> joints;

  // Entities to be added to a collections with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_SENSORS.
  std::vector<EntityId> sensors;

  // Entities to be added to a collections with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_ATTACHMENT_FRAME.
  std::vector<EntityId> attachment_frames;

  // Entities to be added to a collections with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_COORDINATE_FRAME.
  std::vector<EntityId> coordinate_frames;
};

// Option struct for CreateEntity() below.
struct CreateEntityParams {
  // Common entity properties.
  absl::string_view local_name;
  std::string alias;
  std::set<LabelId> labels;

  // If 'parent' is set, an AttachmentComponent with the given 'parent' and
  // 'parent_t_this' pose will be created.
  std::optional<AttachmentEntityId> parent;
  Pose3d parent_t_this;

  // If true, a GeometryComponent with some simple visual and collision geometry
  // will be created. Do not combine this with 'visual_geo', 'collision_geo' or
  // 'geometry'. If geometry is present, then both a default physics and
  // collisions component will also be created.
  bool add_geometry = false;

  // If any of 'visual_geo' and 'collision_geo' is not empty, a
  // GeometryComponent with the given visual and/or collision geometry will be
  // created. See intrinsic/geometry/api/shape_factory.h for
  // generating simple primitives. Usage example:
  //   .visual_geo = {MakeTransformedCenteredBox(1, 1, 1)}
  // If geometry is present, then both a default physics and collisions
  // component will also be created.
  NamedGeometrySet visual_geo;
  NamedGeometrySet collision_geo;

  // Convenience option for cases where visual and collision geometry should be
  // the same. If not empty, overrides both 'visual_geo' and 'collision_geo'.
  // If geometry is present, then both a default physics and collisions
  // component will also be created.
  NamedGeometrySet geometry;

  // If set, a CollisionComponent is created and collision exclusion pairs
  // with the given entities are added. If set and empty, an empty
  // CollisionComponent will be created. This automatically creates
  // CollisionComponents on the other entities if they are not already present.
  std::optional<std::vector<EntityId>> collision_exclusions;

  // If true, a KinematicsComponent with MOTION_TYPE_FIXED will be created.
  //
  // Do not use more than one of {'make_fixed_joint', 'make_prismatic_joint',
  // 'make_revolute_joint'}.
  bool make_fixed_joint = false;

  // If true, a KinematicsComponent with MOTION_TYPE_PRISMATIC will be created.
  // This option will result in a simple, default joint with initial value 0,
  // default axis (Z) and no configured limits, damping etc.
  //
  // Do not use more than one of {'make_fixed_joint', 'make_prismatic_joint',
  // 'make_revolute_joint'}.
  bool make_prismatic_joint = false;

  // If true, a KinematicsComponent with MOTION_TYPE_REVOLUTE will be created.
  // This option will result in a simple, default joint with initial value 0,
  // default axis (Z) and no configured limits, damping etc.
  //
  // Do not use more than one of {'make_fixed_joint', 'make_prismatic_joint',
  // 'make_revolute_joint'}.
  bool make_revolute_joint = false;

  // If true, a SensorComponent of type Camera with some default parameters will
  // be created.
  bool make_camera_sensor = false;

  // If true, a SensorComponent of type DepthCamera with some default parameters
  // will be created.
  bool make_depth_camera_sensor = false;

  // If not empty, a CollectionsMemberComponent is created and the new entity is
  // added as a member with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_LINKS
  // to the given collection entities. These already need to have a
  // CollectionsComponent (e.g., see 'create_empty_collection').
  std::vector<CollectionsEntityId> add_as_link_to_collections;

  // If not empty, a CollectionsMemberComponent is created and the new entity is
  // added as a member with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_JOINTS
  // to the given collection entities. These already need to have a
  // CollectionsComponent (e.g., see 'create_empty_collection').
  std::vector<CollectionsEntityId> add_as_joint_to_collections;

  // If not empty, a CollectionsMemberComponent is created and the new entity is
  // added as a member with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_SENSOR
  // to the given collection entities. These already need to have a
  // CollectionsComponent (e.g., see 'create_empty_collection').
  std::vector<CollectionsEntityId> add_as_sensor_to_collections;

  // If not empty, a CollectionsMemberComponent is created and the new entity is
  // added as a member with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_ATTACHMENT_FRAMES
  // to the given collection entities. These already need to have a
  // CollectionsComponent (e.g., see 'create_empty_collection').
  std::vector<CollectionsEntityId> add_as_attachment_frame_to_collections;

  // If not empty, a CollectionsMemberComponent is created and the new entity is
  // added as a member with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_COORDINATE_FRAMES
  // to the given collection entities. These already need to have a
  // CollectionsComponent (e.g., see 'create_empty_collection').
  std::vector<CollectionsEntityId> add_as_coordinate_frame_to_collections;

  // If true, an empty CollectionsComponent will be created. Cannot be used
  // together with 'create_collection_with_link_members'.
  bool create_empty_collection = false;

  // If not empty, a CollectionsComponent will be created and the given entities
  // will be added with type
  // intrinsic_proto::world::CollectionsComponent::COLLECTION_TYPE_LINKS. A
  // CollectionsMemberComponent will be created automatically on the member
  // entities if it doesn't already exist.
  std::vector<EntityId> create_collection_with_link_members;

  // If set (=contains at least one member id), a CollectionsComponent will
  // be created and the given entities will be added with the appropriate types.
  // A CollectionsMemberComponent will be created automatically on the member
  // entities if it doesn't already exist.
  CollectionMemberParams create_collection_with_members;

  // If true, a simple, default PhysicsComponent with a mass of 1 kg and other
  // default values will be created.
  bool make_default_physics = false;

  // If true, a simple, default RobotComponent with a "kinematic_chain" solver
  // will be created.
  bool make_default_robot = false;

  // If not empty, a ppr component with resource name will be created.
  absl::string_view resource_name;

  // If true, a SimulationComponent will be created which has the "IsStatic"
  // property set (all other properties have the default value).
  bool make_static = false;

  // If not empty, the new entity will have a UserDataComponent with the given
  // user data protos.
  WorldHashMap<std::string, google::protobuf::Any> user_data_protos;
};

namespace internal {

// Implementation details for CreateEntity() below.
absl::StatusOr<EntityId> CreateEntityImpl(World* world,
                                          const CreateEntityParams& params);

}  // namespace internal

// Creates an entity with all necessary components as specified through various
// options (see CreateEntityParams). Automatically modifies other entities to
// ensure world consistency (e.g., for collections memberships and collision
// exclusion pairs).
// Returns the id of the created entity as a typed entity id. The caller is
// responsible for ensuring that all the components required by the returned
// id are created via the corresponding options in 'params'. Usage example:
//
// INTR_ASSIGN_OR_RETURN(AttachmentEntityId result, CreateEntity(&world,
// {...}));
template <typename... ComponentTypes>
absl::StatusOr<world_entity_details::TypedResult<ComponentTypes...>>
CreateEntity(World* world, const CreateEntityParams& params) {
  INTR_ASSIGN_OR_RETURN(EntityId result,
                        internal::CreateEntityImpl(world, params));
  return world->ValidateEntity<ComponentTypes...>(result);
}

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_TEST_WORLD_TEST_UTILS_H_
