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

#ifndef INTRINSIC_WORLD_ASPECTS_ENTITY_WORLD_INTERFACE_H_
#define INTRINSIC_WORLD_ASPECTS_ENTITY_WORLD_INTERFACE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/collections_member_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/grouping.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/proto/collections_component.pb.h"
#include "intrinsic/world/proto/collision_checker_config.pb.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/proto/physics_component.pb.h"
#include "intrinsic/world/proto/sensor_component.pb.h"

// This file contains an interface for the entity based aspects to use in place
// of the World object. This interface is intended to live for the duration of
// the migration away from aspects and be inlined with the World object after.

namespace intrinsic {
namespace entity_aspect_world_details {

class EntityWorld {
 public:
  virtual ~EntityWorld() = default;

  // ------------------------------------
  // -- General Entity methods
  // ------------------------------------

  // Returns all of the entity ids in this World.
  virtual WorldHashSet<EntityId> GetEntityIds() const = 0;

  // Attempts to find the EntityId that corresponds to the provided DofId.
  virtual absl::StatusOr<EntityId> GetEntityIdByDofId(DofId dof_id) const = 0;

  // Attempts to find the DofId that corresponds to the provided EntityId.
  virtual absl::StatusOr<DofId> GetDofIdByEntityId(
      EntityId entity_id) const = 0;

  // Returns all of the entity handles in this World that have all of the
  // given ComponentTypes.
  //
  // The set template pack ComponentTypes can be one of two alternatives:
  // - A pack of component types like AttachmentComponentType or
  //   CollisionComponentType.
  // - A single parameter of type TypedEntityId<T...> where T is a set of
  //   component types.
  //
  // This facilitates being able to do things like:
  // INTR_ASSIGN_OR_RETURN(std::vector<TypedEntityId<CollisionComponentType>>
  // result, world->GetTypedEntityIds<CollisionComponentType>());
  //
  // INTR_ASSIGN_OR_RETURN(std::vector<PhysicalEntityId> result,
  // world->GetTypedEntityIds<PhysicalEntityId>());
  template <typename... ComponentTypes>
  std::vector<world_entity_details::TypedResult<ComponentTypes...>>
  GetTypedEntityIds() const;

  // Creates a new Entity in the World and returns its id.
  virtual EntityId CreateEntity() = 0;

  // Removes the given entity. If there are any other entities in the World that
  // point to this entity or are parented by this entity this operation could
  // cause problems in the future as this method does not do any checks. Use
  // carefully and only after removing the other references to this entity.
  virtual absl::Status RemoveEntity(EntityId entity_id) = 0;

  // Removes the given entity. This enforces that we do not have other entities
  // referencing the given entity. The enforcement limits the entities that can
  // be deleted, for example we can only delete leaf entities in the attachment
  // graph.
  virtual absl::Status SafelyRemoveEntity(EntityId entity_id) = 0;

  // Returns true if this entity id is part of the entities managed by this
  // world.
  virtual bool HasEntity(EntityId entity_id) const = 0;

  // Get a pointer to the entity based on the given id.
  virtual absl::StatusOr<const WorldEntity*> GetEntityById(
      EntityId id) const = 0;

  // Get a pointer to the entity based on the given id.
  virtual absl::StatusOr<WorldEntity*> GetEntityById(EntityId id) = 0;

  // Returns the set of entities that contain the given label.
  virtual WorldHashSet<EntityId> FindByLabel(LabelId label) const = 0;

  // Return the local name of a given EntityId. If the EntityId is invalid, an
  // error string is return.
  virtual std::string GetLocalNameForEntityById(EntityId id) const = 0;

  // ------------------------------------
  // -- PhysicalWorld methods
  // ------------------------------------

  // Returns all of the direct child objects of the given AttachmentEntityId.
  // The handle must be a valid entity.
  virtual std::vector<AttachmentEntityId> GetChildrenOf(
      AttachmentEntityId entity_handle) const = 0;

  // Adds an attachment component to the given entity. It then populates the
  // attachment component with the data that is has a parent with the given id
  // and that it has the given transform to that parent. Returns the handle to
  // the entity as a AttachmentEntityId. If the component already exists it will
  // return an error.
  virtual absl::StatusOr<AttachmentEntityId> CreateAttachmentComponent(
      AttachmentEntityId parent_handle, EntityId id,
      const Pose3d& parent_t_entity,
      std::optional<absl::Time> timestamp = std::nullopt) = 0;

  // Removes the attachment component from the entity. The given entity must not
  // have any children. If it has children they must be removed or reparented
  // first before removing this component.
  virtual absl::Status DeleteAttachmentComponent(
      AttachmentEntityId entity_handle) = 0;

  // Updates a transform between two physical entities. These entities must be
  // directly connected. Either A or B can be the parent entity.
  virtual void UpdateAttachmentPose(AttachmentEntityId a_handle,
                                    AttachmentEntityId b_handle,
                                    const Pose3d& a_t_b) = 0;

  // Adds the given entity as a child to parent entity. This will only succeed
  // if the two entities are not already connected. This moves any child
  // entities with the given entity.
  virtual absl::Status ReparentEntity(
      AttachmentEntityId new_parent_handle, AttachmentEntityId child_handle,
      const Pose3d& parent_t_entity,
      std::optional<absl::Time> timestamp = std::nullopt) = 0;

  // Returns a transform between the two given entities. These entity may not be
  // directly connected but still have an indirect connection. This method
  // inspects these connections to calculate a transform that describes the pose
  // between the two given entities. Returns the transform in the form
  // of a_t_b.
  virtual Pose3d GetTransform(AttachmentEntityId a_handle,
                              AttachmentEntityId b_handle) const = 0;

  // Returns the common ancestor of the two given handles, the handles must be
  // present within the world.
  virtual absl::StatusOr<AttachmentEntityId> FindCommonAncestor(
      AttachmentEntityId a_handle, AttachmentEntityId b_handle) const = 0;

  // Returns a map of labels to the entity ids that match with the key label.
  virtual WorldHashMap<LabelId, WorldHashSet<EntityId>> GetLabelsMap()
      const = 0;

  // Marks the given connection between a_handle and b_handle as inaccurate or
  // not inaccurate based on the given input. NOTE: There must be a child/parent
  // relationship between a_handle and b_handle. Either a_handle is the parent
  // and b_handle is the child or vice versa.
  virtual absl::Status MarkTransformInaccuracy(AttachmentEntityId a_handle,
                                               AttachmentEntityId b_handle,
                                               bool inaccurate) = 0;

  // Returns any connections that are marked inaccurate between a_handle and
  // b_handle. a_handle and b_handle do not have to be directly connected.
  // The returned pairs are ordered so that the first element of each pair is
  // closer to a_handle in the attachment graph and the second element is closer
  // to b_handle.
  virtual WorldHashSet<std::pair<AttachmentEntityId, AttachmentEntityId>>
  GetInaccurateTransforms(AttachmentEntityId a_handle,
                          AttachmentEntityId b_handle) const = 0;

  // Updates exactly one inaccurate transform between two physical objects,
  // clearing the inaccurate flag in the process. Returns an error if there are
  // 0 or more than 1 inaccurate transform between the two entities. If there is
  // only one inaccurate transform we return the entity id of the child entity
  // of the transform. That is if we have a chain entities of A -> C -> D -> B,
  // and we have the inaccuare transform between C and D, then the entity id for
  // D would be returned.
  virtual absl::StatusOr<AttachmentEntityId> UpdateIndirectTransform(
      AttachmentEntityId a_handle, AttachmentEntityId b_handle,
      const Pose3d& a_t_b,
      std::optional<absl::Time> timestamp = std::nullopt) = 0;

  // Creates a physics component for the given entity with the given proto.
  virtual absl::StatusOr<PhysicsEntityId> CreatePhysicsComponent(
      EntityId id,
      const intrinsic_proto::world::PhysicsComponent& physics_component) = 0;

  // Creates a sensor component for the given entity with the given proto.
  virtual absl::StatusOr<SensorEntityId> CreateSensorComponent(
      EntityId id,
      const intrinsic_proto::world::SensorComponent& sensor_component) = 0;

  // ------------------------------------
  // -- CollisionWorld methods
  // ------------------------------------

  // Adds a collision component to the given entity.
  virtual absl::StatusOr<CollisionEntityId> CreateCollisionComponent(
      EntityId id) = 0;

  // Add the input pair to the list of pairwise ids that should not be
  // collision checked against each other.
  virtual absl::Status AddExclusionPair(PhysicalEntityId first,
                                        PhysicalEntityId second) = 0;

  // Remove the input pair from the list of pairwise ids that should not be
  // collision checked against each other. The order of the object_ids in the
  // pair is not importnat.
  virtual absl::Status RemoveExclusionPair(PhysicalEntityId first,
                                           PhysicalEntityId second) = 0;

  // Cleans up the collision exclusions in the world by removing any non-mutual
  // exclusions (including "dangling" exclusions with non-existing entity ids).
  // I.e., for any pair of entities A and B we remove the collision exclusion
  // A->B if the collision exclusion B->A is not present or if B does not exist.
  virtual absl::Status RemoveNonmutualExclusions() = 0;

  // Wipe the list of pairwise ids that should not be collision checked against
  // each other.
  virtual void ClearExclusionPairs() = 0;

  // Return the list of pairwise object ids that are not collision checked
  // against each other.
  virtual std::vector<std::pair<PhysicalEntityId, PhysicalEntityId>>
  GetExclusionPairs(bool filter_empty_collision_geometry) const = 0;

  // Build and return a CollisionChecker object from the input dynamic object
  // list and the given rule set. If `collision_checker_config` is unconfigured
  // or omitted (CONFIG_NOT_SET), defaults to
  // `kDefaultCollisionCheckerConfigCase`.
  virtual absl::StatusOr<std::shared_ptr<CollisionChecker>> GetCollisionChecker(
      const WorldHashSet<PhysicalEntityId>& dynamic_objects,
      const intrinsic_proto::RuleSet& rule_set,
      const intrinsic_proto::world::CollisionCheckerConfig&
          collision_checker_config = {}) const = 0;

  // Build and return a CollisionChecker object from the input dynamic object
  // list and the default rule set. If `collision_checker_config` is
  // unconfigured or omitted (CONFIG_NOT_SET), defaults to
  // `kDefaultCollisionCheckerConfigCase`.
  virtual absl::StatusOr<std::shared_ptr<CollisionChecker>> GetCollisionChecker(
      const WorldHashSet<PhysicalEntityId>& dynamic_objects,
      const intrinsic_proto::world::CollisionCheckerConfig&
          collision_checker_config = {}) const = 0;

  // Return true if the two object ids appear in the exclusion map.
  virtual bool IsCollisionExcluded(PhysicalEntityId object_id_1,
                                   PhysicalEntityId object_id_2) const = 0;

  // Get the mesh, belonging to this object_id in model space.
  virtual NamedGeometrySet GetLocallyTransformedShapes(
      PhysicalEntityId object_id) const = 0;

  // Get the combined shapes of the inpput object_id in their current
  // transform relative to world frame. Note this does not change the
  // representations from models space to world space or vice versa.
  virtual NamedGeometrySet GetGlobalyTransformedShapes(
      PhysicalEntityId object_id) const = 0;

  // Get the object ids that are excluded from the input list.
  virtual WorldHashSet<PhysicalEntityId> GetExcludedObjectdIds(
      const WorldHashSet<PhysicalEntityId>& object_ids) const = 0;

  // Return true if no collision geometry is defined. This can be due to not
  // being a CollisionComponent or a GeometryComponent.
  virtual bool HasEmptyCollisionGeometry(const EntityId& id) const = 0;

  // Create a single static spatial tree representative of the input static
  // physical objects in world space.
  virtual NamedGeometrySet GetStaticSpatialTreeInWorldSpace(
      const WorldHashSet<PhysicalEntityId>& static_object_ids) const = 0;

  // Given object_id find its memoized SpatialTree representation in model space
  // and return its key.
  virtual NamedGeometrySet GetSpatialTreeInModelSpace(
      PhysicalEntityId object_id) const = 0;

  // Returns the current default rule set used with this world.
  virtual intrinsic_proto::RuleSet GetDefaultRuleSet() const = 0;

  // Sets the current default rule set used with this world.
  virtual absl::Status SetDefaultRuleSet(
      const intrinsic_proto::RuleSet& rule_set) = 0;

  // ------------------------------------
  // -- Grouping methods
  // ------------------------------------

  // Returns the set of valid GroupIds (which is a subset of the LabelIds found
  // on all Entities).
  ABSL_DEPRECATED(
      "GroupIds are deprecated; use LabelIds on Entities instead. This "
      "function should only be called by EntityGrouping.")
  virtual const WorldHashSet<GroupId>& GetGroupIds() const = 0;

  // Attempts to add a GroupId to the set of valid GroupIds. Returns an error if
  // the group already exists or an equivalent LabelId exists on an Entity.
  ABSL_DEPRECATED(
      "GroupIds are deprecated; use LabelIds on Entities instead. This "
      "function should only be called by EntityGrouping.")
  virtual absl::Status AddGroupId(const GroupId& group_id) = 0;

  // Attempts to remove a GroupId to the set of valid GroupIds. Returns an error
  // if the group does not exist.
  ABSL_DEPRECATED(
      "GroupIds are deprecated; use LabelIds on Entities instead. This "
      "function should only be called by EntityGrouping.")
  virtual absl::Status RemoveGroupId(const GroupId& group_id) = 0;

  // ------------------------------------
  // -- KinematicWorld methods
  // ------------------------------------

  // Checks if the DoF associated with joint_handle can be set to raw_value.
  virtual absl::Status CheckDofRawValue(JointEntityId joint_handle,
                                        double raw_value) const = 0;

  // Attempts to sets the raw value of the specified DoF. If successful, updates
  // the DoF's raw value and affected transforms (some of which may be
  // associated with dependent DoFs).
  virtual absl::Status SetDofRawValue(
      JointEntityId joint_handle, double raw_value, bool enforce_limits,
      std::optional<absl::Time> timestamp = std::nullopt) = 0;

  // Given a RobotCollectionsEntityId, returns the ordered list of DoFs in that
  // robot. (Note that unlike getting the joints directly from
  // CollectionsComponent, this function will filter out fixed and fully
  // dependent joints.)
  virtual absl::StatusOr<std::vector<JointEntityId>> GetRobotDofs(
      RobotCollectionsEntityId robot_handle) const = 0;

  // Returns all the final entities of a kinematic tree.
  virtual absl::StatusOr<WorldHashSet<AttachmentEntityId>>
  GetFinalEntitiesOfRobot(CollectionsEntityId robot_id) const = 0;

  // Checks if robot_id represents a linear kinematic chain. If it does, returns
  // the ID of the final entity of that chain (usually a link or coordinate
  // frame). If it does not, returns kInvalidEntityId.
  virtual absl::StatusOr<AttachmentEntityId>
  GetFinalEntityOfRobotKinematicChain(CollectionsEntityId robot_id) const = 0;

  // Check whether a robot is a kinematic chain or not.
  virtual absl::StatusOr<bool> IsChainKinematicChain(
      RobotCollectionsEntityId robot_id) const = 0;

  // Returns an ordered set of RobotCollectionsEntityIds corresponding to the
  // set of Robots (in order of appearance) in the chain of entities between id1
  // and id2.
  virtual absl::StatusOr<std::vector<RobotCollectionsEntityId>>
  GetRobotIdsInChain(AttachmentEntityId id1, AttachmentEntityId id2) const = 0;

  // Returns a set of CartesianKinematicViews. These views correspond to DOFs
  // that can be used to position id2's Entity with respect to id1's Entity. A
  // set is returned because there can be multiple independent ways of moving
  // the given objects. If no DOFs can modify the pose between the objects then
  // an empty set is returned.
  //
  // An example usage might be to pass in the gripper pickup point as one of the
  // object ids and the object being picked up as the second object id. Then
  // later when trying to pick up the object you would specify that you want the
  // pose between the two objects to be identity. This will solve IK and find
  // the correct Dofs to position the objects with respect to each other.
  // Another example might be to specify a glue dispenser attached to a robot
  // and an object that needs to have glue deposited on it attached to a
  // different robot. The returned kinematic views may control one or both arms
  // to ensure that when a pose is specified relative to the two objects it is
  // achieved.
  virtual absl::StatusOr<std::vector<std::unique_ptr<CartesianKinematicView>>>
  GetCartesianKinematicViews(AttachmentEntityId id1,
                             AttachmentEntityId id2) = 0;

  virtual absl::StatusOr<
      std::vector<std::unique_ptr<const CartesianKinematicView>>>
  GetCartesianKinematicViews(AttachmentEntityId id1,
                             AttachmentEntityId id2) const = 0;

  // Returns a cartesian view that correspond to the chain of DOFs between id1
  // and id2. As opposed to GetCartesianKinematicViews, this cartesian view can
  // span multiple robots connected in a chain.
  virtual absl::StatusOr<std::unique_ptr<const CartesianKinematicView>>
  GetCartesianKinematicView(AttachmentEntityId id1,
                            AttachmentEntityId id2) const = 0;

  virtual absl::StatusOr<std::unique_ptr<CartesianKinematicView>>
  GetCartesianKinematicView(AttachmentEntityId id1, AttachmentEntityId id2) = 0;

  // Gets the DofKinematicView representing any dofs in the chain between the
  // two given entities. The resulting view will allow you to indirectly control
  // the pose between the two given entities. Any DOF on the entities themselves
  // that does not change the pose between them will not be part of the
  // resulting view.
  //
  // NOTE: This may cross robot boundaries or may be a subset of a robot.
  virtual absl::StatusOr<std::unique_ptr<DofKinematicView>> GetDofKinematicView(
      AttachmentEntityId id1, AttachmentEntityId id2) = 0;

  // Gets the DofKinematicView representing any dofs in the chain between the
  // two given entities. The resulting view will allow you to indirectly control
  // the pose between the two given entities. Any DOF on the entities themselves
  // that does not change the pose between them will not be part of the
  // resulting view.
  //
  // NOTE: This may cross robot boundaries or may be a subset of a robot.
  virtual absl::StatusOr<std::unique_ptr<const DofKinematicView>>
  GetDofKinematicView(AttachmentEntityId id1, AttachmentEntityId id2) const = 0;

  // Gets the DofKinematicView associated with the given joint ids.
  virtual absl::StatusOr<std::unique_ptr<DofKinematicView>> GetDofKinematicView(
      const std::vector<JointEntityId>& joint_ids) = 0;
  virtual absl::StatusOr<std::unique_ptr<const DofKinematicView>>
  GetDofKinematicView(const std::vector<JointEntityId>& joint_ids) const = 0;

  // Gets the DofKinematicView for a given RobotCollectionsEntity.
  virtual absl::StatusOr<std::unique_ptr<DofKinematicView>> GetDofKinematicView(
      RobotCollectionsEntityId robot_id) = 0;
  virtual absl::StatusOr<std::unique_ptr<const DofKinematicView>>
  GetDofKinematicView(RobotCollectionsEntityId robot_id) const = 0;

  // Returns a DofKinematicView for the specified robot GroupId. This function
  // is intended to match the behavior of Robots::GetDofKinematicView(), meaning
  // the returned view does not necessarily represent all of the DoFs in the
  // robot group.
  ABSL_DEPRECATED(
      "GroupIds are deprecated. Prefer to use World functions that take "
      "RobotCollectionsEntityId arguments.")
  virtual absl::StatusOr<std::unique_ptr<DofKinematicView>>
  GetDofKinematicViewForRobotGroupId(GroupId robot_group_id) = 0;
  ABSL_DEPRECATED(
      "GroupIds are deprecated. Prefer to use World functions that take "
      "RobotCollectionsEntityId arguments.")
  virtual absl::StatusOr<std::unique_ptr<const DofKinematicView>>
  GetDofKinematicViewForRobotGroupId(GroupId robot_group_id) const = 0;

  // ------------------------------------
  // -- Robots methods
  // ------------------------------------

  // Attempts to get the base link of a robot collections entity.
  virtual absl::StatusOr<LinkEntityId> GetBaseLink(
      CollectionsEntityId robot_id) const = 0;

  // Creates and returns new robot and link entities, with the link as a child
  // of the specified parent entity.
  virtual absl::StatusOr<std::pair<RobotCollectionsEntityId, LinkEntityId>>
  CreateRobotAndBaseLink(AttachmentEntityId parent_entity_id) = 0;

  // Creates and returns new joint and link entities as children of the
  // specified parent. If the parent is part of a robot, the new joint and link
  // entities will be added to that robot.
  virtual absl::StatusOr<std::pair<JointEntityId, LinkEntityId>> AddJoint(
      AttachmentEntityId parent_id, const Pose3d& parent_t_inboard,
      intrinsic_proto::world::KinematicsComponent::MotionType motion_type,
      const eigenmath::Vector3d& axis, double value, double lower_limit,
      double upper_limit) = 0;

  // Creates and returns a new entities as a child of the specified parent link.
  // The parent link must be part of a robot, whose data will be updated by this
  // function.
  virtual absl::StatusOr<RobotCoordinateFrameEntityId>
  AddCoordinateFrameToRobot(LinkEntityId parent_link_id,
                            const Pose3d& parent_link_t_coordinate_frame) = 0;

  // Sorts the specified robot's link and joint lists according to the rules
  // described in intrinsic/world/proto/collections_component.proto.
  virtual absl::Status SortRobotLinkAndJointLists(
      RobotCollectionsEntityId robot_id) = 0;

  // Removes RobotCollectionsEntities, their member Entities, and any children
  // of those Entities.
  virtual absl::Status RemoveRobots(
      const std::vector<RobotCollectionsEntityId>& robot_ids) = 0;

  // Returns the set of GroupIds that have RobotCollectionsEntityIds associated
  // with them (i.e. the GroupIds that are valid arguments to
  // GetRobotCollectionsEntityIdsForRobotGroupId).
  ABSL_DEPRECATED(
      "This function should only be called by EntityRobots. Instead, use "
      "World-level functions to interact directly with robots.")
  virtual WorldHashSet<GroupId> GetRobotGroupIds() const = 0;

  // Returns an ordered list of RobotCollectionsEntityIds associated with the
  // provided GroupId.
  ABSL_DEPRECATED(
      "This function should only be called by EntityRobots. Instead, use "
      "World-level functions to interact directly with robots.")
  virtual absl::StatusOr<const std::vector<RobotCollectionsEntityId>*>
  GetRobotCollectionsEntityIdsForRobotGroupId(
      const GroupId& group_id) const = 0;

  // Sets the ordered list of RobotCollectionsEntityIds associated with the
  // specified GroupId, overwriting any previous value. An empty list will
  // disable the GroupId.
  ABSL_DEPRECATED(
      "This function should only be called by EntityRobots. Instead, use "
      "World-level functions to interact directly with robots.")
  virtual void SetRobotCollectionsEntityIdsForRobotGroupId(
      const GroupId& group_id,
      const std::vector<RobotCollectionsEntityId>& robot_ids) = 0;

  // Returns the base link that is the ancestor of all other robots' base links
  // in the group. Returns an error if the robots in the group do not share a
  // common base link.
  ABSL_DEPRECATED(
      "Prefer to use GetBaseLink() which operates on "
      "RobotCollectionsEntityIds.")
  virtual absl::StatusOr<AttachmentEntityId> GetBaseLinkForRobotGroupId(
      const GroupId& group_id) const = 0;

  // Returns the most distal entity (from the world root) of the robots in the
  // specified group. Returns an error if the robots in the group are not
  // attached to each other.
  ABSL_DEPRECATED(
      "This function should only be called by EntityRobots. Instead, use "
      "World-level functions to interact directly with robots.")
  virtual absl::StatusOr<AttachmentEntityId> GetFinalEntityForRobotGroupId(
      const GroupId& group_id) const = 0;

  // Sets a tip for a robot GroupId, which enables
  // GetCartesianKinematicViewForRobotGroupId().
  ABSL_DEPRECATED(
      "This function should only be called by EntityRobots. Instead, use "
      "World-level functions to interact directly with robots.")
  virtual absl::Status SetTipIdForRobotGroupId(
      const GroupId& group_id, PhysicalEntityId tip_object_id) = 0;

  // Returns a CartesianKinematicView for the specified robot GroupId. This
  // function is intended to match the behavior of
  // Robots::GetCartesianKinematicView(), meaning the returned view does not
  // necessarily represent all of the DoFs in the robot group.
  ABSL_DEPRECATED(
      "GroupIds are deprecated. Prefer to use World functions that take "
      "RobotCollectionsEntityId arguments.")
  virtual absl::StatusOr<std::unique_ptr<CartesianKinematicView>>
  GetCartesianKinematicViewForRobotGroupId(const GroupId& group_id) = 0;
  ABSL_DEPRECATED(
      "GroupIds are deprecated. Prefer to use World functions that take "
      "RobotCollectionsEntityId arguments.")
  virtual absl::StatusOr<std::unique_ptr<const CartesianKinematicView>>
  GetCartesianKinematicViewForRobotGroupId(const GroupId& group_id) const = 0;

  // Sets the DofId for an EntityId.
  ABSL_DEPRECATED(
      "This function should only be called by EntityRobots. Use JointEntityIds "
      "instead of DofIds.")
  virtual void SetDofIdForEntityId(DofId dof_id, EntityId entity_id) = 0;

  // Returns an ordered list of DofIds for the specified robot GroupId.
  ABSL_DEPRECATED(
      "This function should only be called by EntityRobots. Instead, use "
      "World-level functions to interact directly with robots.")
  virtual absl::StatusOr<std::vector<DofId>> GetDofIdsForRobotGroupId(
      const GroupId& group_id) const = 0;

  // Helper function to simplify the process of reading collection members and
  // validating that they conform to a specific type.
  //
  // This function should only be called from within
  // //intrinsic/world/aspects. Other callers should use
  // World::ValidateCollectionMembers() instead.
  template <typename... ComponentTypes>
  static absl::StatusOr<
      std::vector<world_entity_details::TypedResult<ComponentTypes...>>>
  ValidateCollectionMembers(
      const entity_aspect_world_details::EntityWorld& world,
      CollectionsEntityId collections_id,
      intrinsic_proto::world::CollectionsComponent::CollectionType type);

  // Helper function to simplify the process of searching for a collection
  // parent and validating it conforms to a specific type.
  //
  // This function should only be called from within
  // //intrinsic/world/aspects. Other callers should use
  // World::ValidateCollectionParentAmongTypes() instead.
  template <typename... ComponentTypes>
  static absl::StatusOr<world_entity_details::TypedResult<ComponentTypes...>>
  ValidateCollectionParentAmongTypes(
      const entity_aspect_world_details::EntityWorld& world,
      CollectionsMemberEntityId collections_member_id,
      const WorldHashSet<
          intrinsic_proto::world::CollectionsComponent::CollectionType>& types);
};

template <typename... ComponentTypes>
absl::StatusOr<
    std::vector<world_entity_details::TypedResult<ComponentTypes...>>>
EntityWorld::ValidateCollectionMembers(
    const entity_aspect_world_details::EntityWorld& world,
    CollectionsEntityId collections_id,
    intrinsic_proto::world::CollectionsComponent::CollectionType type) {
  INTR_ASSIGN_OR_RETURN(const WorldEntity* ent,
                        world.GetEntityById(collections_id));
  INTR_ASSIGN_OR_RETURN(const CollectionsComponent* collections,
                        ent->GetComponent<CollectionsComponent>());
  const std::vector<CollectionsMemberEntityId>& members =
      collections->GetCollectionMembers(type);
  std::vector<world_entity_details::TypedResult<ComponentTypes...>> ret;
  for (CollectionsMemberEntityId member : members) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* member_ent,
                          world.GetEntityById(member));
    if (!member_ent->ValidateEntity<ComponentTypes...>().ok()) {
      return intrinsic::InternalErrorBuilder()
             << "could not validate member of collection type "
             << intrinsic_proto::world::CollectionsComponent::
                    CollectionType_Name(type);
    }
    ret.emplace_back(member.value());
  }
  return ret;
}

template <typename... ComponentTypes>
absl::StatusOr<world_entity_details::TypedResult<ComponentTypes...>>
EntityWorld::ValidateCollectionParentAmongTypes(
    const entity_aspect_world_details::EntityWorld& world,
    CollectionsMemberEntityId collections_member_id,
    const WorldHashSet<
        intrinsic_proto::world::CollectionsComponent::CollectionType>& types) {
  INTR_ASSIGN_OR_RETURN(const WorldEntity* ent,
                        world.GetEntityById(collections_member_id));
  INTR_ASSIGN_OR_RETURN(const auto* collections_member,
                        ent->GetComponent<CollectionsMemberComponent>());
  INTR_ASSIGN_OR_RETURN(
      CollectionsEntityId parent_candidate,
      collections_member->FindParentCollectionAmongTypes(types));
  INTR_ASSIGN_OR_RETURN(const WorldEntity* parent_candidate_ent,
                        world.GetEntityById(parent_candidate));
  INTR_RETURN_IF_ERROR(
      parent_candidate_ent->ValidateEntity<ComponentTypes...>())
      << "while trying to validate collection parent of entity with ID "
      << collections_member_id.value();
  return world_entity_details::TypedResult<ComponentTypes...>(
      parent_candidate.value());
}

template <typename... ComponentTypes>
std::vector<world_entity_details::TypedResult<ComponentTypes...>>
EntityWorld::GetTypedEntityIds() const {
  std::vector<world_entity_details::TypedResult<ComponentTypes...>> results;
  for (const auto& entity_id : GetEntityIds()) {
    ASSIGN_OR_DIE(auto entity, GetEntityById(entity_id));
    if (entity->IsEntityValid<ComponentTypes...>()) {
      results.emplace_back(entity_id);
    }
  }
  return results;
}

}  // namespace entity_aspect_world_details
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_ASPECTS_ENTITY_WORLD_INTERFACE_H_
