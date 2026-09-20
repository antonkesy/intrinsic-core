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

#include "intrinsic/world/kinematics_builder.h"

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/coordinate_frame.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/aggregate_type.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/component/physics_component.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"

using ::intrinsic::entity_aspect_world_details::EntityWorld;

namespace intrinsic {
namespace kinematics {

namespace {

constexpr const char* kDefaultSolverKey = "kinematic_chain";

absl::StatusOr<AttachmentEntityId> GetParentEntityId(
    const EntityWorld& world, const WorldEntity& entity) {
  INTR_ASSIGN_OR_RETURN(auto* entity_attach,
                        entity.GetComponent<AttachmentComponent>());
  return entity_attach->GetParentId();
}

absl::StatusOr<const AttachmentComponent*> GetAttachmentComponent(
    const EntityWorld& world, EntityId id) {
  INTR_ASSIGN_OR_RETURN(const auto* entity, world.GetEntityById(id),
                        _ << "When looking up id " << id.value());
  return entity->GetComponent<AttachmentComponent>();
}

// Given a list of candidates, returns the first one that is an ancestor to
// `element'. Returns an error if no candidate satisfies the condition.
template <class Container>
absl::StatusOr<AttachmentEntityId> FindAncestor(const EntityWorld& world,
                                                const Container& candidate_ids,
                                                AttachmentEntityId element) {
  for (const auto& id : candidate_ids) {
    INTR_ASSIGN_OR_RETURN(auto common_ancestor,
                          world.FindCommonAncestor(id, element));
    if (common_ancestor != id) {
      continue;
    }
    // TODO(jeanfrancoisd): Consider checking the error case of more than one
    // common ancestor.
    return id;
  }
  return intrinsic::NotFoundErrorBuilder()
         << "Could not find an entity that is on "
            "the same branch of entity with ID "
         << element.value();
}

absl::StatusOr<ElementId> AddKinematicLinkWithPose(const EntityWorld& world,
                                                   const LinkEntityId& link_id,
                                                   ElementId parent_element_id,
                                                   Skeleton* skeleton,
                                                   const Pose3d& pose) {
  INTR_ASSIGN_OR_RETURN(const auto* link_ent, world.GetEntityById(link_id),
                        _ << "When looking up link_id " << link_id.value());

  std::string link_name = CreateKinematicElementName(link_id, *link_ent);

  INTR_ASSIGN_OR_RETURN(const PhysicsComponent* link_phys,
                        link_ent->GetComponent<PhysicsComponent>());
  Link::Parameters params;
  params.mass = link_phys->GetMassKg();
  params.center_of_gravity = link_phys->GetThisTCenterOfMass();
  params.inertia = link_phys->GetInertiaMatrix();

  // Ignore the correct transform and use identity instead.
  return skeleton->CreateLink(link_name, params, parent_element_id, pose);
}

absl::StatusOr<ElementId> AddKinematicLink(const EntityWorld& world,
                                           const LinkEntityId& link_id,
                                           ElementId parent_element_id,
                                           Skeleton* skeleton) {
  Pose3d parent_t_link;
  if (parent_element_id != kInvalidElementId) {
    INTR_ASSIGN_OR_RETURN(const auto* link_ent, world.GetEntityById(link_id),
                          _ << "When looking up link_id " << link_id.value());
    INTR_ASSIGN_OR_RETURN(auto parent_id, GetParentEntityId(world, *link_ent));
    parent_t_link = world.GetTransform(parent_id, link_id);
  }
  return AddKinematicLinkWithPose(world, link_id, parent_element_id, skeleton,
                                  parent_t_link);
}

bool ArePartsOfSameRobot(const EntityWorld& world, EntityId a, EntityId b) {
  auto collection_a_or =
      EntityWorld::ValidateCollectionParentAmongTypes<CollectionsEntityId>(
          world, CollectionsMemberEntityId(a.value()),
          CollectionsComponent::RobotPartTypes());
  if (!collection_a_or.ok()) {
    return false;
  }
  auto collection_b_or =
      EntityWorld::ValidateCollectionParentAmongTypes<CollectionsEntityId>(
          world, CollectionsMemberEntityId(b.value()),
          CollectionsComponent::RobotPartTypes());
  if (!collection_b_or.ok()) {
    return false;
  }
  return collection_a_or.value() == collection_b_or.value();
}

absl::StatusOr<ElementId> AddFixedJoint(
    const EntityWorld& world, const AttachmentEntityId& child_world_id,
    const AttachmentEntityId& parent_world_id,
    const ElementId parent_element_id, Skeleton* skeleton) {
  INTR_ASSIGN_OR_RETURN(const auto* child_entity,
                        world.GetEntityById(child_world_id),
                        _ << "When looking up id " << child_world_id.value());
  std::string joint_name = absl::StrFormat(
      "base attachment of %s",
      CreateKinematicElementName(child_world_id, *child_entity));
  Joint::Parameters params;
  params.type = Joint::FIXED;

  Pose3d parent_t_joint = world.GetTransform(parent_world_id, child_world_id);

  return skeleton->CreateJoint(joint_name, params, parent_element_id,
                               parent_t_joint);
}

absl::StatusOr<ElementId> AddKinematicJoint(const EntityWorld& world,
                                            const JointEntityId& joint_id,
                                            ElementId parent_element_id,
                                            Skeleton* skeleton) {
  if (parent_element_id == kInvalidElementId) {
    return absl::InvalidArgumentError(
        "Joint can't be a the root of a skeleton");
  }

  INTR_ASSIGN_OR_RETURN(const auto* joint_ent, world.GetEntityById(joint_id),
                        _ << "when looking up joint_id " << joint_id.value());

  std::string joint_name = CreateKinematicElementName(joint_id, *joint_ent);

  INTR_ASSIGN_OR_RETURN(const KinematicsComponent* joint_kin,
                        joint_ent->GetComponent<KinematicsComponent>());
  Joint::Parameters params;
  switch (joint_kin->GetMotionType()) {
    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_REVOLUTE:
      params.type = Joint::REVOLUTE;
      break;

    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_PRISMATIC:
      params.type = Joint::PRISMATIC;
      break;

    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_FIXED:
      params.type = Joint::FIXED;
      break;

    default:
      return intrinsic::UnimplementedErrorBuilder()
             << "Unhandled joint motion type " << joint_kin->GetMotionType();
  }
  params.axis = joint_kin->GetAxis();
  // TODO (b/288158648) : Expose Soft Limits in skeleton to be used for
  // planning.
  std::tie(params.system_limits.position.lower,
           params.system_limits.position.upper) =
      joint_kin->GetApplicationRawValueFixedLimits();
  params.system_limits.velocity = joint_kin->GetApplicationVelocityLimit();
  params.system_limits.acceleration =
      joint_kin->GetApplicationAccelerationLimit();
  params.system_limits.effort = joint_kin->GetApplicationEffortLimit();
  params.system_limits.jerk = joint_kin->GetApplicationJerkLimit();

  std::tie(params.soft_limits.position.lower,
           params.soft_limits.position.upper) =
      joint_kin->GetApplicationRawValueFixedLimits();
  params.soft_limits.velocity = joint_kin->GetApplicationVelocityLimit();
  params.soft_limits.acceleration =
      joint_kin->GetApplicationAccelerationLimit();
  params.soft_limits.effort = joint_kin->GetApplicationEffortLimit();
  params.soft_limits.jerk = joint_kin->GetApplicationJerkLimit();

  INTR_ASSIGN_OR_RETURN(auto parent_link_id,
                        GetParentEntityId(world, *joint_ent));
  INTR_ASSIGN_OR_RETURN(const auto* parent_link_attach,
                        GetAttachmentComponent(world, parent_link_id));

  Pose3d parent_t_joint;
  if (!ArePartsOfSameRobot(world, joint_id, parent_link_id)) {
    parent_t_joint = parent_link_attach->GetParentTThis();
  }
  parent_t_joint *= joint_kin->GetParentTInboard();

  return skeleton->CreateJoint(joint_name, params, parent_element_id,
                               parent_t_joint);
}

// Add the solver keys of a robot entity to the skeleton.
absl::Status AddKinematicSolvers(
    const WorldEntity* robot_ent,
    const WorldHashMap<EntityId, ElementId>& world_id_to_skeleton_id,
    const ElementId& base_link_id, Skeleton* skeleton) {
  INTR_ASSIGN_OR_RETURN(const RobotComponent* robot_component,
                        robot_ent->GetComponent<RobotComponent>());
  for (const auto& [frame_base_id, frame_tip_id] :
       robot_component->GetSolvableFrames()) {
    // Get solver key
    INTR_ASSIGN_OR_RETURN(
        std::string solver_key,
        robot_component->GetSolverKeyForFrames(frame_base_id, frame_tip_id));

    // frame_base_id can be kInvalidEntityId for two reasons:
    // (1) to define the base frame of the robot component: 0 -> x
    // (2) to define a default solver: 0 -> 0. In this case the tip id is also
    //     kInvalidEntityId
    auto element_base_id = base_link_id;
    if (frame_base_id != kInvalidEntityId) {
      auto element_base_id_it = world_id_to_skeleton_id.find(frame_base_id);
      if (element_base_id_it == world_id_to_skeleton_id.end()) {
        // Currently the builder ignores entities that are not joints or
        // links. Therefore, we skip those solvers and  do not throw an
        // error. b/188701575
        LOG(WARNING)
            << "Can't find entity id " << frame_base_id.value()
            << " in skeleton. Skipping adding ik solver key for frames "
            << frame_base_id.value() << " to " << frame_tip_id.value();
        continue;
      }
      element_base_id = element_base_id_it->second;
    }

    // Case ([0|y],x) -> solver_key
    if (frame_tip_id != kInvalidEntityId) {
      // Valid entry. Add as is to skeleton solver map.
      auto element_tip_id_it = world_id_to_skeleton_id.find(frame_tip_id);
      if (element_tip_id_it == world_id_to_skeleton_id.end()) {
        // Currently the builder ignores entities that are not joints or
        // links. Therefore, we skip those solvers and  do not throw an
        // error. b/188701575
        LOG(WARNING)
            << "Can't find entity id " << frame_tip_id.value()
            << " in skeleton. Skipping adding ik solver key for frames "
            << frame_base_id.value() << "to " << frame_tip_id.value();
        continue;
      }
      INTR_RETURN_IF_ERROR(skeleton->SetSolverKey(
          element_base_id, element_tip_id_it->second, solver_key));
      continue;
    }

    // Case (0, 0) -> solver_key
    if (frame_tip_id == kInvalidEntityId && frame_base_id == kInvalidEntityId) {
      // Due to b/188547702 we have (0,0) combinations for from and to id. In
      // case of the kinematic chain, we will skip and process it at a later
      // time. All other cases use base and tip frame as currently done in the
      // code base.
      if (solver_key == kDefaultSolverKey) {
        continue;
      }
      if (skeleton->HasOneTip()) {
        INTR_RETURN_IF_ERROR(skeleton->SetSolverKey(
            element_base_id, skeleton->GetTipIds().front(), solver_key));
        continue;
      }
    }

    // Remaining cases have unsupported behavior.
    return intrinsic::InternalErrorBuilder()
           << "Unsupported solver mapping configuration. Base id "
           << frame_base_id.value() << "to " << frame_tip_id.value();
  }
  return absl::OkStatus();
}

// Adds a kinematic chain solver as default solver if no other solver is defined
// for the pair.
absl::Status AddKinematicChainSolverForSkeleton(Skeleton* skeleton) {
  auto link_ids = skeleton->GetAllLinkIds();
  for (int i = 0; i < link_ids.size(); ++i) {
    for (int j = i + 1; j < link_ids.size(); ++j) {
      if (!skeleton->GetAllSolverKeys(link_ids[i], link_ids[j]).empty()) {
        //  Do not add a default solver if one already exist.
        continue;
      }
      INTR_RETURN_IF_ERROR(
          skeleton->SetSolverKey(link_ids[i], link_ids[j], kDefaultSolverKey));
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<Skeleton>> BuildChainSkeletonImpl(
    const EntityWorld& world, AttachmentEntityId id1, AttachmentEntityId id2) {
  // TODO(jeanfrancoisd): Review why we need to look into the robots.

  AttachmentEntityId base;
  AttachmentEntityId tip;
  std::vector<RobotCollectionsEntityId> robot_ids;

  INTR_ASSIGN_OR_RETURN(const AttachmentEntityId ancestor,
                        world.FindCommonAncestor(id1, id2));
  if (id1 == ancestor) {
    base = id1;
    tip = id2;

    // Robot IDs are in the order they occur in the chain from base --> tip.
    INTR_ASSIGN_OR_RETURN(robot_ids, world.GetRobotIdsInChain(base, tip));
  } else if (id2 == ancestor) {
    base = id2;
    tip = id1;

    // Robot IDs are in the order they occur in the chain from base --> tip.
    INTR_ASSIGN_OR_RETURN(robot_ids, world.GetRobotIdsInChain(base, tip));
  } else {
    // Robot IDs are in the order they occur in the chain from base --> tip.
    INTR_ASSIGN_OR_RETURN(
        const std::vector<RobotCollectionsEntityId> robot_ids1,
        world.GetRobotIdsInChain(ancestor, id1));
    INTR_ASSIGN_OR_RETURN(
        const std::vector<RobotCollectionsEntityId> robot_ids2,
        world.GetRobotIdsInChain(ancestor, id2));

    if (!robot_ids1.empty() && !robot_ids2.empty()) {
      return absl::UnimplementedError(
          "BuildChainSkeleton does not yet support chains of robots in which "
          "any robot is not a descendant of the preceding robot in the chain.");
    } else if (!robot_ids1.empty()) {
      robot_ids = robot_ids1;
      base = id2;
      tip = id1;
    } else {
      robot_ids = robot_ids2;
      base = id1;
      tip = id2;
    }
  }

  auto skeleton = std::make_unique<Skeleton>(
      absl::StrFormat("skeleton_id_%d_to_id_%d", base.value(), tip.value()));

  if (robot_ids.empty()) {
    return absl::InvalidArgumentError(
        "BuildChainSkeleton does not support a chain "
        "that is not associated with a robot. Please check inputs.");
  }
  bool is_first_robot = true;
  EntityId current_tip = kInvalidEntityId;
  ElementId current_parent_id = kInvalidElementId;
  WorldHashMap<EntityId, ElementId> world_id_to_skeleton_id;
  AttachmentEntityId last_robot_tip_id = kRootEntityId;

  for (const auto& robot_id : robot_ids) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* robot_ent,
                          world.GetEntityById(robot_id));
    INTR_ASSIGN_OR_RETURN(AttachmentEntityId current_base_id,
                          world.GetBaseLink(robot_id));

    if (current_parent_id != kInvalidElementId) {
      if (current_tip == kInvalidEntityId) {
        return absl::InternalError(
            "Current tip shouldn't be invalid at this point.");
      }
      if (last_robot_tip_id == kRootEntityId) {
        return absl::InternalError(
            "Something went wrong during building the kinematic structure for "
            "the problem. Could not identify tip of previous robot in the "
            "chain");
      }
      // When we finished processing the previous robot, current_parent_id was
      // assigned to the tip of the robot.
      INTR_ASSIGN_OR_RETURN(
          current_parent_id,
          AddFixedJoint(world, /*child_world_id=*/current_base_id,
                        /*parent_world_id=*/last_robot_tip_id,
                        /*parent_element_id=*/current_parent_id,
                        skeleton.get()));
      world_id_to_skeleton_id[current_base_id] = current_parent_id;
    } else if (current_base_id != base) {
      // Create a link for the `base` entity.
      INTR_ASSIGN_OR_RETURN(const WorldEntity* base_entity,
                            world.GetEntityById(base));
      INTR_ASSIGN_OR_RETURN(
          current_parent_id,
          skeleton->CreateLink(CreateKinematicElementName(base, *base_entity),
                               Link::Parameters()));
      world_id_to_skeleton_id[base] = current_parent_id;

      // Add a fixed joint from the `base` to the base of the first robot.
      INTR_ASSIGN_OR_RETURN(
          current_parent_id,
          skeleton->CreateJoint("fixed joint to first robot base link",
                                Joint::Parameters{.type = Joint::FIXED},
                                current_parent_id,
                                world.GetTransform(base, current_base_id)));

      world_id_to_skeleton_id[current_base_id] = current_parent_id;

      // The fixed joint behaves like a preceding robot.
      is_first_robot = false;
    }

    INTR_ASSIGN_OR_RETURN(auto robot_tips,
                          world.GetFinalEntitiesOfRobot(robot_id));

    // When the robot has more than one tip, we need to find the one that is on
    // the branch of the desired end of the kinematic chain.
    INTR_ASSIGN_OR_RETURN(current_tip, FindAncestor(world, robot_tips, tip));

    INTR_ASSIGN_OR_RETURN(auto link_ids,
                          EntityWorld::ValidateCollectionMembers<LinkEntityId>(
                              world, robot_id, CollectionsComponent::kLinks));

    INTR_ASSIGN_OR_RETURN(auto joint_ids,
                          EntityWorld::ValidateCollectionMembers<JointEntityId>(
                              world, robot_id, CollectionsComponent::kJoints));

    // Add all links and joints to the skeleton. Each iteration of this `while`
    // either adds a joint or a link and then updates `current_base_id` to
    // contain the ID of the next joint or link in the kinematic chain.
    ElementId base_link_element_id = kInvalidElementId;
    while (true) {
      auto link_iter =
          std::find(link_ids.begin(), link_ids.end(), current_base_id);
      if (link_iter != link_ids.end()) {
        if (!is_first_robot && link_iter == link_ids.begin()) {
          // Fixes the removal of the fixed joint in
          // https://github.com/intrinsic-ai/intrinsic-core/blob/main/google3/intrinsic/world/conversion/sdf/world_from_sdf.cc#L1657
          // when recreating the fixed joint. In most cases the link transform
          // is identity and the legacy code for kinematics does ignore link
          // transformations. Therefore, this transform is moved to the fixed
          // joint.
          INTR_ASSIGN_OR_RETURN(
              current_parent_id,
              AddKinematicLinkWithPose(world, *link_iter, current_parent_id,
                                       skeleton.get(), Pose3d::Identity()));
        } else {
          INTR_ASSIGN_OR_RETURN(
              current_parent_id,
              AddKinematicLink(world, *link_iter, current_parent_id,
                               skeleton.get()));
        }

        if (link_iter == link_ids.begin()) {
          base_link_element_id = current_parent_id;
        }
        world_id_to_skeleton_id[link_iter->id] = current_parent_id;
      } else {
        auto joint_iter =
            std::find(joint_ids.begin(), joint_ids.end(), current_base_id);
        if (joint_iter != joint_ids.end()) {
          INTR_ASSIGN_OR_RETURN(
              current_parent_id,
              AddKinematicJoint(world, *joint_iter, current_parent_id,
                                skeleton.get()));
          world_id_to_skeleton_id[joint_iter->id] = current_parent_id;
        } else {
          // b/188701575
          LOG(WARNING) << "Entity id=" << current_base_id
                       << " is neither a joint nor a link";
        }
      }

      // This equality is satisfied if no link or joint was added in this
      // iteration, which means no links or joints remain to be added for
      // this robot.
      if (current_base_id == current_tip) {
        last_robot_tip_id = current_base_id;
        break;
      }

      // Advance the `current_base_id` to the next link or joint that is
      // on the chain toward the `tip`.
      auto children = world.GetChildrenOf(current_base_id);
      INTR_ASSIGN_OR_RETURN(current_base_id,
                            FindAncestor(world, children, tip));
    }

    // Add solver keys for specified frames, except for kinematic chain solver
    // that will be set for all link-link pairs below.
    if (base_link_element_id != kInvalidElementId) {
      INTR_RETURN_IF_ERROR(
          AddKinematicSolvers(robot_ent, world_id_to_skeleton_id,
                              base_link_element_id, skeleton.get()));
    }

    is_first_robot = false;
  }

  // If the tip of the last robot is not the given `tip`, then add a coordinate
  // frame to the skeleton to include the `tip`.
  if (current_tip != tip) {
    const Pose3d current_tip_t_tip =
        world.GetTransform(AttachmentEntityId(current_tip), tip);
    INTR_ASSIGN_OR_RETURN(const WorldEntity* tip_entity,
                          world.GetEntityById(tip));

    INTR_ASSIGN_OR_RETURN(
        current_parent_id,
        skeleton->AddCoordinateFrame(
            std::make_unique<CoordinateFrame>(
                CreateKinematicElementName(tip, *tip_entity)),
            world_id_to_skeleton_id[current_tip], current_tip_t_tip));
    world_id_to_skeleton_id[tip] = current_parent_id;
  }

  // Add default solver for all link-link pairs in the skeleton.
  INTR_RETURN_IF_ERROR(AddKinematicChainSolverForSkeleton(skeleton.get()));

  return std::move(skeleton);
}
}  // namespace

}  // namespace kinematics

absl::StatusOr<std::unique_ptr<kinematics::Skeleton>> BuildChainSkeleton(
    const EntityWorld& world, AttachmentEntityId id1, AttachmentEntityId id2) {
  return kinematics::BuildChainSkeletonImpl(world, id1, id2);
}

std::string CreateKinematicElementName(EntityId id, const WorldEntity& entity) {
  std::string element_name = entity.GetLocalName();
  if (element_name.empty()) {
    return absl::StrFormat("ID %i", id.value());
  }
  return entity.GetLocalName();
}

}  // namespace intrinsic
