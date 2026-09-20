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

#include "intrinsic/world/build_tree_skeleton_from_world.h"

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/component/physics_component.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/util/walk_attachment_tree.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace {

// Given an `entity_id` in `world`, this returns the kinematics::ElementId that
// belongs to the same entity in `skeleton`.
//
// This assumes that entities in `skeleton` were named using the
// GetUniqueName() function below.
absl::StatusOr<kinematics::ElementId> LookupSkeletonElementId(
    const World& world, AttachmentEntityId root,
    const kinematics::Skeleton& skeleton, EntityId entity_id) {
  std::string element_name = GetUniqueName(world, root, entity_id);
  icon::RealtimeStatusOr<kinematics::ElementId> element_id =
      skeleton.FindElementIdByName(element_name);
  if (!element_id.ok()) {
    return element_id.status();
  }
  return *element_id;
}

// Adds a joint to `skeleton` that corresponds to `entity` (i.e. the entity
// with `id` in `world`).
//
// Requirements:
// * `entity` must be a JointEntity (in particular, it must have a
//   KinematicsComponent and AttachmentComponent)
// * `id` must be the ID of `entity` in `world`
// * `skeleton` must already have a link that corresponds to `entity`'s parent
//   link.
//
// Returns an error if any of the requirements is violated, and forwards other
// errors that may come up in the process (notably, if `world` is
// inconsistent).
// Returns OkStatus on success. In this case, a new joint is added to
// `skeleton`.
absl::Status AddJoint(const World& world, AttachmentEntityId root,
                      AttachmentEntityId id, const WorldEntity& entity,
                      kinematics::Skeleton& skeleton) {
  INTR_RETURN_IF_ERROR(entity.ValidateEntity<JointEntityId>());
  INTR_ASSIGN_OR_RETURN(const KinematicsComponent* joint_kin,
                        entity.GetComponent<KinematicsComponent>());
  // Populate joint parameters.
  kinematics::Joint::Parameters params;
  switch (joint_kin->GetMotionType()) {
    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_REVOLUTE:
      params.type = kinematics::Joint::REVOLUTE;
      break;
    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_PRISMATIC:
      params.type = kinematics::Joint::PRISMATIC;
      break;
    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_FIXED:
      params.type = kinematics::Joint::FIXED;
      break;
    default:
      return intrinsic::UnimplementedErrorBuilder()
             << "Unhandled joint motion type " << joint_kin->GetMotionType();
  }
  params.axis = joint_kin->GetAxis();
  std::tie(params.system_limits.position.lower,
           params.system_limits.position.upper) =
      joint_kin->GetSystemRawValueFixedLimits();
  params.system_limits.velocity = joint_kin->GetSystemVelocityLimit();
  params.system_limits.acceleration = joint_kin->GetSystemAccelerationLimit();
  params.system_limits.effort = joint_kin->GetSystemEffortLimit();
  params.system_limits.jerk = joint_kin->GetSystemJerkLimit();

  std::tie(params.soft_limits.position.lower,
           params.soft_limits.position.upper) =
      joint_kin->GetApplicationRawValueFixedLimits();
  params.soft_limits.velocity = joint_kin->GetApplicationVelocityLimit();
  params.soft_limits.acceleration =
      joint_kin->GetApplicationAccelerationLimit();
  params.soft_limits.effort = joint_kin->GetApplicationEffortLimit();
  params.soft_limits.jerk = joint_kin->GetApplicationJerkLimit();

  INTR_ASSIGN_OR_RETURN(const auto* attachment_component,
                        entity.GetComponent<AttachmentComponent>());
  AttachmentEntityId parent_id = attachment_component->GetParentId();
  if (parent_id == kInvalidEntityId) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Joint '", GetUniqueName(world, root, id), "' has invalid parent"));
  }
  // Look up (skeleton) ElementId for (World) `parent_id`
  INTR_ASSIGN_OR_RETURN(
      kinematics::ElementId parent_element_id,
      LookupSkeletonElementId(world, root, skeleton, parent_id));
  return skeleton
      .CreateJoint(GetUniqueName(world, root, id), params, parent_element_id,
                   joint_kin->GetParentTInboard())
      .status();
}

// Adds a link to `skeleton` that corresponds to `entity` (i.e. the entity
// with `id` in `world`).
//
// Requirements:
// * `entity` must be a LinkEntity (in particular, it must have an
//   AttachmentComponent)
// * `id` must be the ID of `entity` in `world`
// * `skeleton` must already have a link that corresponds to `entity`'s parent
//   link.
//
// Returns an error if any of the requirements is violated, and forwards other
// errors that may come up in the process (notably, if `world` is
// inconsistent).
// Returns OkStatus on success. In this case, a new link is added to `skeleton`.
absl::Status AddLink(const World& world, AttachmentEntityId root,
                     AttachmentEntityId id, const WorldEntity& entity,
                     kinematics::Skeleton& skeleton) {
  INTR_RETURN_IF_ERROR(entity.ValidateEntity<PhysicalEntityId>());
  INTR_RETURN_IF_ERROR(entity.ValidateEntity<PhysicsEntityId>());
  INTR_ASSIGN_OR_RETURN(const auto* physics_component,
                        entity.GetComponent<PhysicsComponent>());
  kinematics::Link::Parameters link_parameters = {
      .center_of_gravity = physics_component->GetThisTCenterOfMass(),
      .inertia = physics_component->GetInertiaMatrix(),
      .mass = physics_component->GetMassKg()};
  if (skeleton.GetAllElementIds().empty()) {
    // This is the root link, so just add it and discard any parent_t_this
    // transformation it may have.
    return skeleton.CreateLink(GetUniqueName(world, root, id), link_parameters)
        .status();
  }
  INTR_ASSIGN_OR_RETURN(const auto* attachment_component,
                        entity.GetComponent<AttachmentComponent>());
  INTR_ASSIGN_OR_RETURN(
      kinematics::ElementId parent_element_id,
      LookupSkeletonElementId(world, root, skeleton,
                              attachment_component->GetParentId()));
  return skeleton
      .CreateLink(GetUniqueName(world, root, id), link_parameters,
                  parent_element_id, attachment_component->GetParentTThis())
      .status();
}

// Adds a coordinate frame to `skeleton` that corresponds to `entity` (i.e. the
// entity with `id` in `world`).
//
// Requirements:
// * `entity` must have an AttachmentComponent
// * `id` must be the ID of `entity` in `world`
// * `skeleton` must already have a link that corresponds to `entity`'s parent
//   link.
//
// Returns an error if any of the requirements is violated, and forwards other
// errors that may come up in the process (notably, if `world` is
// inconsistent).
// Returns OkStatus on success. In this case, a new link is added to `skeleton`.
absl::Status AddCoordinateFrame(const World& world, AttachmentEntityId root,
                                AttachmentEntityId id,
                                const WorldEntity& entity,
                                kinematics::Skeleton& skeleton) {
  std::string frame_name = GetUniqueName(world, root, id);
  if (skeleton.GetAllElementIds().empty()) {
    // This is the root frame, so just add it and discard any parent_t_this
    // transformation it may have.
    return skeleton.CreateCoordinateFrame(frame_name).status();
  }
  INTR_ASSIGN_OR_RETURN(const auto* attachment_component,
                        entity.GetComponent<AttachmentComponent>());
  INTR_ASSIGN_OR_RETURN(
      kinematics::ElementId parent_element_id,
      LookupSkeletonElementId(world, root, skeleton,
                              attachment_component->GetParentId()));
  return skeleton
      .CreateCoordinateFrame(frame_name, parent_element_id,
                             attachment_component->GetParentTThis())
      .status();
}

// As this class walks the AttachmentEntity tree (see
// intrinsic/world/util/walk_attachment_tree.h for details), it
// builds up a Skeleton that represents the same kinematic model.
class SkeletonWalker final : public intrinsic::AttachmentTreeHandler {
 public:
  // Creates a new SkeletonWalker. The walker will look at the subtree rooted at
  // `root_id` and generate a skeleton named `skeleton_name`.
  // The Skeleton will have a joint/link for each corresponding object in that
  // subtree.
  SkeletonWalker(const World& world, AttachmentEntityId root_id,
                 absl::string_view skeleton_name)
      : world_(world),
        requested_subtree_root_id_(root_id),
        skeleton_(std::make_unique<kinematics::Skeleton>(skeleton_name)) {}

  // This does the bulk of the work, adding Joint/Link elements to the skeleton.
  absl::Status OnStartEntity(AttachmentEntityId handle,
                             const WorldEntity& entity) override {
    if (handle == requested_subtree_root_id_) {
      processing_requested_subtree_ = true;
    } else if (!processing_requested_subtree_) {
      return absl::OkStatus();
    }
    if (handle == kRootEntityId) {
      return skeleton_
          ->CreateCoordinateFrame(GetUniqueName(world_, kRootEntityId, handle))
          .status();
    }
    if (entity.IsEntityValid<JointEntityId>()) {
      return AddJoint(world_, requested_subtree_root_id_, handle, entity,
                      *skeleton_);
    }
    if (entity.IsEntityValid<PhysicalEntityId>() &&
        entity.IsEntityValid<PhysicsEntityId>()) {
      return AddLink(world_, requested_subtree_root_id_, handle, entity,
                     *skeleton_);
    }
    return AddCoordinateFrame(world_, requested_subtree_root_id_, handle,
                              entity, *skeleton_);
  }

  // When an entity is finished, we check the World for any IK solver
  // declarations that use `entity` as their base link.
  // For any solvers we find, we determine the corresponding Skeleton base and
  // tip ElementIds, and set the solver key in our Skeleton accordingly.
  absl::Status OnEndEntity(AttachmentEntityId handle,
                           const WorldEntity& entity) override {
    if (!processing_requested_subtree_) {
      return absl::OkStatus();
    }
    // All children of `entity` have been processed and added to the
    // `skeleton_`. Now check if `entity` is the base link of any
    // RobotCollections, and if so, apply the solver key for that collection
    // to all child links and joints.
    auto robot_collection_ids =
        world_.GetTypedEntityIds<RobotCollectionsEntityId>();
    for (const auto& robot_collection_id : robot_collection_ids) {
      INTR_ASSIGN_OR_RETURN(LinkEntityId base_link_id,
                            world_.GetBaseLink(robot_collection_id));
      if (base_link_id.id != handle.id) {
        continue;
      }
      INTR_ASSIGN_OR_RETURN(const WorldEntity* robot_collection_entity,
                            world_.GetEntityById(robot_collection_id));
      INTR_ASSIGN_OR_RETURN(
          const RobotComponent* robot_component,
          robot_collection_entity->GetComponent<RobotComponent>());

      // Since we know that `base_link_id` == `handle`, `entity` must be the
      // corresponding World entity, so give it a helpful local name.
      INTR_ASSIGN_OR_RETURN(
          const kinematics::ElementId base_link_element_id,
          LookupSkeletonElementId(world_, requested_subtree_root_id_,
                                  *skeleton_, base_link_id));

      for (const auto& [frame_base_id, frame_tip_id] :
           robot_component->GetSolvableFrames()) {
        // Get solver key
        INTR_ASSIGN_OR_RETURN(std::string solver_key,
                              robot_component->GetSolverKeyForFrames(
                                  frame_base_id, frame_tip_id));

        // frame_base_id can be kInvalidEntityId for two reasons:
        // (1) To define the base frame of the robot component. In this case, we
        //     replace it with `base_link_element_id`.
        // (2) To define a default solver for the entire robot. In this case,
        //     the tip id must also be kInvalidEntityId.
        if (frame_tip_id == kInvalidEntityId) {
          if (frame_base_id != kInvalidEntityId) {
            return intrinsic::InternalErrorBuilder()
                   << "Unsupported solver mapping configuration. If tip ID "
                      "is '"
                   << kInvalidEntityId << "', then base ID must be too.";
          }

          LOG(INFO) << "Both base and tip are kInvalidEntityId, setting solver "
                    << solver_key << " for entire robot '"
                    << robot_collection_entity->GetLocalName() << "'";
          // Set the solver key for any tip links that
          // a) are directly connected to `base_link`, i.e.
          //    CommonAncestor(base_link_element_id, tip_link) ==
          //        base_link_element_id)
          // b) are part of the same RobotCollection as `base_link`
          INTR_ASSIGN_OR_RETURN(
              auto world_tip_ids,
              world_.GetFinalEntitiesOfRobot(robot_collection_id));
          for (const AttachmentEntityId& tip_id : world_tip_ids) {
            INTR_ASSIGN_OR_RETURN(
                kinematics::ElementId tip_frame,
                LookupSkeletonElementId(world_, requested_subtree_root_id_,
                                        *skeleton_, tip_id));
            LOG(INFO) << "Setting solver " << solver_key << " for chain from "
                      << skeleton_->GetElementName(base_link_element_id)
                      << " to " << skeleton_->GetElementName(tip_frame);
            INTR_RETURN_IF_ERROR(skeleton_->SetSolverKey(
                base_link_element_id, tip_frame, solver_key));
          }

          continue;
        }

        // If we're here, we know that `frame_tip_id` is *not*
        // kInvalidEntityId.
        CHECK(frame_tip_id != kInvalidEntityId);
        // `frame_base_id` might still be kInvalidEntityId (i.e. unspecified.
        // Use the collection`s base link as a fallback in that case.
        kinematics::ElementId solver_chain_base_element_id =
            base_link_element_id;
        if (frame_base_id != kInvalidEntityId) {
          INTR_ASSIGN_OR_RETURN(
              solver_chain_base_element_id,
              LookupSkeletonElementId(world_, requested_subtree_root_id_,
                                      *skeleton_, frame_base_id));
        }

        INTR_ASSIGN_OR_RETURN(
            kinematics::ElementId tip_element_id,
            LookupSkeletonElementId(world_, requested_subtree_root_id_,
                                    *skeleton_, frame_tip_id));

        LOG(INFO) << "Setting solver key '" << solver_key
                  << "' for chain from link "
                  << skeleton_->GetElementName(solver_chain_base_element_id)
                  << " to Skeleton ID "
                  << skeleton_->GetElementName(tip_element_id);
        INTR_RETURN_IF_ERROR(skeleton_->SetSolverKey(
            solver_chain_base_element_id, tip_element_id, solver_key));
      }
    }
    if (handle == requested_subtree_root_id_) {
      // If we're done processing `root_id_`, we're also done processing the
      // entire subtree below it. We can't bail out early (other than by
      // returning a non-OK status), so we set processing_root_ to false so that
      // subsequent calls to OnStartEntity() and OnEndEntity() know to simply
      // return absl::OkStatus() and not do anything.
      processing_requested_subtree_ = false;
    }
    return absl::OkStatus();
  }

  // Use this to retrieve the finished Skeleton after WalkAttachmentTree()
  // finishes.
  //
  // This is rvalue qualified to make clear that the walker must be "consumed"
  // to get the Skeleton. Callers need to std::move(walker), so it's obvious the
  // walker should not be re-used afterwards (and in particular, it's not valid
  // to call GetSkeleton() more than once).
  std::unique_ptr<kinematics::Skeleton> GetSkeleton() && {
    return std::move(skeleton_);
  }

 private:
  const World& world_;
  AttachmentEntityId requested_subtree_root_id_;
  std::unique_ptr<kinematics::Skeleton> skeleton_;
  // True while we're handling the tree rooted at `requested_subtree_root_id_`,
  // false otherwise. We ignore any entities we encounter while this is false,
  // unless it's `requested_subtree_root_id_`itself.
  bool processing_requested_subtree_ = false;
};

}  // namespace

std::string GetUniqueName(const World& world, AttachmentEntityId root,
                          EntityId id) {
  std::string element_name = world.GetLocalNameForEntityById(id);
  std::string root_name = world.GetLocalNameForEntityById(root);
  if (element_name != root_name &&
      world.FindByLocalNames(root, {element_name}).size() == 1) {
    // Name is unique in the World, return it as-is.
    return element_name;
  } else {
    return absl::StrCat(element_name, "(", id.value(), ")");
  }
}

absl::StatusOr<std::unique_ptr<kinematics::Skeleton>>
BuildTreeSkeletonFromWorld(const World& world, AttachmentEntityId root) {
  INTR_RETURN_IF_ERROR(world.GetEntityById(root).status());
  SkeletonWalker walker(
      world, root,
      absl::StrCat("skeleton_with_root_at_",
                   GetUniqueName(world, kRootEntityId, root)));

  INTR_RETURN_IF_ERROR(WalkAttachmentTree(world, walker));

  return std::move(walker).GetSkeleton();
}

}  // namespace intrinsic
