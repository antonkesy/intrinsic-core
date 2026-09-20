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

#include "intrinsic/world/objects/kinematic_object_internal.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.pb.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world_data.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/physical_object.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/robot_payload/robot_payload.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {

KinematicObject::KinematicObject(ObjectWorldResourceId id, WorldObjectName name,
                                 RobotCollectionsEntityId robot_entity_id,
                                 WorldHashSet<AttachmentEntityId> entity_ids,
                                 ObjectWorldData& data)
    : PhysicalObject(std::move(id), std::move(name), robot_entity_id,
                     std::move(entity_ids), data),
      robot_entity_id_(robot_entity_id) {}

absl::StatusOr<eigenmath::VectorXd> KinematicObject::GetJointPositions() const {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<const DofKinematicView> dof_view,
                        GetKinematicView());
  return dof_view->GetDofValues();
}

absl::StatusOr<JointLimitsXd> KinematicObject::GetJointSystemLimits() const {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<const DofKinematicView> dof_view,
                        GetKinematicView());
  return dof_view->GetDofSystemLimits();
}

absl::StatusOr<JointLimitsXd> KinematicObject::GetJointApplicationLimits()
    const {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<const DofKinematicView> dof_view,
                        GetKinematicView());
  return dof_view->GetDofApplicationLimits();
}

absl::StatusOr<std::vector<JointEntityId>> KinematicObject::GetJointEntityIds()
    const {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<const DofKinematicView> dof_view,
                        GetKinematicView());
  return dof_view->GetJointEntityIds();
}

absl::StatusOr<
    std::vector<intrinsic_proto::world::KinematicsComponent::MotionType>>
KinematicObject::GetJointTypes() const {
  INTR_ASSIGN_OR_RETURN(
      const CollectionsComponent* collections,
      GetEntityWorld().GetComponentByEntityId<CollectionsComponent>(
          robot_entity_id_));
  const std::vector<CollectionsMemberEntityId>& joint_ids =
      collections->GetCollectionMembers(CollectionsComponent::kJoints);

  std::vector<intrinsic_proto::world::KinematicsComponent::MotionType> result;
  result.reserve(joint_ids.size());

  for (CollectionsMemberEntityId joint_id : joint_ids) {
    INTR_ASSIGN_OR_RETURN(
        const KinematicsComponent* kinematics,
        GetEntityWorld().GetComponentByEntityId<KinematicsComponent>(joint_id));
    result.push_back(kinematics->GetMotionType());
  }

  return result;
}

absl::StatusOr<WorldHashMap<std::string, eigenmath::VectorXd>>
KinematicObject::GetNamedJointConfigurations() const {
  INTR_ASSIGN_OR_RETURN(const RobotComponent* component,
                        GetEntityWorld().GetComponentByEntityId<RobotComponent>(
                            GetRobotEntityId()));

  return component->GetNamedConfigurations();
}

// Returns the named joint configuration stored on this kinematic object.
absl::StatusOr<eigenmath::VectorXd> KinematicObject::GetNamedJointConfiguration(
    absl::string_view name) const {
  INTR_ASSIGN_OR_RETURN(const auto configurations,
                        GetNamedJointConfigurations());
  if (!configurations.contains(name)) {
    return absl::NotFoundError(
        absl::StrCat("Missing named configuration '", name, "'"));
  }
  return configurations.at(name);
}

std::vector<Frame*> KinematicObject::GetIsoFlangeFrames() {
  std::vector<Frame*> result;
  // TODO(b/226077700): Allow explicitly configuring the flange frames, e.g.,
  // via RobotComponent on the robot's collections entity.
  for (Frame* frame : GetFrames()) {
    if (frame->GetName() == FlangeFrameName()) {
      result.push_back(frame);
    }
  }
  return result;
}

std::vector<const Frame*> KinematicObject::GetIsoFlangeFrames() const {
  std::vector<const Frame*> result;
  // TODO(b/226077700): Allow explicitly configuring the flange frames, e.g.,
  // via RobotComponent on the robot's collections entity.
  for (const Frame* frame : GetFrames()) {
    if (frame->GetName() == FlangeFrameName()) {
      result.push_back(frame);
    }
  }
  return result;
}

absl::StatusOr<Frame*> KinematicObject::GetSingleIsoFlangeFrame() {
  std::vector<Frame*> frames = GetIsoFlangeFrames();
  if (frames.empty()) {
    return absl::NotFoundError(
        absl::Substitute("Kinematic object \"$0\" does no have any flange "
                         "frame configured, but exactly one was expected.",
                         GetName().value()));
  } else if (frames.size() > 1) {
    return absl::NotFoundError(absl::Substitute(
        "Kinematic object \"$0\" has more than one flange "
        "frame configured, but exactly one was expected. The available flange "
        "frames are: $1.",
        GetName().value(),
        absl::StrJoin(frames, ", ", [](std::string* out, const Frame* frame) {
          out->append(frame->GetName().value());
        })));
  }
  return frames.front();
}

absl::StatusOr<const Frame*> KinematicObject::GetSingleIsoFlangeFrame() const {
  // Delegate to non-const implementation.
  INTR_ASSIGN_OR_RETURN(
      Frame * result,
      const_cast<KinematicObject*>(this)->GetSingleIsoFlangeFrame());
  return result;
}

absl::Status KinematicObject::SetJointPositions(
    const eigenmath::VectorXd& joint_positions,
    std::optional<absl::Time> timestamp, bool enforce_monotonic_time) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<DofKinematicView> dof_view,
                        GetKinematicView());
  return dof_view->SetDofValues(joint_positions, /*enforce_limits=*/true,
                                timestamp, enforce_monotonic_time);
}

absl::Status KinematicObject::SetJointPositions(
    const eigenmath::VectorXd& joint_positions, bool enforce_limits,
    std::optional<absl::Time> timestamp, bool enforce_monotonic_time) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<DofKinematicView> dof_view,
                        GetKinematicView());
  return dof_view->SetDofValues(joint_positions, enforce_limits, timestamp,
                                enforce_monotonic_time);
}

absl::Status KinematicObject::SetJointSystemLimits(
    const JointLimitsXd& joint_limits, bool enforce_limits) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<DofKinematicView> dof_view,
                        GetKinematicView());
  return dof_view->SetDofSystemLimits(joint_limits, enforce_limits);
}

absl::Status KinematicObject::SetJointApplicationLimits(
    const JointLimitsXd& joint_limits, bool enforce_limits) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<DofKinematicView> dof_view,
                        GetKinematicView());
  return dof_view->SetDofApplicationLimits(joint_limits, enforce_limits);
}

absl::Status KinematicObject::UpdateJointSystemLimits(
    const intrinsic_proto::JointLimitsUpdate& joint_limits,
    bool enforce_limits) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<DofKinematicView> dof_view,
                        GetKinematicView());

  INTR_ASSIGN_OR_RETURN(
      auto updated_limits,
      UpdateJointLimits(dof_view->GetDofSystemLimits(), joint_limits));
  return dof_view->SetDofSystemLimits(updated_limits, enforce_limits);
}

absl::Status KinematicObject::UpdateJointApplicationLimits(
    const intrinsic_proto::JointLimitsUpdate& joint_limits,
    bool enforce_limits) {
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<DofKinematicView> dof_view,
                        GetKinematicView());

  INTR_ASSIGN_OR_RETURN(
      auto updated_limits,
      UpdateJointLimits(dof_view->GetDofApplicationLimits(), joint_limits));
  return dof_view->SetDofApplicationLimits(updated_limits, enforce_limits);
}

absl::Status KinematicObject::AddIkSolverKey(
    absl::string_view ik_solver, std::optional<std::string> tip_link_name) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  INTR_ASSIGN_OR_RETURN(
      RobotComponent * robot_component,
      world->GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));

  INTR_ASSIGN_OR_RETURN(LinkEntityId base_link_entity_id,
                        world->GetBaseLink(GetRobotEntityId()),
                        _ << "Failed to get base link for " << GetName());
  INTR_ASSIGN_OR_RETURN(auto final_entities,
                        world->GetFinalEntitiesOfRobot(GetRobotEntityId()),
                        _ << "Failed to get final entities for " << GetName());

  AttachmentEntityId tip_link_entity_id(kInvalidEntityId);
  if (final_entities.size() > 1) {
    if (!tip_link_name.has_value()) {
      return absl::FailedPreconditionError(absl::StrCat(
          "More than one final entity found for ", GetName().value(),
          ". Expected exactly one. Please specify a tip link name to remove "
          "the ambiguity."));
    }

    // Look for the tip link name in the final entities.
    for (const auto& final_entity : final_entities) {
      const auto final_entity_name =
          world->GetLocalNameForEntityById(final_entity);
      if (final_entity_name == *tip_link_name) {
        tip_link_entity_id = final_entity;
        break;
      }
    }

    // If we didn't find the tip link name, return an error.
    if (tip_link_entity_id == kInvalidEntityId) {
      return absl::FailedPreconditionError(absl::StrCat(
          "More than one final entity found for ", GetName().value(),
          ". The specified link name '", *tip_link_name,
          "' does not match any of the final entities."));
    }
  } else if (final_entities.empty()) {
    return absl::FailedPreconditionError(
        absl::StrCat("No final entity found for ", GetName().value()));
  } else {
    tip_link_entity_id = *final_entities.begin();

    // If we have a tip link name, check that it matches the final entity name.
    if (tip_link_name.has_value()) {
      const auto final_entity_name =
          world->GetLocalNameForEntityById(tip_link_entity_id);
      if (final_entity_name != *tip_link_name) {
        return absl::FailedPreconditionError(
            absl::StrCat("The tip link name '", *tip_link_name,
                         "' does not match the name of the final entity '",
                         final_entity_name, "'"));
      }
    }
  }

  return robot_component->AddSolvableFrames(base_link_entity_id,
                                            tip_link_entity_id, ik_solver);
}

absl::Status KinematicObject::SetRobotIKSolverKey(
    absl::string_view ik_solver_key) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  INTR_ASSIGN_OR_RETURN(
      RobotComponent * robot_component,
      world->GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));
  if (ik_solver_key.empty()) {
    return absl::InvalidArgumentError("The ik_solver_key cannot be empty.");
  }
  auto solver_frames = robot_component->GetSolvableFrames();
  for (const auto& solver_frame_pair : solver_frames) {
    INTR_RETURN_IF_ERROR(robot_component->RemoveSolvableFrames(
        solver_frame_pair.first, solver_frame_pair.second));
    INTR_RETURN_IF_ERROR(robot_component->AddSolvableFrames(
        solver_frame_pair.first, solver_frame_pair.second, ik_solver_key));
  }
  return absl::OkStatus();
}

absl::Status KinematicObject::UpdateRobotKinematics(
    const std::vector<std::pair<world::ObjectEntityFilter, Pose3d>>&
        kinematics_update) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());

  for (const auto& update : kinematics_update) {
    INTR_ASSIGN_OR_RETURN(auto entity_id, GetTransformEntityId(update.first));
    INTR_ASSIGN_OR_RETURN(auto entity, world->GetEntityById(entity_id));
    if (entity->IsEntityValid<JointEntityId>()) {
      INTR_ASSIGN_OR_RETURN(auto kinematic_component,
                            entity->GetComponent<KinematicsComponent>());
      kinematic_component->SetParentTInboard(update.second);
      INTR_RETURN_IF_ERROR(
          world->RefreshDofParentTThis(JointEntityId(entity_id.id)));
    } else if (entity->IsEntityValid<LinkEntityId>()) {
      INTR_ASSIGN_OR_RETURN(auto attachment_component,
                            entity->GetComponent<AttachmentComponent>());
      attachment_component->SetParentTThis(update.second, absl::Now());
    } else {
      return absl::InvalidArgumentError(
          absl::StrCat("Entity ", entity->GetLocalName(),
                       " is not a joint or link entity."));
    }
  }
  INTR_ASSIGN_OR_RETURN(
      RobotComponent * robot_component,
      world->GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));
  robot_component->SetAreKinematicsUpdated(true);
  return absl::OkStatus();
}

absl::StatusOr<std::vector<std::pair<world::ObjectEntityFilter, Pose3d>>>
KinematicObject::ExtractRobotKinematicsProperties() const {
  std::vector<std::pair<world::ObjectEntityFilter, Pose3d>>
      kinematics_chain_data;
  const auto& world = GetEntityWorld();
  auto entity_ids = GetEntityIds();
  for (const auto& entity_id : entity_ids) {
    INTR_ASSIGN_OR_RETURN(auto entity, world.GetEntityById(entity_id));
    if (entity->IsEntityValid<JointEntityId>()) {
      INTR_ASSIGN_OR_RETURN(auto kinematic_component,
                            entity->GetComponent<KinematicsComponent>());
      world::ObjectEntityFilter filter;
      filter.IncludeEntityName(entity->GetLocalName());
      kinematics_chain_data.push_back(
          std::make_pair(filter, kinematic_component->GetParentTInboard()));
    } else if (entity->IsEntityValid<LinkEntityId>()) {
      INTR_ASSIGN_OR_RETURN(auto attachment_component,
                            entity->GetComponent<AttachmentComponent>());
      world::ObjectEntityFilter filter;
      filter.IncludeEntityName(entity->GetLocalName());
      kinematics_chain_data.push_back(
          std::make_pair(filter, attachment_component->GetParentTThis()));
    }
  }
  return kinematics_chain_data;
}

absl::StatusOr<bool> KinematicObject::AreRobotKinematicsUpdated() const {
  const auto& world = GetEntityWorld();
  INTR_ASSIGN_OR_RETURN(
      const RobotComponent* robot_component,
      world.GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));
  return robot_component->AreKinematicsUpdated();
}

absl::StatusOr<CartesianLimits> KinematicObject::GetCartesianLimits() const {
  INTR_ASSIGN_OR_RETURN(const RobotComponent* component,
                        GetEntityWorld().GetComponentByEntityId<RobotComponent>(
                            GetRobotEntityId()));
  return component->GetCartesianLimits();
}

absl::Status KinematicObject::SetCartesianLimits(
    const CartesianLimits& cart_limits) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  INTR_ASSIGN_OR_RETURN(
      RobotComponent * component,
      world->GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));
  return component->SetCartesianLimits(cart_limits);
}

absl::Status KinematicObject::SetNamedJointConfiguration(
    absl::string_view name, const eigenmath::VectorXd& joint_position) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  INTR_ASSIGN_OR_RETURN(
      RobotComponent * component,
      world->GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<const DofKinematicView> dof_view,
                        GetKinematicView());
  if (dof_view->GetDofCount() != joint_position.size()) {
    return absl::InvalidArgumentError(
        absl::Substitute("DoF count mismatch: provided joint configuration "
                         "named \"$0\" has $1 DoFs, "
                         "world's robot \"$2\" expects $3 DoFs",
                         name, joint_position.size(), GetName().value(),
                         dof_view->GetDofCount()));
  }

  component->SetNamedConfiguration(name, joint_position);

  return absl::OkStatus();
}

absl::Status KinematicObject::RemoveNamedJointConfiguration(
    absl::string_view name) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  INTR_ASSIGN_OR_RETURN(
      RobotComponent * component,
      world->GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));

  if (!component->GetNamedConfigurations().contains(name)) {
    return absl::NotFoundError(absl::Substitute(
        "Configuration with name \"$0\" cannot be removed from kinematic "
        "object \"$1\" since no such configuration exists.",
        name, GetName().value()));
  }

  component->RemoveNamedConfiguration(name);

  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<DofKinematicView>>
KinematicObject::GetKinematicView() {
  // The kinematic view could also be cached, but we'd need to invalidate the
  // cached view automatically when the underlying World gets updated.
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  return world->GetDofKinematicView(robot_entity_id_);
}

absl::StatusOr<std::unique_ptr<const DofKinematicView>>
KinematicObject::GetKinematicView() const {
  // The kinematic view could also be cached, but we'd need to invalidate the
  // cached view automatically when the underlying World gets updated.
  return GetEntityWorld().GetDofKinematicView(robot_entity_id_);
}

absl::StatusOr<AttachmentEntityId>
KinematicObject::FinalEntityIfKinematicObjectOrElseRootEntity() const {
  INTR_ASSIGN_OR_RETURN(
      AttachmentEntityId final_entity_id,
      GetEntityWorld().GetFinalEntityOfRobotKinematicChain(GetRobotEntityId()));
  if (final_entity_id == kInvalidEntityId) {
    return absl::InvalidArgumentError(absl::Substitute(
        "Could not uniquely determine the final entity of object \"$0\". Does "
        "it have more than one leaf entity (e.g., a two-finger gripper)?",
        GetName().value()));
  }
  return final_entity_id;
}

absl::StatusOr<std::optional<RobotPayload>> KinematicObject::GetMountedPayload()
    const {
  INTR_ASSIGN_OR_RETURN(const RobotComponent* component,
                        GetEntityWorld().GetComponentByEntityId<RobotComponent>(
                            GetRobotEntityId()));
  return component->GetMountedPayload();
}

absl::StatusOr<RobotPayload*> KinematicObject::GetMutableMountedPayload() {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  INTR_ASSIGN_OR_RETURN(
      RobotComponent * component,
      world->GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));
  std::optional<RobotPayload>& payload = component->GetMountedPayload();
  if (payload.has_value()) {
    return &payload.value();
  }
  return nullptr;
}

absl::Status KinematicObject::SetMountedPayload(
    std::optional<RobotPayload> payload) {
  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  INTR_ASSIGN_OR_RETURN(
      RobotComponent * component,
      world->GetComponentByEntityId<RobotComponent>(GetRobotEntityId()));
  return component->SetMountedPayload(payload);
}

absl::StatusOr<WorldHashMap<AttachmentEntityId,
                            WorldHashMap<AttachmentEntityId, std::string>>>
KinematicObject::GetIkSolvers() const {
  INTR_ASSIGN_OR_RETURN(const RobotComponent* component,
                        GetEntityWorld().GetComponentByEntityId<RobotComponent>(
                            GetRobotEntityId()));
  WorldHashSet<std::pair<AttachmentEntityId, AttachmentEntityId>>
      base_tip_pairs = component->GetSolvableFrames();
  WorldHashMap<AttachmentEntityId,
               WorldHashMap<AttachmentEntityId, std::string>>
      solver_key_map;
  for (const auto& [base, tip] : base_tip_pairs) {
    INTR_ASSIGN_OR_RETURN(std::string solver_key,
                          component->GetSolverKeyForFrames(base, tip));
    // Look up the robot base and tip ID(s), so we can fill them in if the proto
    // specifies kInvalidEntityId.
    INTR_ASSIGN_OR_RETURN(LinkEntityId robot_base_id,
                          GetEntityWorld().GetBaseLink(GetRobotEntityId()));
    INTR_ASSIGN_OR_RETURN(
        auto world_tip_ids,
        GetEntityWorld().GetFinalEntitiesOfRobot(GetRobotEntityId()));
    AttachmentEntityId final_base = base;
    if (final_base == kInvalidEntityId) {
      // If the base link is invalid, assume the user meant the robot's base
      // link.
      final_base = robot_base_id;
    }
    if (tip == kInvalidEntityId) {
      // Tip is kInvalidEntityId, the solver is for the entire robot (downstream
      // of the specified base, if any). In that case, we set the solver for all
      // tip links.
      for (const auto& robot_tip_id : world_tip_ids) {
        solver_key_map[final_base][robot_tip_id] = solver_key;
      }
    } else {
      // If we have an explicit tip link, set the solver for the chain from base
      // to that one link.
      solver_key_map[final_base][tip] = std::move(solver_key);
    }
  }
  return solver_key_map;
}

}  // namespace object_world
}  // namespace intrinsic
