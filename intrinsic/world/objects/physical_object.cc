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

#include "intrinsic/world/objects/physical_object.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collision_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world_creation_utils.h"
#include "intrinsic/world/objects/object_world_data.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {
namespace {

using ::intrinsic::object_world::object_world_object_entity_filter_details::
    GetObjectCollisionEntitiesMatchingEntityFilter;
using ::intrinsic::object_world::object_world_object_entity_filter_details::
    GetObjectEntitiesMatchingEntityFilter;

}  // namespace

PhysicalObject::PhysicalObject(ObjectWorldResourceId id, WorldObjectName name,
                               CollectionsEntityId collection_entity_id,
                               WorldHashSet<AttachmentEntityId> entity_ids,
                               ObjectWorldData& data)
    : WorldObject(std::move(id), std::move(name), std::move(entity_ids), data),
      collection_entity_id_(collection_entity_id) {}

absl::StatusOr<bool> PhysicalObject::NameIsGlobalAlias() const {
  INTR_ASSIGN_OR_RETURN(const WorldEntity* entity,
                        GetEntityWorld().GetEntityById(collection_entity_id_));
  if (!entity->GetAlias().empty()) {
    return true;
  }

  INTR_ASSIGN_OR_RETURN(AttachmentEntityId root_id, GetRootEntityId());
  INTR_ASSIGN_OR_RETURN(const WorldEntity* root_entity,
                        GetEntityWorld().GetEntityById(root_id));
  if (!root_entity->GetAlias().empty()) {
    return true;
  }
  return false;
}

absl::Status PhysicalObject::SetName(const WorldObjectName& name,
                                     bool name_is_global_alias) {
  if (name.value().empty()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Name of object \"", name_.value(),
        "\" cannot be updated. The object name must not be empty."));
  }

  INTR_RETURN_IF_ERROR(
      GetObjectWorldData().CheckObjectNameAgainstGlobalFrameNames(name));

  INTR_RETURN_IF_ERROR(CheckNameIsCompatibleWithObjectView(name.value()));
  INTR_RETURN_IF_ERROR(
      GetParent()->CheckObjectNameAgainstChildrenObjectAndFrameNames(name,
                                                                     this));

  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  INTR_ASSIGN_OR_RETURN(const WorldEntity* entity,
                        world->GetEntityById(collection_entity_id_));

  const auto old_name = name_;

  if (name_is_global_alias) {
    WorldEntity* root_entity = nullptr;
    if (entity->GetAlias().empty()) {
      // Object name was generated from the alias of the collections root
      // entity.
      INTR_ASSIGN_OR_RETURN(bool old_name_is_global_alias, NameIsGlobalAlias());
      if (old_name_is_global_alias) {
        INTR_ASSIGN_OR_RETURN(AttachmentEntityId root_id, GetRootEntityId());
        INTR_ASSIGN_OR_RETURN(root_entity, world->GetEntityById(root_id));
      }
    }

    absl::Status status = world->SetAlias(collection_entity_id_, name.value());
    if (absl::IsInvalidArgument(status)) {
      return absl::InvalidArgumentError(
          absl::StrCat("Name of object \"", name_.value(),
                       "\" cannot be updated. The object name \"", name.value(),
                       "\" is already in use."));
    }
    INTR_RETURN_IF_ERROR(status);

    // Order matters here. Update object name after setting the alias was
    // successful to avoid that the two names get out of sync.
    name_ = name;

    if (root_entity != nullptr) {
      // If the object name originally was generated from the alias of the
      // collections root entity, we have now switched to an alias on the
      // collections entity and should clear the alias on the collections
      // root entity.
      INTR_RETURN_IF_ERROR(root_entity->SetAlias(""));
    }
  } else {
    // For the case of non global alias, set collection entity's local name to
    // the new name and clear previously used aliases if applicable
    INTR_ASSIGN_OR_RETURN(WorldEntity * mutable_collection_entity,
                          world->GetEntityById(collection_entity_id_));
    INTR_RETURN_IF_ERROR(mutable_collection_entity->SetLocalName(name.value()));
    name_ = name;
    if (entity->GetAlias().empty()) {
      // Object name was generated from the alias of the collections root
      // entity.
      INTR_ASSIGN_OR_RETURN(AttachmentEntityId root_id, GetRootEntityId());
      INTR_RETURN_IF_ERROR(world->SetAlias(root_id, ""));
    } else {
      INTR_RETURN_IF_ERROR(world->SetAlias(collection_entity_id_, ""));
    }
  }

  INTR_RETURN_IF_ERROR(
      GetObjectWorldData().RenameObject(old_name, name, *this));
  return absl::OkStatus();
}

absl::Status PhysicalObject::ReparentTo(WorldObject& new_parent) {
  return ReparentTo(new_parent, world::ObjectEntityFilter::BaseEntity());
}

absl::Status PhysicalObject::ReparentToFinalEntityOf(WorldObject& new_parent) {
  return ReparentTo(new_parent, world::ObjectEntityFilter::FinalEntity());
}

absl::Status PhysicalObject::ReparentTo(
    WorldObject& new_parent, const world::ObjectEntityFilter& filter) {
  INTR_RETURN_IF_ERROR(
      new_parent.CheckObjectNameAgainstChildrenObjectAndFrameNames(GetName(),
                                                                   this))
      << "Cannot reparent object with name \"" << GetName().value()
      << "\" to new parent with name \"" << new_parent.GetName()
      << "\" because name \"" << GetName().value()
      << "\" already exists under object \"" << new_parent.GetName() << "\".";

  INTR_ASSIGN_OR_RETURN(
      const auto parent_entity_ids,
      GetObjectEntitiesMatchingEntityFilter(new_parent, filter,
                                            /*expanded_list=*/false),
      _.SetPrepend() << absl::Substitute(
          "Object \"$0\" could not find parent entity in object \"$1\": ",
          GetName().value(), new_parent.GetName().value()));
  if (parent_entity_ids.empty()) {
    return absl::NotFoundError(absl::Substitute(
        "Object \"$0\" could not find parent entity within object \"$1\"",
        GetName().value(), new_parent.GetName().value()));
  }

  if (parent_entity_ids.size() != 1) {
    std::vector<std::string> entity_names;
    for (const auto& entity_id : parent_entity_ids) {
      absl::StatusOr<const WorldEntity*> entity =
          GetEntityWorld().GetEntityById(entity_id);
      std::string entity_name = entity.ok()
                                    ? entity.value()->GetLocalName()
                                    : std::string(entity.status().message());
      entity_names.push_back(std::move(entity_name));
    }
    // Sorts the names to make the error message deterministic.
    absl::c_sort(entity_names);
    return absl::InvalidArgumentError(absl::Substitute(
        "\"$0\" can only be parented to a single entity but \"$1\""
        "has multiple final entities: [$2]. Either specify one of these "
        "entities explicitly, or create a frame under one of these entities "
        "and then parent \"$0\" to the newly created frame. Refer to "
        "https://flowstate.intrinsic.ai/docs/learn/world_concepts/#entities "
        "for more information about entities.",
        GetName().value(), new_parent.GetName(),
        absl::StrJoin(entity_names, ", ")));
  }

  // Update entities.
  const AttachmentEntityId new_parent_id = *parent_entity_ids.begin();

  // Disallows the request if parenting to an entity that is a child of the
  // root object.
  if (new_parent.GetId() == RootObjectId() && new_parent_id != kRootEntityId) {
    absl::StatusOr<const WorldEntity*> parent_entity =
        GetEntityWorld().GetEntityById(new_parent_id);
    if (!parent_entity.ok()) {
      // This should never happen but we still have to handle it.
      LOG(ERROR) << "Failed to get entity for id: " << new_parent_id;
    }
    const std::string parent_name =
        parent_entity.ok() ? parent_entity.value()->GetLocalName() : "";

    return absl::InvalidArgumentError(
        absl::Substitute("Cannot reparent object \"$0\" to root via \"$1\". An "
                         "object can only be parented to another object or to "
                         "a frame that has an object as an ancestor.",
                         GetName().value(), parent_name));
  }

  INTR_ASSIGN_OR_RETURN(
      AttachmentEntityId child_id, GetRootEntityId(),
      _.SetPrepend() << absl::Substitute("Failed to reparent object \"$0\": ",
                                         GetName().value()));
  Pose3d new_parent_t_child =
      GetEntityWorld().GetTransform(new_parent_id, child_id);

  // If this is an attachment frame, then we will parent directly to the frame,
  // but the pose must be the identity pose in order to match that of the
  // attachment frame.
  for (const Frame* parent_frame : new_parent.GetFrames()) {
    if (parent_frame->GetEntityId() == new_parent_id &&
        parent_frame->IsAttachmentFrame()) {
      new_parent_t_child = Pose3d();
      break;
    }
  }

  INTR_ASSIGN_OR_RETURN(
      World * world, GetMutableEntityWorld(),
      _.SetPrepend() << absl::Substitute("Failed to reparent object \"$0\": ",
                                         GetName().value()));
  INTR_RETURN_IF_ERROR(world->ReparentEntity(new_parent_id, child_id,
                                             new_parent_t_child, absl::Now()))
          .SetPrepend()
      << absl::Substitute("Failed to reparent object \"$0\": ",
                          GetName().value());

  // Update object-view only once entity update was successful.
  INTR_RETURN_IF_ERROR(GetParent()->RemoveChildAsymmetric(*this)).SetPrepend()
      << absl::Substitute("Failed to reparent object \"$0\": ",
                          GetName().value());
  SetParentAsymmetric(&new_parent);
  new_parent.AddChildAsymmetric(this);

  return absl::OkStatus();
}

absl::Status PhysicalObject::ToggleCollisionsWith(
    WorldObject& other, bool enable_collisions,
    const world::ObjectEntityFilter& this_entity_filter,
    const world::ObjectEntityFilter& other_entity_filter) {
  if (other.GetId() == RootObjectId()) {
    // Will return error for root object.
    return other.ToggleCollisionsWith(other, enable_collisions,
                                      this_entity_filter, other_entity_filter);
  }

  INTR_ASSIGN_OR_RETURN(WorldHashSet<CollisionEntityId> this_entities,
                        GetObjectCollisionEntitiesMatchingEntityFilter(
                            *this, this_entity_filter));
  INTR_ASSIGN_OR_RETURN(WorldHashSet<CollisionEntityId> other_entities,
                        GetObjectCollisionEntitiesMatchingEntityFilter(
                            other, other_entity_filter));

  for (const CollisionEntityId& this_entity_id : this_entities) {
    for (const CollisionEntityId& other_entity_id : other_entities) {
      INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
      if (enable_collisions) {
        INTR_RETURN_IF_ERROR(world->RemoveExclusionPair(
            PhysicalEntityId(this_entity_id.value()),
            PhysicalEntityId(other_entity_id.value())));
      } else {
        INTR_RETURN_IF_ERROR(
            world->AddExclusionPair(PhysicalEntityId(this_entity_id.value()),
                                    PhysicalEntityId(other_entity_id.value())));
      }
    }
  }

  return absl::OkStatus();
}

namespace {

absl::Status RemoveAllCollisionReferencesToEntity(World* world,
                                                  EntityId entity_id) {
  for (CollisionEntityId other_entity_id :
       world->GetTypedEntityIds<CollisionEntityId>()) {
    if (other_entity_id == entity_id) continue;

    INTR_ASSIGN_OR_RETURN(
        CollisionComponent * collision,
        world->GetComponentByEntityId<CollisionComponent>(other_entity_id));

    collision->RemoveExclusionId(PhysicalEntityId(entity_id));
  }

  // Remove the entity from the default rule set in case it is there.
  intrinsic_proto::RuleSet rule_set = world->GetDefaultRuleSet();
  for (auto rule_itr = rule_set.mutable_rules()->begin();
       rule_itr != rule_set.mutable_rules()->end();) {
    const auto& rule = *rule_itr;
    std::set<EntityId::ValueType> new_id_1;
    for (auto& id : rule.id_1()) {
      if (id != entity_id.value()) {
        new_id_1.insert(id);
      }
    }

    std::set<EntityId::ValueType> new_id_2;
    for (auto& id : rule.id_2()) {
      if (id != entity_id.value()) {
        new_id_2.insert(id);
      }
    }

    if (new_id_1.empty() && rule.id_1_size() != 0) {
      rule_itr = rule_set.mutable_rules()->erase(rule_itr);
    } else if (new_id_2.empty() && rule.id_2_size() != 0) {
      rule_itr = rule_set.mutable_rules()->erase(rule_itr);
    } else {
      rule_itr->clear_id_1();
      rule_itr->mutable_id_1()->Add(new_id_1.begin(), new_id_1.end());
      rule_itr->clear_id_2();
      rule_itr->mutable_id_2()->Add(new_id_2.begin(), new_id_2.end());
      rule_itr++;
    }
  }

  INTR_RETURN_IF_ERROR(world->SetDefaultRuleSet(rule_set));
  return absl::OkStatus();
}

}  // namespace

absl::Status PhysicalObject::Delete(bool delete_child_objects) {
  if (!delete_child_objects && !GetChildren().empty()) {
    return absl::InvalidArgumentError(absl::Substitute(
        "Cannot delete object \"$0\" since it has children ($1) "
        "and force-deletion was not requested.",
        GetName().value(),
        absl::StrJoin(GetChildren(), ", ",
                      [](std::string* out, const WorldObject* child) {
                        absl::StrAppend(out, "\"", child->GetName().value(),
                                        "\"");
                      })));
  }

  // Recursively delete all child objects (if there are any) including their
  // frames.
  for (WorldObject* child : GetChildren()) {
    INTR_RETURN_IF_ERROR(child->Delete(/*delete_child_objects=*/true));
  }

  // We need to delete all of the entities, but do it in a way that safely
  // deletes the frames from the object world model. To do this, we need to
  // ping-pong between deleting terminal non-frames and terminal frames until
  // we've covered all of the entities.
  //
  // We can do this by labeling each node with a "phase" number:
  //
  // 1. Root link is assigned phase 0
  // 2. All non-frame children of phase N are assigned phase N
  // 3. All frame children are assigned phase (N + 1)
  // 4. Repeat step (2) with (N + 2)
  //
  // Then, start by deleting the nodes in label-reverse order, where odd phases
  // delete frames, and even phases delete non-frames.
  WorldHashSet<AttachmentEntityId> frame_ids;
  for (const Frame* frame : GetFrames()) {
    frame_ids.insert(frame->GetEntityId());
  }

  INTR_ASSIGN_OR_RETURN(AttachmentEntityId root_entity_id, GetRootEntityId());
  WorldHashMap<AttachmentEntityId, int> phase_labels{{root_entity_id, 0}};

  std::function<absl::StatusOr<int>(AttachmentEntityId)> compute_phase =
      [&](AttachmentEntityId eid) -> absl::StatusOr<int> {
    if (phase_labels.contains(eid)) {
      return phase_labels.at(eid);
    }
    bool is_frame = frame_ids.contains(eid);
    INTR_ASSIGN_OR_RETURN(
        const AttachmentComponent* attachment,
        GetEntityWorld().GetComponentByEntityId<AttachmentComponent>(eid));
    AttachmentEntityId parent_id = attachment->GetParentId();
    bool parent_is_frame = frame_ids.contains(parent_id);

    // If the frame status differs between parent and entity, then increase
    // phase label.
    INTR_ASSIGN_OR_RETURN(int phase, compute_phase(parent_id));
    if (is_frame != parent_is_frame) {
      phase++;
    }

    phase_labels[eid] = phase;
    return phase;
  };

  int starting_phase = 0;
  for (const AttachmentEntityId eid : GetEntityIds()) {
    INTR_ASSIGN_OR_RETURN(int phase, compute_phase(eid));
    starting_phase = std::max(starting_phase, phase);
  }

  INTR_ASSIGN_OR_RETURN(World * world, GetMutableEntityWorld());
  auto delete_entity = [world](AttachmentEntityId member_id) -> absl::Status {
    // Remove and collision exclusions that might be referencing the deleted
    // attachment entities. Note that we do not check for dangling references in
    // collection or collection member components of other entities since this
    // should be kept consistent by means of the object-view itself (e.g.,
    // deleting an object will delete a collection and all of its members).
    INTR_RETURN_IF_ERROR(
        RemoveAllCollisionReferencesToEntity(world, member_id));
    INTR_RETURN_IF_ERROR(world->RemoveEntity(member_id));
    return absl::OkStatus();
  };

  // For each phase, if it's an even phase, then remove non-frame entities, and
  // otherwise remove frame entities.
  for (int phase = starting_phase; phase >= 0; --phase) {
    if (phase % 2 == 0) {
      for (const AttachmentEntityId eid : GetEntityIds()) {
        if (phase_labels.at(eid) == phase) {
          INTR_RETURN_IF_ERROR(delete_entity(eid));
        }
      }
    } else {
      // Child frames are those that have non-frame parents, so any child frame
      // in this phase should be OK to delete including children.
      for (Frame* f : GetChildFrames()) {
        if (phase_labels.at(f->GetEntityId()) == phase) {
          AttachmentEntityId eid = f->GetEntityId();
          bool is_movable = f->CheckIsMovable().ok();
          INTR_RETURN_IF_ERROR(f->DeleteIncludingChildFrames());
          if (!is_movable) {
            INTR_RETURN_IF_ERROR(delete_entity(eid));
          }
        }
      }
    }
  }

  // Remove the collection entity.
  INTR_RETURN_IF_ERROR(world->RemoveEntity(collection_entity_id_));

  // Remove any references to this object.
  INTR_RETURN_IF_ERROR(GetParent()->RemoveChildAsymmetric(*this));

  // Delete this object itself by removing it from the internal data holder.
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<WorldObject> self,
                        GetObjectWorldData().RemoveObject(GetId()));

  return absl::OkStatus();
}

absl::StatusOr<Pose3d> PhysicalObject::GetParentTThis() const {
  INTR_ASSIGN_OR_RETURN(AttachmentEntityId this_id,
                        GetTransformOriginEntityId());
  INTR_ASSIGN_OR_RETURN(AttachmentEntityId parent_id,
                        GetParent()->GetTransformOriginEntityId());
  return GetEntityWorld().GetTransform(parent_id, this_id);
}

absl::StatusOr<AttachmentEntityId>
PhysicalObject::FinalEntityIfKinematicObjectOrElseRootEntity() const {
  return GetRootEntityId();
}

}  // namespace object_world
}  // namespace intrinsic
