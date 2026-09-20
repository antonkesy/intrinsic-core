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

#include "intrinsic/world/objects/object_world_data.h"

#include <memory>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {
namespace {

absl::Status ObjectNameEqualsGlobalFrameNameError(absl::string_view name) {
  return absl::InvalidArgumentError(
      absl::StrCat("A frame under the root object may not have the same name "
                   "as an object (\"",
                   name, "\")"));
}

absl::Status ObjectNameEqualsGlobalObjectNameError(absl::string_view name) {
  return absl::InvalidArgumentError(
      absl::StrCat("A global object may not have the same name "
                   "as another global object (\"",
                   name, "\")"));
}

}  // namespace

ObjectWorldData::ObjectWorldData(const World& world) : world_(world) {}

// Not defined in the header because of incomplete types (WorldObject).
ObjectWorldData::~ObjectWorldData() = default;

const World& ObjectWorldData::GetEntityWorld() const { return world_; }

absl::StatusOr<World*> ObjectWorldData::GetMutableEntityWorld() {
  return absl::FailedPreconditionError(
      "ObjectWorldData::GetMutableEntityWorld() called from immutable "
      "context.");
}

absl::Status ObjectWorldData::CheckObjectNameAgainstGlobalObjectNames(
    const WorldObjectName& name_to_be_added) const {
  for (const auto& [_, object] : objects_by_id_) {
    INTR_ASSIGN_OR_RETURN(bool object_name_is_global_alias,
                          object->NameIsGlobalAlias());
    if (object_name_is_global_alias &&
        object->GetName().value() == name_to_be_added.value()) {
      return ObjectNameEqualsGlobalObjectNameError(name_to_be_added.value());
    }
  }
  for (const WorldObject* object :
       objects_by_id_.at(RootObjectId())->GetChildren()) {
    if (object->GetName().value() == name_to_be_added.value()) {
      return ObjectNameEqualsGlobalObjectNameError(name_to_be_added.value());
    }
  }
  return absl::OkStatus();
}

absl::Status ObjectWorldData::CheckGlobalFrameNameAgainstGlobalObjectNames(
    const FrameName& name_to_be_added) const {
  for (const auto& [_, object] : objects_by_id_) {
    INTR_ASSIGN_OR_RETURN(bool object_name_is_global_alias,
                          object->NameIsGlobalAlias());
    if (object_name_is_global_alias &&
        object->GetName().value() == name_to_be_added.value()) {
      return ObjectNameEqualsGlobalFrameNameError(name_to_be_added.value());
    }
  }
  for (const WorldObject* object :
       objects_by_id_.at(RootObjectId())->GetChildren()) {
    if (object->GetName().value() == name_to_be_added.value()) {
      return ObjectNameEqualsGlobalFrameNameError(name_to_be_added.value());
    }
  }
  return absl::OkStatus();
}

absl::Status ObjectWorldData::CheckObjectNameAgainstGlobalFrameNames(
    const WorldObjectName& name_to_be_added) const {
  for (const Frame* frame :
       objects_by_id_.at(RootObjectId())->GetFramesSorted()) {
    if (frame->GetName().value() == name_to_be_added.value()) {
      return ObjectNameEqualsGlobalFrameNameError(name_to_be_added.value());
    }
  }
  return absl::OkStatus();
}

absl::Status ObjectWorldData::InsertObject(
    std::unique_ptr<WorldObject> object) {
  ObjectWorldResourceId id = object->GetId();
  WorldObject* object_ptr = object.get();
  if (!objects_by_id_.insert({id, std::move(object)}).second) {
    return absl::AlreadyExistsError(absl::StrCat(
        "Object with the id \"", id.value(), "\" already exists."));
  }
  objects_by_name_[object_ptr->GetName()].insert(object_ptr);
  for (const AttachmentEntityId& entity_id : object_ptr->GetEntityIds()) {
    objects_by_entity_id_[entity_id] = object_ptr;
  }
  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<WorldObject>> ObjectWorldData::RemoveObject(
    ObjectWorldResourceId object_id) {
  if (!objects_by_id_.contains(object_id)) {
    return absl::NotFoundError(
        absl::Substitute("Object with the id \"$0\" could not be removed since "
                         "it cannot be found.",
                         object_id.value()));
  }

  auto object = std::move(objects_by_id_.extract(object_id).mapped());
  objects_by_name_[object->GetName()].erase(object.get());
  for (const AttachmentEntityId& entity_id : object->GetEntityIds()) {
    objects_by_entity_id_.erase(entity_id);
  }

  return object;
}

absl::Status ObjectWorldData::RenameObject(const WorldObjectName& old_name,
                                           const WorldObjectName& new_name,
                                           WorldObject& object) {
  objects_by_name_[old_name].erase(&object);
  objects_by_name_[new_name].insert(&object);
  return absl::OkStatus();
}

void ObjectWorldData::RegisterEntity(AttachmentEntityId entity_id,
                                     WorldObject* object) {
  objects_by_entity_id_[entity_id] = object;
}

void ObjectWorldData::UnregisterEntity(AttachmentEntityId entity_id) {
  objects_by_entity_id_.erase(entity_id);
}

MutableObjectWorldData::MutableObjectWorldData(World& world)
    : ObjectWorldData(world), mutable_world_(world) {}

absl::StatusOr<World*> MutableObjectWorldData::GetMutableEntityWorld() {
  return &mutable_world_;
}

}  // namespace object_world
}  // namespace intrinsic
