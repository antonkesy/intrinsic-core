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

#include "intrinsic/world/util/compose_world_util.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/blocking_counter.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/geometry_options.h"
#include "intrinsic/geometry/proto/geometry_storage_refs.pb.h"
#include "intrinsic/geometry/proto/v1/geometry.pb.h"
#include "intrinsic/geometry/proto/v1/geometry_storage_refs.pb.h"
#include "intrinsic/geometry/proto/v1/transformed_geometry.pb.h"
#include "intrinsic/geometry/storage/geometry_deserializer.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/resources/proto/geometric_resource_data.pb.h"
#include "intrinsic/scene/proto/v1/entity.pb.h"
#include "intrinsic/scene/util/scene_object_updates.h"
#include "intrinsic/stats/scoped_span.h"
#include "intrinsic/util/proto/pb_hash.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/thread_pool.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/object_world_proto_utils.h"
#include "intrinsic/world/objects/simple_transform_node_visitor.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"
#include "intrinsic/world/service/objects/object_world_updates_utils.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

namespace {

using GeometryStorageRefsV0 = intrinsic_proto::geometry::GeometryStorageRefs;
using GeometryStorageRefsV1 =
    intrinsic_proto::geometry::v1::GeometryStorageRefs;
using intrinsic_proto::world::ObjectWorldUpdate;
using intrinsic_proto::world::ObjectWorldUpdates;
using object_world::Frame;
using object_world::ObjectWorld;
using object_world::SimpleTransformNodeConstVisitor;
using object_world::TransformNode;
using object_world::WorldObject;

absl::Status ApplyWorldUpdate(ObjectWorld* object_world,
                              const ObjectWorldUpdate& object_world_update,
                              GeometryLibrary& geolib, bool quiet) {
  switch (object_world_update.update_case()) {
    case ObjectWorldUpdate::kCreateFrame: {
      INTR_ASSIGN_OR_RETURN(
          auto frame, HandleCreateFrameRequest(
                          *object_world, object_world_update.create_frame()));
      LOG_IF(INFO, !quiet) << "Created frame '" << frame->GetName() << "'";
      break;
    }
    case ObjectWorldUpdate::kUpdateObjectName: {
      intrinsic_proto::world::UpdateObjectNameRequest request =
          object_world_update.update_object_name();
      INTR_ASSIGN_OR_RETURN(
          const WorldObject* object,
          GetObjectByReference(*object_world, request.object()));
      WorldObjectName old_object_name = object->GetName();
      INTR_ASSIGN_OR_RETURN(
          object, HandleUpdateObjectNameRequest(*object_world, request));
      LOG_IF(INFO, !quiet) << "Update object named '" << old_object_name
                           << "' to '" << object->GetName() << "'";
      break;
    }
    case ObjectWorldUpdate::kUpdateKinematicObjectProperties: {
      INTR_ASSIGN_OR_RETURN(
          auto* kinematic_object,
          HandleUpdateKinematicObjectPropertiesRequest(
              *object_world,
              object_world_update.update_kinematic_object_properties()));
      LOG_IF(INFO, !quiet) << "Updated kinematic object properties for '"
                           << kinematic_object->GetName() << "'";
      break;
    }
    case ObjectWorldUpdate::kUpdateFrameName: {
      intrinsic_proto::world::UpdateFrameNameRequest request =
          object_world_update.update_frame_name();
      INTR_ASSIGN_OR_RETURN(
          const Frame* frame,
          GetFrameByReference(*object_world, request.frame()));
      WorldObjectName old_object_name = frame->GetParent()->GetName();
      FrameName old_frame_name = frame->GetName();
      INTR_ASSIGN_OR_RETURN(
          frame, HandleUpdateFrameNameRequest(*object_world, request));
      LOG_IF(INFO, !quiet) << "Update frame named '" << old_frame_name
                           << "' (parent object: " << old_object_name
                           << ") to '" << frame->GetName() << "'";
      break;
    }
    case ObjectWorldUpdate::kUpdateTransform: {
      INTR_ASSIGN_OR_RETURN(
          const TransformNode* transform_node,
          HandleUpdateTransformRequest(*object_world,
                                       object_world_update.update_transform()));
      SimpleTransformNodeConstVisitor log_transform_visitor(
          [&](const Frame& frame) {
            LOG_IF(INFO, !quiet)
                << "Updated transform of frame " << frame.GetName();
            return absl::OkStatus();
          },
          [&](const WorldObject& object) {
            LOG_IF(INFO, !quiet)
                << "Updated transform of object '" << object.GetName() << "'";
            return absl::OkStatus();
          });
      INTR_RETURN_IF_ERROR(transform_node->Accept(log_transform_visitor));
      break;
    }
    case ObjectWorldUpdate::kReparentObject: {
      INTR_ASSIGN_OR_RETURN(
          const WorldObject* resource_object,
          HandleReparentObjectRequest(*object_world,
                                      object_world_update.reparent_object()));
      LOG_IF(INFO, !quiet) << "Reparented object '"
                           << resource_object->GetName() << "'";
      break;
    }
    case ObjectWorldUpdate::kReparentFrame: {
      INTR_ASSIGN_OR_RETURN(
          const Frame* resource_frame,
          object_world::HandleReparentFrameRequest(
              *object_world, object_world_update.reparent_frame()));
      LOG_IF(INFO, !quiet) << "Reparented frame '" << resource_frame->GetName()
                           << "'";
      break;
    }
    case ObjectWorldUpdate::kToggleCollisions: {
      INTR_ASSIGN_OR_RETURN(
          auto object_pair,
          HandleToggleCollisionsRequest(
              *object_world, object_world_update.toggle_collisions()));
      LOG_IF(INFO, !quiet) << "Toggled collisions for '"
                           << object_pair.first->GetName() << "' and '"
                           << object_pair.second->GetName() << "'";
      break;
    }
    case ObjectWorldUpdate::kUpdateObjectJoints: {
      INTR_ASSIGN_OR_RETURN(
          const object_world::KinematicObject* object,
          object_world::HandleUpdateObjectJointsRequest(
              *object_world, object_world_update.update_object_joints()));
      LOG_IF(INFO, !quiet) << "Updated joints for '" << object->GetName()
                           << "'";
      break;
    }
    case ObjectWorldUpdate::kUpdateObjectJoint: {
      INTR_ASSIGN_OR_RETURN(
          const object_world::KinematicObject* object,
          object_world::HandleUpdateObjectJointRequest(
              *object_world, object_world_update.update_object_joint()));
      LOG_IF(INFO, !quiet)
          << "Updated joint '"
          << object_world_update.update_object_joint().joint_name() << "' for '"
          << object->GetName() << "'";
      break;
    }
    case ObjectWorldUpdate::kUpdateObjectProperties: {
      INTR_ASSIGN_OR_RETURN(
          const object_world::WorldObject* object,
          object_world::HandleUpdateObjectPropertiesRequest(
              *object_world, object_world_update.update_object_properties(),
              &geolib));
      LOG_IF(INFO, !quiet) << "Updated properties for '" << object->GetName()
                           << "'";
      break;
    }
    case ObjectWorldUpdate::kCreateObject: {
      const intrinsic_proto::world::CreateObjectRequest& create_object_request =
          object_world_update.create_object();
      if (create_object_request.object_spec_case() !=
          intrinsic_proto::world::CreateObjectRequest::
              kCreateSingleEntityObject) {
        return absl::InvalidArgumentError(
            "Invalid CreateObjectRequest: only single entity objects "
            "supported.");
      }
      INTR_ASSIGN_OR_RETURN(const object_world::WorldObject* object,
                            object_world::HandleCreateObjectRequest(
                                *object_world, create_object_request, &geolib));
      LOG_IF(INFO, !quiet) << "Created new object '" << object->GetName()
                           << "'";
      break;
    }
    case ObjectWorldUpdate::kUpdateEntityProperties: {
      const intrinsic_proto::world::UpdateEntityPropertiesRequest&
          update_entity_properties_request =
              object_world_update.update_entity_properties();
      INTR_ASSIGN_OR_RETURN(
          const AttachmentEntityId attachment_id,
          object_world::HandleUpdateEntityPropertiesRequest(
              *object_world, update_entity_properties_request, &geolib));

      INTR_ASSIGN_OR_RETURN(
          const object_world::WorldObject* object,
          GetObjectByReference(
              *object_world,
              update_entity_properties_request.entity().reference()));
      LOG_IF(INFO, !quiet) << "Updated entity properties for entity: "
                           << attachment_id
                           << " (parent object: " << object->GetName() << ")";
      break;
    }
    case ObjectWorldUpdate::kUpdateCollisionSettings: {
      const intrinsic_proto::world::UpdateCollisionSettingsRequest&
          update_collision_settings_request =
              object_world_update.update_collision_settings();
      INTR_RETURN_IF_ERROR(object_world::HandleUpdateCollisionSettingsRequest(
          *object_world, update_collision_settings_request))
          << "Failed to update collision settings";
      break;
    }
    case ObjectWorldUpdate::kDeleteFrame: {
      const intrinsic_proto::world::DeleteFrameRequest& delete_frame_request =
          object_world_update.delete_frame();
      INTR_RETURN_IF_ERROR(object_world::HandleDeleteFrameRequest(
          *object_world, delete_frame_request))
          << "Failed to delete frame";
      break;
    }
    case ObjectWorldUpdate::kDeleteObject: {
      const intrinsic_proto::world::DeleteObjectRequest& delete_object_request =
          object_world_update.delete_object();
      INTR_RETURN_IF_ERROR(object_world::HandleDeleteObjectRequest(
          *object_world, delete_object_request))
          << absl::StrCat("Failed to delete object. Request: ",
                          delete_object_request);
      break;
    }
    case ObjectWorldUpdate::kUpdateFrameProperties: {
      const intrinsic_proto::world::UpdateFramePropertiesRequest&
          update_frame_properties_request =
              object_world_update.update_frame_properties();
      INTR_ASSIGN_OR_RETURN(
          const object_world::Frame* frame,
          object_world::HandleUpdateFramePropertiesRequest(
              *object_world, update_frame_properties_request),
          _ << absl::StrCat("Failed to update frame properties. Request: ",
                            update_frame_properties_request));
      LOG_IF(INFO, !quiet) << "Updated frame properties for frame: "
                           << frame->GetName();
    } break;
    case ObjectWorldUpdate::UPDATE_NOT_SET: {
      return absl::InvalidArgumentError("Invalid ObjectWorldUpdate case");
    }
    default:
      return intrinsic::UnimplementedErrorBuilder()
             << "Update " << object_world_update.update_case()
             << " is not supported when composing resources";
      break;
  }

  return absl::OkStatus();
}

// Applies the given world updates to the resources. Only a limited set of
// object operations are supported.
absl::StatusOr<ComposeWorldResult> ApplyWorldUpdate(
    World initial_world, const ObjectWorldUpdates& updates,
    GeometryLibrary& geolib, const ComposeWorldOptions& options) {
  World current_world = std::move(initial_world);
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<ObjectWorld> object_world,
                        ObjectWorld::CreateView(current_world));

  std::vector<ComposeWorldResult::CompositionError> composition_errors;
  for (size_t index = 0; index < updates.updates_size(); ++index) {
    const auto& object_world_update = updates.updates(index);
    World world_before_update = current_world.Clone();
    absl::Status status = ApplyWorldUpdate(
        object_world.get(), object_world_update, geolib, options.quiet);
    if (!status.ok()) {
      if (options.update_policy !=
          ComposeWorldOptions::UpdatePolicy::kSkipFailedUpdates) {
        return status;
      }

      // Revert to the world before the failed update.
      current_world = std::move(world_before_update);
      INTR_ASSIGN_OR_RETURN(object_world,
                            ObjectWorld::CreateView(current_world));

      // Save the failed update status and proto.
      composition_errors.push_back({
          .problem =
              ComposeWorldResult::UpdateError{
                  .index = index,
                  .problem_update = object_world_update,
              },
          .status = status,
      });
    }
  }

  return ComposeWorldResult{.world = std::move(current_world),
                            .composition_errors = composition_errors};
}

absl::Status AddResourceObject(
    const intrinsic_proto::resources::GeometricResourceInstanceData&
        resource_instance_data,
    WorldObject* root, GeometryLibrary& geolib,
    const ComposeWorldOptions& options) {
  if (!resource_instance_data.has_world_fragment() &&
      !resource_instance_data.has_scene_object()) {
    LOG_IF(INFO, !options.quiet)
        << "Skipping resource instance '" << resource_instance_data.name()
        << "' because it doesn't have any geometry attached.";
    return absl::OkStatus();
  }
  LOG_IF(INFO, !options.quiet)
      << "Adding resource '" << resource_instance_data.name()
      << "' to world...";

  const scene_object::UpdatePolicy update_policy =
      options.update_policy ==
              ComposeWorldOptions::UpdatePolicy::kSkipFailedUpdates
          ? scene_object::UpdatePolicy::kSkipFailed
          : scene_object::UpdatePolicy::kDefault;

  // TODO(b/231187507): Consider expose both cases for NameIsGlobalAlias
  // or switch to non global alias case
  absl::StatusOr<WorldObject*> resource_object = root->CreateChildResource(
      resource_instance_data, WorldObjectNameType::kNameIsGlobalAlias,
      Pose3d(eigenmath::Vector3d(0, 0, 0)), &geolib, update_policy);
  LOG_IF(INFO, !options.quiet && resource_object.ok())
      << "Resource '" << resource_instance_data.name()
      << "' added with id: " << (*resource_object)->GetId();
  return resource_object.status();
}

}  // namespace

absl::StatusOr<ComposeWorldResult> ComposeResourceInstancesIntoWorld(
    const std::vector<
        intrinsic_proto::resources::GeometricResourceInstanceData>&
        resource_instance_data,
    const ObjectWorldUpdates& updates, GeometryLibrary& geolib,
    const ComposeWorldOptions& options) {
  const stats::ScopedSpan span("ComposeResourceInstancesIntoWorld");
  World world = World::CreateEmptyWorld();

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<ObjectWorld> object_world,
                        ObjectWorld::CreateView(world));
  // Start by adding each resource as a new object under root.
  INTR_ASSIGN_OR_RETURN(WorldObject * root,
                        object_world->GetObject(RootObjectName()));

  std::vector<ComposeWorldResult::CompositionError> add_object_errors;
  for (const auto& resource_instance_data : resource_instance_data) {
    absl::Status add_object_status =
        AddResourceObject(resource_instance_data, root, geolib, options);

    if (!add_object_status.ok()) {
      if (options.update_policy !=
          ComposeWorldOptions::UpdatePolicy::kSkipFailedUpdates) {
        return add_object_status;
      }

      absl::string_view resource_name = resource_instance_data.name();
      LOG(ERROR) << "Resource '" << resource_name
                 << "' was not added to world due to error: "
                 << add_object_status;
      add_object_errors.push_back({.problem = resource_instance_data.name(),
                                   .status = std::move(add_object_status)});
    }
  }

  absl::StatusOr<ComposeWorldResult> result =
      ApplyWorldUpdate(std::move(world), updates, geolib, options);

  if (result.ok()) {
    for (auto&& error : add_object_errors) {
      result->composition_errors.push_back(std::move(error));
    }
  }
  return result;
}

absl::StatusOr<ComposeWorldResult> ComposeResourceSetIntoWorld(
    const intrinsic_proto::resources::GeometricResourceSetData& data,
    GeometryLibrary& geolib, const ComposeWorldOptions& options) {
  return ComposeResourceInstancesIntoWorld(
      {data.instance_data().begin(), data.instance_data().end()},
      data.updates(), geolib, options);
}

}  // namespace intrinsic
