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

#include "intrinsic/world/service/worldservice_impl.h"

#include <cstddef>
#include <cstdint>
#include <future>  // NOLINT(build/c++11)
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/api/io.h"
#include "intrinsic/geometry/api/renderable_generation.h"
#include "intrinsic/geometry/compatibility/io.h"
#include "intrinsic/geometry/proto/geometry_service_types.pb.h"
#include "intrinsic/geometry/storage/geometry_library.h"
#include "intrinsic/geometry/storage/in_memory_storage.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/stats/scoped_span.h"
#include "intrinsic/util/status/ret_check_grpc.h"
#include "intrinsic/util/status/status_builder_grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_macros_grpc.h"
#include "intrinsic/util/unique_id.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/component/ppr_component.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/service/world_mutex.h"
#include "intrinsic/world/service/world_service.pb.h"
#include "intrinsic/world/service/world_storage.h"
#include "intrinsic/world/util/entity_search_util.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

using ::intrinsic_proto::world::EntitySearchCriteria;
using PPRComponentProto = ::intrinsic_proto::world::PPRComponent;
using ::intrinsic_proto::world::internal::CreateWorldRequest;
using ::intrinsic_proto::world::internal::EntityWithMetadata;
using ::intrinsic_proto::world::internal::GetIkSolutionRequest;
using ::intrinsic_proto::world::internal::GetWorldRequest;
using ::intrinsic_proto::world::internal::SingleIKSolution;
using ::intrinsic_proto::world::internal::WorldWithMetadata;

namespace {

// Creates a MapGeometryLibrary from an iterable collection of
// GeometryWithMetadata protos.
template <typename GeometryProtos>
absl::StatusOr<std::unique_ptr<MapGeometryLibrary>> CreateMapGeometryLibrary(
    const GeometryProtos& geometry_protos) {
  MapGeometryLibrary::RawMaps geo_maps;
  for (const auto& geo_proto : geometry_protos) {
    if (geo_proto.has_geometry_storage_refs_v0()) {
      const std::string& geo_id =
          geo_proto.geometry_storage_refs_v0().fingerprint();
      if (geo_id.empty()) {
        return absl::InvalidArgumentError(
            "Encountered Geometry without fingerprint");
      }
      if (geo_maps.geometry_map.contains(geo_id)) {
        return absl::InvalidArgumentError(
            absl::StrCat("Found duplicate geometry ids: ", geo_id));
      }
      INTR_ASSIGN_OR_RETURN(
          geo_maps.geometry_map[geo_id],
          geometry_compatibility::ToGeometry(geo_proto.geometry_v0()));
    }
    if (geo_proto.has_geometry_storage_refs()) {
      if (!geo_proto.has_inline_geometry()) {
        return absl::InvalidArgumentError(
            "Encountered Geometry with v1 storage refs but without "
            "inline geometry. We expect the inline geometry to be "
            "populated to store geometries for the corresponding v1 "
            "storage refs.");
      }
      INTR_ASSIGN_OR_RETURN(const Geometry geo,
                            ToGeometry(geo_proto.inline_geometry()));

      const std::string& exact_ref =
          geo_proto.geometry_storage_refs().exact_geometry_ref();
      if (exact_ref.empty()) {
        return absl::InvalidArgumentError(
            "Encountered Geometry without exact geometry ref");
      }
      if (geo_maps.exact_geometry_map.contains(exact_ref)) {
        return absl::InvalidArgumentError(
            absl::StrCat("Found duplicate exact geometry refs: ", exact_ref));
      }
      geo_maps.exact_geometry_map.try_emplace(exact_ref,
                                              geo.GetExactGeometry());

      const std::string& renderable_ref =
          geo_proto.geometry_storage_refs().renderable_ref();
      if (!renderable_ref.empty()) {
        if (geo_maps.renderable_map.contains(renderable_ref)) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Found duplicate renderable refs: ", renderable_ref));
        }
        INTR_ASSIGN_OR_RETURN(geo_maps.renderable_map[renderable_ref],
                              GetOrGenerateRenderable(geo));
      }
    }
  }

  return GetMapGeometryLibrary(std::move(geo_maps));
}

}  // namespace

absl::StatusOr<std::unique_ptr<WorldServiceImpl>>
WorldServiceImpl::CreateService(
    WorldStorage& world_storage,
    std::optional<ObjectWorldCompatibilityFunc> compat_func,
    GeometryLibrary& geo_lib) {
  std::future<WorldStorage*> instant_world_storage = std::async(
      std::launch::deferred, [&world_storage]() { return &world_storage; });
  std::future<GeometryLibrary*> instant_geo_lib =
      std::async(std::launch::deferred, [&geo_lib]() { return &geo_lib; });
  return absl::WrapUnique(new WorldServiceImpl(std::move(instant_world_storage),
                                               compat_func,
                                               std::move(instant_geo_lib)));
}

absl::StatusOr<std::unique_ptr<WorldServiceImpl>>
WorldServiceImpl::CreateService(
    std::future<WorldStorage*> world_storage,
    std::optional<ObjectWorldCompatibilityFunc> compat_func,
    std::future<GeometryLibrary*> geo_lib) {
  return absl::WrapUnique(new WorldServiceImpl(
      std::move(world_storage), compat_func, std::move(geo_lib)));
}

WorldStorage* WorldServiceImpl::WorldStore() {
  absl::MutexLock l(world_storage_mutex_);
  if (world_storage_ == nullptr && world_storage_future_.valid()) {
    world_storage_ = world_storage_future_.get();
    CHECK(world_storage_ != nullptr) << "WorldStorage future returned nullptr";
  }
  return world_storage_;
}

GeometryLibrary* WorldServiceImpl::GeoLib() {
  absl::MutexLock l(geo_lib_mutex_);
  if (geo_lib_ == nullptr && geo_lib_future_.valid()) {
    geo_lib_ = geo_lib_future_.get();
    CHECK(geo_lib_ != nullptr) << "GeometryLibrary future returned nullptr";
  }
  return geo_lib_;
}

grpc::Status WorldServiceImpl::CreateWorld(
    grpc::ServerContext* context,
    const intrinsic_proto::world::internal::CreateWorldRequest* request,
    intrinsic_proto::world::internal::WorldWithMetadata* response) {
  const stats::ScopedSpan span("WorldService/CreateWorld", context);

  const std::string world_id = absl::StrCat("World_", WebSafeUuid());

  std::shared_ptr<WorldAndMutex> world_ptr;

  if (request->has_world() && request->world().has_world_data()) {
    INTR_ASSIGN_OR_RETURN_GRPC(
        World world, World::Deserialize(request->world().world_data()));

    // If we provide geometry then reserialize them with the default geolib.
    if (!request->geometries().empty()) {
      INTR_ASSIGN_OR_RETURN_GRPC(
          std::unique_ptr<GeometryLibrary> geolib,
          CreateMapGeometryLibrary(request->geometries()));

      for (const GeometryEntityId entity_id :
           world.GetTypedEntityIds<GeometryEntityId>()) {
        INTR_ASSIGN_OR_RETURN_GRPC(
            GeometryComponent * geo_component,
            world.GetComponentByEntityId<GeometryComponent>(entity_id));

        // Reserialize geometries into our GeoLib().
        for (const std::string& geo_set_name :
             geo_component->GetGeometryNames()) {
          INTR_ASSIGN_OR_RETURN_GRPC(
              NamedGeometrySet geos,
              geo_component->GetGeometry(geo_set_name, geolib->Deserializer()));

          NamedGeometryProtoSet geo_protos;
          for (const auto& [geo_name, geo] : geos) {
            INTR_ASSIGN_OR_RETURN_GRPC(geo_protos[geo_name],
                                       ToProto(geo, &GeoLib()->Serializer()));
          }

          geo_component->SetGeometry(geo_set_name, geo_protos);
        }
      }
    }

    INTR_ASSIGN_OR_RETURN_GRPC(
        world_ptr, WorldStore()->AddWorld(world_id, std::move(world),
                                          /*skip_compat_check=*/false));
  } else if (!request->geometries().empty()) {
    return InvalidArgumentErrorBuilderGrpc().LogError()
           << "No world was specified but geometry was specified";
  } else {
    // If we didn't have world data and we don't have geometry we are in the
    // default empty world case.
    INTR_ASSIGN_OR_RETURN_GRPC(
        world_ptr, WorldStore()->AddWorld(world_id, World::CreateEmptyWorld(),
                                          /*skip_compat_check=*/true));
  }

  if (!request->user_tag().empty()) {
    absl::WriterMutexLock lock(*world_ptr->mtx);
    world_ptr->user_tag = request->user_tag();
  }

  {
    absl::ReaderMutexLock created_world_lock(*world_ptr->mtx);
    const World& world = world_ptr->world;
    INTR_RET_CHECK_GRPC(!world_ptr->world_structure_hash.empty());
    // Should not need to load since we just constructed with a pointer.
    INTR_ASSIGN_OR_RETURN_GRPC(
        *response, WorldToProto(world_id, world_ptr->world_structure_hash,
                                world_ptr->user_tag, world));
  }

  return grpc::Status::OK;
}

grpc::Status WorldServiceImpl::GetWorld(
    grpc::ServerContext* context,
    const intrinsic_proto::world::internal::GetWorldRequest* request,
    intrinsic_proto::world::internal::WorldWithMetadata* response) {
  const stats::ScopedSpan span("WorldService/GetWorld", context);

  INTR_ASSIGN_OR_RETURN_GRPC(std::shared_ptr<WorldAndMutex> world_ptr,
                             WorldStore()->GetWorld(request->world_id()));

  absl::MutexLock lock(*world_ptr->mtx);
  World world = (*world_ptr)->Clone();

  // TODO: b/325289138 -- Remove this. :( By default we need to remove the
  // coordinate frames from the collections members entity lists since this
  // creates a world that is incompatible with older versions of the object
  // world APIs.
  if (!request->keep_frame_entity_collection_members_hack()) {
    for (const CollectionsEntityId collections_id :
         world.GetTypedEntityIds<CollectionsComponentType>()) {
      INTR_ASSIGN_OR_RETURN_GRPC(
          CollectionsComponent * collections_component,
          world.GetComponentByEntityId<CollectionsComponent>(collections_id));

      for (const CollectionsMemberEntityId frame_member_id :
           collections_component->GetCollectionMembers(
               CollectionsComponent::kCoordinateFrames)) {
        INTR_ASSIGN_OR_RETURN_GRPC(
            CollectionsMemberComponent * collections_member_component,
            world.GetComponentByEntityId<CollectionsMemberComponent>(
                frame_member_id));

        INTR_RETURN_IF_ERROR_GRPC(
            collections_member_component->DeleteParentCollection(
                collections_id, CollectionsComponent::kCoordinateFrames));

        if (collections_member_component->GetParentCollectionsIdToTypesMap()
                .empty()) {
          INTR_ASSIGN_OR_RETURN_GRPC(WorldEntity * entity,
                                     world.GetEntityById(frame_member_id));
          INTR_RETURN_IF_ERROR_GRPC(
              entity->RemoveComponent<CollectionsMemberComponent>());
        }
      }

      INTR_RETURN_IF_ERROR_GRPC(collections_component->SetCollectionMembers(
          CollectionsComponent::kCoordinateFrames, {}));

      if (collections_component->GetAllCollectionMembers().empty()) {
        INTR_ASSIGN_OR_RETURN_GRPC(WorldEntity * entity,
                                   world.GetEntityById(collections_id));
        INTR_RETURN_IF_ERROR_GRPC(
            entity->RemoveComponent<CollectionsComponent>());
      }
    }

    // TODO: b/340091582 -- Remove this. :( Old ICON sim bus hardware modules
    // used the presence of `ResourceName` on entities to determine which
    // objects matched that which was passed to the resource. That's no longer
    // the case after cl/664909949, but old ICON resources may still be deployed
    // that have this behavior. Keep this here until we've verified that we've
    // released a breaking change that includes the aforementioned cl.
    for (const PPREntityId ppr_id :
         world.GetTypedEntityIds<PPRComponentType>()) {
      if (world.ValidateEntity<CollectionsEntityId>(ppr_id).ok()) {
        continue;
      }

      INTR_ASSIGN_OR_RETURN_GRPC(
          PPRComponent * ppr_component,
          world.GetComponentByEntityId<PPRComponent>(ppr_id));
      INTR_ASSIGN_OR_RETURN_GRPC(PPRComponentProto ppr_proto,
                                 ppr_component->ToProto());
      ppr_proto.clear_resource_name();
      INTR_RETURN_IF_ERROR_GRPC(ppr_component->UpdateFromProto(ppr_proto));
    }
  }

  INTR_ASSIGN_OR_RETURN_GRPC(
      *response,
      WorldToProto(request->world_id(), world_ptr->world_structure_hash,
                   world_ptr->user_tag, world));
  return grpc::Status::OK;
}

absl::StatusOr<EntityWithMetadata> WorldServiceImpl::GetEntity(
    absl::string_view world_id, const EntitySearchCriteria& entity) {
  INTR_ASSIGN_OR_RETURN(std::shared_ptr<WorldAndMutex> world_ptr,
                        WorldStore()->GetWorld(world_id));
  absl::ReaderMutexLock lock(*world_ptr->mtx);
  const World& world = world_ptr->world;

  INTR_ASSIGN_OR_RETURN(EntityId entity_id, GetSingleEntity(world, entity));
  INTR_ASSIGN_OR_RETURN(const WorldEntity* world_entity,
                        world.GetEntityById(entity_id));
  return EntityToProto(world_id, entity_id, *world_entity);
}

absl::StatusOr<WorldWithMetadata> WorldServiceImpl::WorldToProto(
    absl::string_view world_id, absl::string_view world_structure_hash,
    absl::string_view user_tag, const World& world) {
  WorldWithMetadata response;
  response.set_world_id(world_id);
  response.set_world_structure_hash(world_structure_hash);
  response.set_user_tag(user_tag);

  INTR_ASSIGN_OR_RETURN(*response.mutable_world_data(), world.Serialize());
  return response;
}

absl::StatusOr<EntityWithMetadata> WorldServiceImpl::EntityToProto(
    absl::string_view world_id, EntityId entity_id, const WorldEntity& entity) {
  EntityWithMetadata response;
  response.set_world_id(world_id);
  response.set_entity_id(entity_id.value());

  INTR_ASSIGN_OR_RETURN(*response.mutable_entity_data(), entity.ToProto());
  return response;
}

grpc::Status WorldServiceImpl::GetIkSolution(
    grpc::ServerContext* context, const GetIkSolutionRequest* request,
    SingleIKSolution* response) {
  const stats::ScopedSpan span("WorldService/GetIkSolution", context);

  INTR_ASSIGN_OR_RETURN_GRPC(std::shared_ptr<WorldAndMutex> world_ptr,
                             WorldStore()->GetWorld(request->world_id()));
  absl::WriterMutexLock lock(*world_ptr->mtx);

  INTR_ASSIGN_OR_RETURN_GRPC(
      AttachmentEntityId root_id,
      (*world_ptr)
          ->ValidateEntity<AttachmentEntityId>(EntityId(request->root_id())),
      _.LogError() << " for root entity");

  INTR_ASSIGN_OR_RETURN_GRPC(
      AttachmentEntityId tip_id,
      (*world_ptr)
          ->ValidateEntity<AttachmentEntityId>(EntityId(request->tip_id())),
      _.LogError() << " for tip entity");

  INTR_ASSIGN_OR_RETURN_GRPC(
      std::vector<std::unique_ptr<CartesianKinematicView>> cart_views,
      (*world_ptr)->GetCartesianKinematicViews(root_id, tip_id));
  response->set_world_id(request->world_id());

  INTR_ASSIGN_OR_RETURN_GRPC(Pose3d pose, FromProto(request->root_t_tip()),
                             _.LogError());

  // Loop through all the possible cart views and check each one for a possible
  // solution to the inverse kinematics based on the given solvers. The first
  // answer is the one we will return.
  for (const std::unique_ptr<CartesianKinematicView>& cart_view : cart_views) {
    if (cart_view == nullptr) {
      continue;
    }

    absl::StatusOr<std::string> solver_key = cart_view->GetIKSolverKey();
    if (!solver_key.ok() || solver_key->empty()) {
      if (cart_views.size() == 1) {
        return NotFoundErrorBuilderGrpc().LogError()
               << "No solver for given chain";
      }
      continue;
    }

    std::shared_ptr<DofKinematicView> dof_view = cart_view->GetDofView();
    if (dof_view == nullptr) {
      return InternalErrorBuilderGrpc().LogError()
             << "CartesianKinematicView has null DofKinematicView";
    }

    auto ik_solutions =
        cart_view->GetIkSolutions(pose, dof_view->GetDofValues());

    if (!ik_solutions) {
      if (cart_views.size() == 1) {
        return NotFoundErrorBuilderGrpc().LogError()
               << "Could not find viable IK solution";
      }
      continue;
    }

    std::optional<eigenmath::VectorXd> single_solution =
        ik_solutions->GetSingleSolution();
    if (!single_solution.has_value()) {
      if (cart_views.size() == 1) {
        return NotFoundErrorBuilderGrpc().LogError()
               << "Could not find viable IK solution";
      }
      continue;
    }

    const eigenmath::VectorXd& dof_vals = *single_solution;
    if (request->update_robot_to_solution()) {
      LOG(INFO) << "Robot joint values updated to IK solution.";
      // Set the IK solution to the dof view to jog the robot via FK.
      INTR_RETURN_IF_ERROR_GRPC(dof_view->SetDofValues(dof_vals, true));
      world_ptr->UpdateTimestamp();

      WorldStore()->MarkWorldAsChanged(request->world_id(), world_ptr,
                                       /*has_state_change=*/true);
    }

    // Set the result.
    const std::vector<JointEntityId>& dofs = dof_view->GetJointEntityIds();
    if (static_cast<int>(dofs.size()) != dof_vals.size()) {
      return InternalErrorBuilderGrpc().LogError()
             << "Mismatch between DOF count (" << dofs.size()
             << ") and IK solution vector size (" << dof_vals.size() << ")";
    }
    for (size_t i = 0; i < dofs.size(); ++i) {
      (*response->mutable_joint_vals())[dofs[i].value()] = dof_vals[i];
    }
    return grpc::Status::OK;
  }

  return NotFoundErrorBuilderGrpc().LogError()
         << "Could not find viable IK solution";
}

}  // namespace intrinsic
