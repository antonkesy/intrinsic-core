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

#include "intrinsic/simulation/gazebo/world_sync_system.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/log_severity.h"
#include "absl/base/macros.h"
#include "absl/container/flat_hash_map.h"
#include "absl/functional/any_invocable.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/strip.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "google/rpc/code.pb.h"
#include "grpcpp/client_context.h"
#include "grpcpp/support/status.h"
#include "gz/common/Profiler.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/simulation/gazebo/components/world_component.h"
#include "intrinsic/simulation/gazebo/type_conversion.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_conversion_rpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/proto/v1/pubsub_world_updates.pb.h"
#include "intrinsic/world/util/tf_frame_util.h"

namespace intrinsic {
namespace simulation {

namespace {
using StreamInterface = grpc::ClientReaderWriterInterface<
    intrinsic_proto::simulation::v1::SimulatorSceneMessage,
    intrinsic_proto::simulation::v1::WorldMessage>;

// Converts object world entity id strings to numeric attachment entity ids that
// world to sdf populates in the `WorldEntityId` component.
// See intrinsic/world/objects/object_world_creation_utils.cc for corresponding
// conversion from numeric id to string id in the world.
// TODO(b/525398693): Remove once world to sdf conversion sets object world
// entity ids.
absl::StatusOr<AttachmentEntityId> GetAttachmentEntityIdFromObjectEntityId(
    std::string_view object_entity_id) {
  constexpr std::string_view kObjectEntityIdPrefix = "eid_";
  if (absl::StartsWith(object_entity_id, kObjectEntityIdPrefix)) {
    object_entity_id =
        absl::StripPrefix(object_entity_id, kObjectEntityIdPrefix);
  }

  uint32_t id_val = 0;
  if (absl::SimpleAtoi(object_entity_id, &id_val)) {
    return AttachmentEntityId(id_val);
  }
  return absl::InvalidArgumentError(absl::StrCat(
      "Extracted object_entity_id is not numeric: ", object_entity_id));
}

struct EntityAttachmentInfo {
  AttachmentEntityId parent_id;
  Pose3d parent_t_this;
  std::string frame_id;
  std::string parent_frame_id;
};

// Builds a map of entity attachment relationships from a collection of world
// objects.
// Returns a map where the key is the numeric attachment entity ID value
// (uint32_t) and the value is EntityAttachmentInfo containing the parent entity
// ID, relative pose, frame ID, and parent frame ID.
absl::StatusOr<absl::flat_hash_map<uint32_t, EntityAttachmentInfo>>
CreateEntityAttachmentGraphFromObjects(
    const std::vector<world::WorldObject>& objects) {
  size_t total_entities = 0;
  for (const auto& object : objects) {
    if (object.Proto().type() == intrinsic_proto::world::ObjectType::ROOT) {
      continue;
    }
    total_entities += object.Proto().entities().size();
  }

  absl::flat_hash_map<std::string, std::string> entity_id_to_frame_id;
  entity_id_to_frame_id.reserve(total_entities + 1);

  for (const auto& object : objects) {
    INTR_ASSIGN_OR_RETURN(
        auto object_frame_ids,
        GetFrameIdByEntityIdFromWorldObjectAndEntityFilter(
            object, world::ObjectEntityFilter::AllEntities()));
    entity_id_to_frame_id.merge(std::move(object_frame_ids));
  }

  absl::flat_hash_map<uint32_t, EntityAttachmentInfo> entity_attachments;
  entity_attachments.reserve(total_entities);

  for (const auto& object : objects) {
    if (object.Proto().type() == intrinsic_proto::world::ObjectType::ROOT) {
      continue;
    }
    for (const auto& [key, entity_proto] : object.Proto().entities()) {
      std::string_view id_str =
          !entity_proto.id().empty() ? entity_proto.id() : key;
      absl::StatusOr<AttachmentEntityId> entity_id =
          GetAttachmentEntityIdFromObjectEntityId(id_str);
      if (!entity_id.ok()) {
        LOG(ERROR) << "Object: " << object.Name() << ", Entity: " << id_str
                   << ": " << entity_id.status();
        continue;
      }
      AttachmentEntityId parent_id = kRootEntityId;
      if (entity_proto.parent_id() != RootEntityId().value()) {
        absl::StatusOr<AttachmentEntityId> parent_id_or =
            GetAttachmentEntityIdFromObjectEntityId(entity_proto.parent_id());
        if (!parent_id_or.ok()) {
          LOG(ERROR) << "Object: " << object.Name() << ", Entity: " << id_str
                     << ": " << parent_id_or.status();
          continue;
        }
        parent_id = *parent_id_or;
      }
      Pose3d parent_t_this = Pose3d::Identity();
      if (entity_proto.has_parent_t_this()) {
        INTR_ASSIGN_OR_RETURN(parent_t_this,
                              FromProto(entity_proto.parent_t_this()));
      }

      std::string frame_id;
      if (auto frame_it = entity_id_to_frame_id.find(id_str);
          frame_it != entity_id_to_frame_id.end()) {
        frame_id = frame_it->second;
      }
      ABSL_ASSERT(!frame_id.empty());

      std::string parent_frame_id;
      if (auto parent_it = entity_id_to_frame_id.find(entity_proto.parent_id());
          parent_it != entity_id_to_frame_id.end()) {
        parent_frame_id = parent_it->second;
      }
      ABSL_ASSERT(!parent_frame_id.empty());

      entity_attachments[entity_id->value()] = EntityAttachmentInfo{
          .parent_id = parent_id,
          .parent_t_this = parent_t_this,
          .frame_id = std::move(frame_id),
          .parent_frame_id = std::move(parent_frame_id),
      };
    }
  }
  return entity_attachments;
}

absl::Status RunSyncWorldSession(
    StreamInterface& stream, std::string_view simulator_name,
    absl::AnyInvocable<
        absl::Status(const intrinsic_proto::simulation::v1::SessionAck&)>&
        on_session_established) {
  intrinsic_proto::simulation::v1::SimulatorSceneMessage init_msg;
  init_msg.mutable_session_init()->set_simulator_name(simulator_name);
  if (!stream.Write(init_msg)) {
    return ToAbslStatus(stream.Finish());
  }

  intrinsic_proto::simulation::v1::WorldMessage ack_msg;
  if (!stream.Read(&ack_msg)) {
    return ToAbslStatus(stream.Finish());
  }

  if (!ack_msg.has_session_ack()) {
    return absl::InvalidArgumentError(
        "First received message was not session_ack.");
  }

  const intrinsic_proto::simulation::v1::SessionAck& ack =
      ack_msg.session_ack();
  INTR_RETURN_IF_ERROR(ToAbslStatus(ack.status()));

  INTR_RETURN_IF_ERROR(on_session_established(ack));

  LOG(INFO) << "Session initialized successfully for simulator "
            << simulator_name
            << ". Active world ID: " << ack.simulator_world_id()
            << ", Topic: " << ack.state_updates_pubsub_topic_name();

  intrinsic_proto::simulation::v1::WorldMessage msg;
  while (stream.Read(&msg)) {
    // Currently, we don't handle messages here. But need to keep the steam
    // open to indicate that the sync is active (over pubsub).
  }

  return ToAbslStatus(stream.Finish());
}

void RunSyncWorldLoop(
    StopToken stop_token, std::string_view simulator_name,
    absl::AnyInvocable<std::unique_ptr<StreamInterface>()> create_stream,
    absl::AnyInvocable<
        absl::Status(const intrinsic_proto::simulation::v1::SessionAck&)>
        on_session_established,
    absl::AnyInvocable<void(const absl::Status&)> on_disconnected) {
  LOG(INFO) << "Starting RunSyncWorldLoop";
  while (!stop_token.stop_requested()) {
    auto stream = create_stream();
    if (stream == nullptr) {
      on_disconnected(
          absl::InternalError("Failed to create SyncWorld stream."));
      break;
    }

    absl::Status run_session_status =
        RunSyncWorldSession(*stream, simulator_name, on_session_established);

    on_disconnected(run_session_status);

    bool is_recoverable_error =
        absl::IsUnavailable(run_session_status) ||
        absl::IsCancelled(run_session_status) ||
        run_session_status.code() == absl::StatusCode::kUnknown;

    if (is_recoverable_error && !stop_token.stop_requested()) {
      LOG(WARNING) << "Stream failed: " << run_session_status
                   << ". Retrying in 1s...";
      absl::Time start_time = absl::Now();
      while (!stop_token.stop_requested() &&
             absl::Now() - start_time < absl::Seconds(1)) {
        absl::SleepFor(absl::Milliseconds(50));
      }
      continue;
    }
    break;
  }
  LOG(INFO) << "Stopping RunSyncWorldLoop";
}

struct WorldEntityStateUpdate {
  Pose3d pose;
  std::string_view frame_id;
  std::string_view parent_frame_id;
};

// Constructs a PubsubWorldUpdates message from World entity updates, stamped
// with the current wall clock time.
// Wall clock time is used instead of sim time since the World service expects
// update timestamps to be monotonic since solution start. However, sim time
// resets every time with sim reset.
intrinsic_proto::world::v1::PubsubWorldUpdates CreatePubsubWorldUpdates(
    absl::Span<const WorldEntityStateUpdate> updates) {
  intrinsic_proto::world::v1::PubsubWorldUpdates updates_proto;
  auto* pubsub_update = updates_proto.add_updates();
  auto* tf_message = pubsub_update->mutable_transform();
  google::protobuf::Timestamp stamp = intrinsic::GetCurrentTimeProto();

  for (const auto& update_info : updates) {
    auto* transform_stamped = tf_message->add_transforms();
    *transform_stamped->mutable_header()->mutable_stamp() = stamp;
    transform_stamped->mutable_header()->set_frame_id(
        update_info.parent_frame_id);
    transform_stamped->set_child_frame_id(update_info.frame_id);

    auto* transform = transform_stamped->mutable_transform();
    *transform->mutable_translation() =
        ToVectorProto(update_info.pose.translation());
    *transform->mutable_rotation() = ToProto(update_info.pose.quaternion());
  }

  return updates_proto;
}
}  // namespace

absl::StatusOr<std::unique_ptr<WorldSyncSystem>> WorldSyncSystem::Create(
    std::string_view simulator_name,
    std::string_view expected_simulator_world_id,
    std::unique_ptr<WorldSyncStub> absl_nonnull world_sync_stub,
    std::shared_ptr<ObjectWorldServiceStub> absl_nonnull
    object_world_service_stub,
    PubSub* pubsub, absl::Duration refresh_time) {
  auto system = absl::WrapUnique(new WorldSyncSystem(
      expected_simulator_world_id, std::move(world_sync_stub),
      std::move(object_world_service_stub), refresh_time, pubsub,
      simulator_name));

  // Launch background thread to handle streaming and connection retry loop
  system->stream_connection_thread_ = std::make_unique<Thread>(
      [sys = system.get(), stub = system->world_sync_stub_.get(),
       name = system->simulator_name_,
       pubsub = system->pubsub_](StopToken stop_token) {
        RunSyncWorldLoop(
            stop_token, name,
            /*create_stream=*/
            [sys, stub, stop_token]() -> std::unique_ptr<StreamInterface> {
              auto ctx = std::make_unique<grpc::ClientContext>();
              ConfigureClientContext(ctx.get());
              ctx->set_deadline(absl::ToChronoTime(absl::InfiniteFuture()));
              absl::MutexLock l(sys->stream_mutex_);
              if (stop_token.stop_requested()) {
                return nullptr;
              }
              sys->stream_context_ = std::move(ctx);
              return stub->SyncWorld(sys->stream_context_.get());
            },
            /*on_session_established=*/
            [sys,
             pubsub](const intrinsic_proto::simulation::v1::SessionAck& ack)
                -> absl::Status {
              if (ack.simulator_world_id() !=
                  sys->expected_simulator_world_id_) {
                return absl::FailedPreconditionError(
                    absl::StrCat("Simulator world ID mismatch. Expected: ",
                                 sys->expected_simulator_world_id_,
                                 ", got: ", ack.simulator_world_id()));
              }
              absl::MutexLock l(sys->stream_mutex_);
              INTR_ASSIGN_OR_RETURN(
                  auto publisher,
                  pubsub->CreatePublisher(ack.state_updates_pubsub_topic_name(),
                                          {}));
              sys->stream_data_ = StreamData{
                  .simulator_world_id = ack.simulator_world_id(),
                  .pose_update_pubsub_topic =
                      ack.state_updates_pubsub_topic_name(),
                  .publisher = std::move(publisher),
                  .pose_cache_invalidated = true,
              };
              return absl::OkStatus();
            },
            /*on_disconnected=*/
            [sys](const absl::Status& status) {
              absl::MutexLock l(sys->stream_mutex_);
              if (status.ok()) {
                LOG(ERROR) << "SyncWorld stream closed with Ok, this should "
                              "not happen in practice!";
                sys->stream_data_ =
                    absl::AbortedError("SyncWorld stream was closed.");
              } else {
                LOG(WARNING) << "SyncWorld stream disconnected: " << status;
                sys->stream_data_ = absl::Status(
                    status.code(), absl::StrCat("SyncWorld stream failed: ",
                                                status.message()));
              }
            });
      });

  return system;
}

WorldSyncSystem::WorldSyncSystem(
    std::string_view expected_simulator_world_id,
    std::unique_ptr<WorldSyncStub> world_sync_stub,
    std::shared_ptr<ObjectWorldServiceStub> object_world_service_stub,
    absl::Duration refresh_time, PubSub* pubsub,
    std::string_view simulator_name)
    : gz::sim::System(),
      expected_simulator_world_id_(expected_simulator_world_id),
      simulator_name_(simulator_name),
      world_sync_stub_(std::move(world_sync_stub)),
      object_world_service_stub_(std::move(object_world_service_stub)),
      time_between_refreshes_(refresh_time),
      pubsub_(pubsub) {}

WorldSyncSystem::~WorldSyncSystem() {
  if (stream_connection_thread_ != nullptr) {
    stream_connection_thread_->request_stop();
  }
  {
    absl::MutexLock l(stream_mutex_);
    if (stream_context_ != nullptr) {
      stream_context_->TryCancel();
    }
  }
}

absl::Status WorldSyncSystem::stream_connection_status() {
  absl::MutexLock l(stream_mutex_);
  return stream_data_.status();
}

absl::Status WorldSyncSystem::RebuildPoseCache(
    const gz::sim::EntityComponentManager& ecm) {
  std::string simulator_world_id;
  {
    absl::MutexLock l(stream_mutex_);
    if (!stream_data_.ok()) {
      return stream_data_.status();
    }
    simulator_world_id = stream_data_->simulator_world_id;
  }

  LOG(INFO) << "Rebuilding pose cache...";

  world::ObjectWorldClient object_world_client(simulator_world_id,
                                               object_world_service_stub_);
  INTR_ASSIGN_OR_RETURN(std::vector<world::WorldObject> objects,
                        object_world_client.ListObjects());

  INTR_ASSIGN_OR_RETURN(
      (absl::flat_hash_map<uint32_t, EntityAttachmentInfo> entity_attachments),
      CreateEntityAttachmentGraphFromObjects(objects));

  // Collect all entities that correspond to world entities...
  absl::flat_hash_map<uint32_t, gz::sim::Entity> world_entity_map;
  ecm.Each<intrinsic::simulation::WorldEntityId>(
      [&world_entity_map](
          const gz::sim::Entity& entity,
          const intrinsic::simulation::WorldEntityId* entity_id) {
        world_entity_map[entity_id->Data()] = entity;
        return true;
      });

  // For each entity, find its attachment info, walk up the tree until
  // you find a representative ancestor, and then store the appropriate
  // information in the pose cache.
  pose_cache_.clear();
  name_cache_.clear();
  ecm.Each<intrinsic::simulation::WorldEntityId>(
      [this, &entity_attachments, &world_entity_map, &ecm](
          const gz::sim::Entity& entity,
          const intrinsic::simulation::WorldEntityId* entity_id) {
        auto it = entity_attachments.find(entity_id->Data());
        if (it == entity_attachments.end()) {
          const bool is_model =
              ecm.Component<gz::sim::components::Model>(entity) != nullptr;
          LOG_IF_EVERY_N_SEC(ERROR, !is_model, 1)
              << "Entity with world ID " << entity_id->Data()
              << " not found in object world entities!";
          return true;
        }

        LOG_IF(WARNING,
               ecm.Component<gz::sim::components::Pose>(entity) == nullptr)
            << "Entity with world ID " << entity_id->Data()
            << " has no pose component in gazebo ecm!";

        AttachmentEntityId world_parent = it->second.parent_id;
        EntityPoseCacheEntry pose_cache_entry{
            .frame_id = it->second.frame_id,
            .direct_parent_frame_id = it->second.parent_frame_id,
        };

        while (world_parent != kRootEntityId &&
               !world_entity_map.contains(world_parent.value())) {
          auto parent_it = entity_attachments.find(world_parent.value());
          if (parent_it == entity_attachments.end()) {
            LOG_EVERY_N_SEC(ERROR, 1)
                << "Parent entity with world ID " << world_parent.value()
                << " not found in object world entities!";
            break;
          }

          pose_cache_entry.world_parent_t_direct_parent =
              parent_it->second.parent_t_this *
              pose_cache_entry.world_parent_t_direct_parent;
          world_parent = parent_it->second.parent_id;
        }

        if (world_parent != kRootEntityId) {
          pose_cache_entry.world_parent_entity =
              world_entity_map[world_parent.value()];
        } else {
          pose_cache_entry.world_parent_entity = gz::sim::kNullEntity;
        }

        pose_cache_[entity] = std::move(pose_cache_entry);
        return true;
      });
  return absl::OkStatus();
}

std::string WorldSyncSystem::GetScopedName(
    gz::sim::Entity entity, const gz::sim::EntityComponentManager& ecm) {
  auto it = name_cache_.find(entity);
  if (it != name_cache_.end()) {
    return it->second;
  }
  std::string name = gz::sim::scopedName(entity, ecm, "::", false);
  name_cache_[entity] = name;
  return name;
}

void WorldSyncSystem::PreUpdate(const gz::sim::UpdateInfo& info,
                                gz::sim::EntityComponentManager& ecm) {
  bool pose_cache_invalidated = false;
  {
    absl::MutexLock l(stream_mutex_);
    if (!stream_data_.ok()) {
      return;
    }
    pose_cache_invalidated = stream_data_->pose_cache_invalidated;
  }

  std::vector<gz::sim::Entity> dirty_entities =
      ecm.EntitiesByComponents(WorldDirtyFlag(true));
  if (!dirty_entities.empty()) {
    pose_cache_invalidated = true;
  }
  if (pose_cache_invalidated) {
    INTR_RETURN_IF_ERROR(RebuildPoseCache(ecm))
        .With(intrinsic::ExtraMessage() << "Failed to rebuild pose cache.")
        .LogEvery(absl::LogSeverity::kError, absl::Seconds(1))
        .With([this](const absl::Status& s) {
          absl::MutexLock l(stream_mutex_);
          if (stream_data_.ok()) {
            stream_data_->pose_cache_invalidated = true;
          }
        });

    absl::MutexLock l(stream_mutex_);
    if (stream_data_.ok()) {
      stream_data_->pose_cache_invalidated = false;
    }
  }

  for (const auto& e : dirty_entities) {
    WorldDirtyFlag* dirty_flag_component = ecm.Component<WorldDirtyFlag>(e);
    if (dirty_flag_component == nullptr) {
      LOG(ERROR)
          << "Could not reset dirty flag for entity! Performance will suffer.";
    } else {
      dirty_flag_component->Data() = false;
    }
  }
}

void WorldSyncSystem::PostUpdate(const gz::sim::UpdateInfo& info,
                                 const gz::sim::EntityComponentManager& ecm) {
  GZ_PROFILE("WorldSyncSystem::PostUpdate");

  // TODO: b/290264637 - We may want to provide a configurable clock for
  // managing the frequency of these updates, since we would generate poses at
  // different rates in different contexts.
  absl::Duration sim_time = absl::FromChrono(info.simTime);
  if (sim_time - last_refresh_sim_time_elapsed_ < time_between_refreshes_) {
    return;
  }

  if (pose_cache_.empty()) {
    return;
  }

  {
    absl::MutexLock l(stream_mutex_);
    if (!stream_data_.ok() || stream_data_->pose_cache_invalidated) {
      return;
    }
  }

  std::vector<WorldEntityStateUpdate> updates;
  ecm.Each<intrinsic::simulation::WorldEntityId>(
      [this, &updates, &ecm](
          const gz::sim::Entity& entity,
          const intrinsic::simulation::WorldEntityId* entity_id) {
        // Does this entity have a world pose? If so we can update its pose in
        // the Intrinsic world.
        if (ecm.Component<gz::sim::components::Pose>(entity) == nullptr) {
          return true;
        }

        INTR_ASSIGN_OR_RETURN(
            const intrinsic::Pose3d world_pose,
            GzToIntrinsicChecked(gz::sim::worldPose(entity, ecm)),
            _.With(intrinsic::ExtraMessage()
                   << "Entity '" << GetScopedName(entity, ecm)
                   << "': WorldEntity '" << entity_id->Data() << "': ")
                .LogEvery(absl::LogSeverity::kError, absl::Seconds(1))
                .With([](const absl::Status& s) { return true; }));

        // Models have poses and world entity IDs, but they are not in the
        // attachment graph, so we can avoid warning about them here since we
        // use world poses and will therefore resolve any parent-child
        // relationships in gazebo that affect our world poses.
        if (!pose_cache_.contains(entity)) {
          const bool is_model =
              ecm.Component<gz::sim::components::Model>(entity) != nullptr;
          LOG_IF_EVERY_N_SEC(WARNING, !is_model, 1)
              << "Unable to find non-model entity '"
              << GetScopedName(entity, ecm) << "' in pose cache!";
          return true;
        }

        intrinsic::Pose3d updated_pose;
        const EntityPoseCacheEntry& cache_entry = pose_cache_[entity];
        // Parent is root, so
        // world_parent_t_direct_parent * updated_pose = world_pose
        // ... meaning
        if (cache_entry.world_parent_entity == gz::sim::kNullEntity) {
          updated_pose =
              cache_entry.world_parent_t_direct_parent.inverse() * world_pose;
        } else {
          // Parent is another entity. Let's find the world pose to that entity.
          const gz::sim::Entity& parent_entity =
              cache_entry.world_parent_entity;

          if (ecm.Component<gz::sim::components::Pose>(parent_entity) ==
              nullptr) {
            LOG_EVERY_N_SEC(ERROR, 1)
                << "Unable to find pose for parent entity '"
                << GetScopedName(parent_entity, ecm) << "'! Assuming root.";
            // Same as above
            updated_pose =
                cache_entry.world_parent_t_direct_parent.inverse() * world_pose;
          } else {
            INTR_ASSIGN_OR_RETURN(
                const intrinsic::Pose3d parent_world_pose,
                GzToIntrinsicChecked(gz::sim::worldPose(parent_entity, ecm)),
                _.With(intrinsic::ExtraMessage()
                       << "Parent entity '" << GetScopedName(parent_entity, ecm)
                       << "': ")
                    .LogEvery(absl::LogSeverity::kError, absl::Seconds(1))
                    .With([](const absl::Status& s) { return true; }));

            // We have the world pose of the parent, so
            // world_pose = parent_world_pose *
            //              world_parent_t_direct_parent *
            //              updated_pose
            // ... meaning
            updated_pose =
                (parent_world_pose * cache_entry.world_parent_t_direct_parent)
                    .inverse() *
                world_pose;
          }
        }

        updates.push_back(WorldEntityStateUpdate{
            .pose = updated_pose,
            .frame_id = cache_entry.frame_id,
            .parent_frame_id = cache_entry.direct_parent_frame_id,
        });
        return true;
      });

  last_refresh_sim_time_elapsed_ = sim_time;

  // Updates can be empty if spawned objects were removed with an outfeed.
  if (updates.empty()) {
    return;
  }

  intrinsic_proto::world::v1::PubsubWorldUpdates updates_proto =
      CreatePubsubWorldUpdates(updates);

  {
    absl::MutexLock l(stream_mutex_);
    if (stream_data_.ok()) {
      if (auto status = stream_data_->publisher.Publish(updates_proto);
          !status.ok()) {
        LOG_EVERY_N_SEC(ERROR, 1)
            << "Failed to publish pose updates: " << status;
      }
    }
  }
}

}  // namespace simulation
}  // namespace intrinsic
