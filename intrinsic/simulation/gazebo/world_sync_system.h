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

#ifndef INTRINSIC_SIMULATION_GAZEBO_WORLD_SYNC_SYSTEM_H_
#define INTRINSIC_SIMULATION_GAZEBO_WORLD_SYNC_SYSTEM_H_

#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "grpcpp/client_context.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "intrinsic/math/pose3.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/simulation/service/proto/v1/simulator_world_sync.grpc.pb.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"

namespace intrinsic {
namespace simulation {

// WorldSyncSystem class monitors simulation entity pose updates and syncs them
// with the simulation service by establishing a session over SimulatorWorldSync
// gRPC stream.
class WorldSyncSystem : public gz::sim::System,
                        public gz::sim::ISystemPreUpdate,
                        public gz::sim::ISystemPostUpdate {
 public:
  using ObjectWorldServiceStub =
      intrinsic_proto::world::ObjectWorldService::StubInterface;
  using WorldSyncStub =
      intrinsic_proto::simulation::v1::SimulatorWorldSync::StubInterface;

  // Creates a Gazebo System that connects to the SimulatorWorldSync service
  // to initialize a session, retrieve the simulator world and pubsub topic,
  // and publish updates.
  //
  // - The Gazebo ECS is assumed to be initialized from the simulator
  // world ID. This System fetches the simulator World objects from Object World
  // Service to establish a mapping between entities in the simulator world and
  // entities in the Gazebo ECS. This mapping is used to generate World pose
  // updates that are published over pubsub.
  //
  // @param simulator_name The unique name identifying this simulator instance.
  // This is forwarded to the SimulatorWorldSync Service when establishing a
  // SyncWorld stream.
  // @param expected_simulator_world_id The ID of the world Gazebo is expected
  // to sync. If this doesn't match the world ID sent by the Simulation Service
  // when establishing a SyncWorld stream, the stream is closed with an error.
  // @param world_sync_stub gRPC stub interface for the SimulatorWorldSync
  // service.
  // @param object_world_service_stub gRPC stub interface for ObjectWorld
  // service.
  // @param pubsub PubSub client to publish pose updates. Should outlive this
  // class.
  // @param refresh_time Interval at which Gazebo poses are synced.
  static absl::StatusOr<std::unique_ptr<WorldSyncSystem>> Create(
      std::string_view simulator_name,
      std::string_view expected_simulator_world_id,
      std::unique_ptr<WorldSyncStub> absl_nonnull world_sync_stub,
      std::shared_ptr<ObjectWorldServiceStub> absl_nonnull
      object_world_service_stub,
      PubSub* absl_nonnull pubsub,
      absl::Duration refresh_time = absl::Milliseconds(10));

  ~WorldSyncSystem() override;

  // Disallow copy and move.
  WorldSyncSystem(const WorldSyncSystem&) = delete;
  WorldSyncSystem& operator=(const WorldSyncSystem&) = delete;
  WorldSyncSystem(WorldSyncSystem&&) = delete;
  WorldSyncSystem& operator=(WorldSyncSystem&&) = delete;

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override;

  // Current SyncWorld stream connection status.
  absl::Status stream_connection_status() ABSL_LOCKS_EXCLUDED(stream_mutex_);

 private:
  // Used to have low-latency updates of poses during the simulation's
  // execution. This is the per-entity data that we need to maintain coherence.
  struct EntityPoseCacheEntry {
    // For a Gazebo entity that corresponds to a World entity, this is the
    // Gazebo entity that corresponds to the first ancestor of the world entity
    // that has a corresponding Gazebo entity. If this is gz::sim::kNullEntity,
    // then the world parent is root.
    gz::sim::Entity world_parent_entity;

    // This is the offset from the pose of World entity corresponding to
    // 'world_parent_entity' to the pose of the direct parent of this World
    // entity.
    intrinsic::Pose3d world_parent_t_direct_parent;

    // The frame ID of the world entity (e.g. "object_name/link_name").
    std::string frame_id;

    // The frame ID of the direct parent entity, or "root" if attached
    // directly to root.
    std::string direct_parent_frame_id;
  };

  struct StreamData {
    std::string simulator_world_id;
    std::string pose_update_pubsub_topic;
    Publisher publisher;
    bool pose_cache_invalidated = true;
  };

  WorldSyncSystem(
      std::string_view expected_simulator_world_id,
      std::unique_ptr<WorldSyncStub> world_sync_stub,
      std::shared_ptr<ObjectWorldServiceStub> object_world_service_stub,
      absl::Duration refresh_time, PubSub* pubsub,
      std::string_view simulator_name);

  absl::Status RebuildPoseCache(const gz::sim::EntityComponentManager& ecm);
  std::string GetScopedName(gz::sim::Entity entity,
                            const gz::sim::EntityComponentManager& ecm);

  const std::string expected_simulator_world_id_;
  const std::string simulator_name_;

  // Cached entity scoped names to avoid repeated costly lookup in PostUpdate.
  absl::flat_hash_map<gz::sim::Entity, std::string> name_cache_;
  absl::flat_hash_map<gz::sim::Entity, EntityPoseCacheEntry> pose_cache_;

  std::unique_ptr<WorldSyncStub> world_sync_stub_;
  std::shared_ptr<ObjectWorldServiceStub> object_world_service_stub_;

  absl::Duration last_refresh_sim_time_elapsed_ = absl::ZeroDuration();
  absl::Duration time_between_refreshes_;

  PubSub* pubsub_;

  absl::Mutex stream_mutex_;
  absl::StatusOr<StreamData> stream_data_ ABSL_GUARDED_BY(stream_mutex_) =
      absl::UnavailableError("Connecting to simulation service...");
  std::unique_ptr<grpc::ClientContext> stream_context_
      ABSL_GUARDED_BY(stream_mutex_);

  // The stream connection thread is declared last to ensure the grpc context
  // and all accessed data remains valid until the thread is destroyed.
  std::unique_ptr<Thread> stream_connection_thread_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_WORLD_SYNC_SYSTEM_H_
