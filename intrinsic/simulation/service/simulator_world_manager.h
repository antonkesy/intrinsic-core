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

#ifndef INTRINSIC_SIMULATION_SERVICE_SIMULATOR_WORLD_MANAGER_H_
#define INTRINSIC_SIMULATION_SERVICE_SIMULATOR_WORLD_MANAGER_H_

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server_context.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"
#include "intrinsic/world/service/updater/world_updater.grpc.pb.h"

namespace intrinsic {
namespace simulation {

// Manages the simulator's target object world instance.
// It handles initialization by cloning from a starting world, filtering out
// objects marked as sim-disabled, and pausing/resuming sim world state updates
// via the `WorldUpdater` service during sim reset.
class SimulatorWorldManager {
 public:
  struct CreateOptions {
    std::string_view simulator_world_id;
    std::string_view start_world_id;
    bool ignore_disabled_object_deletion_error = false;
  };

  // Clones the starting world into the sim world.
  // If options.start_world_id is empty, checks if options.simulator_world_id
  // exists. If it does not exist, returns an error. If it does exist, no clone
  // is made. All objects marked as "disabled" in sim will be deleted from the
  // cloned sim world. This change is applied atomically so that either all
  // objects are deleted, or the sim world is left unchanged in case of an error
  // while deleting an object.
  static absl::StatusOr<std::unique_ptr<SimulatorWorldManager>> Create(
      std::shared_ptr<intrinsic_proto::world::ObjectWorldService::
                          StubInterface> absl_nonnull object_world_service,
      std::shared_ptr<
          intrinsic_proto::world::WorldUpdater::StubInterface> absl_nullable
      world_updater_service,
      const CreateOptions& options);

  // Checks if the given world exists in the object world service.
  // Returns true if the world exists, false if it does not, or the error status
  // if the check fails.
  static absl::StatusOr<bool> CheckWorldExists(
      intrinsic_proto::world::ObjectWorldService::StubInterface& stub,
      std::string_view world_id);

  ~SimulatorWorldManager();

  std::string_view simulator_world_id() const { return simulator_world_id_; }
  [[nodiscard]] std::string generation_id() const
      ABSL_LOCKS_EXCLUDED(generation_mutex_);

  // Pauses the WorldUpdater, resets the simulator world by cloning it from the
  // starting world, and updates generation ID. Pose updates must be restarted
  // by calling `RestartUpdates()` after the reset process is complete.
  absl::Status StopUpdatesAndResetWorld(
      const ::grpc::ServerContext* absl_nullable context,
      std::string_view start_world_id,
      bool ignore_disabled_object_deletion_error);

  // Resumes the WorldUpdater.
  absl::Status RestartUpdates();

 private:
  SimulatorWorldManager(
      std::shared_ptr<intrinsic_proto::world::ObjectWorldService::StubInterface>
          object_world_service,
      std::shared_ptr<intrinsic_proto::world::WorldUpdater::StubInterface>
          world_updater_service,
      std::string_view simulator_world_id);

  void UpdateGenerationId() ABSL_LOCKS_EXCLUDED(generation_mutex_);

  const std::shared_ptr<
      intrinsic_proto::world::ObjectWorldService::StubInterface>
      object_world_service_;
  const std::shared_ptr<intrinsic_proto::world::WorldUpdater::StubInterface>
      world_updater_service_;
  const std::string simulator_world_id_;

  mutable absl::Mutex generation_mutex_;
  std::string generation_id_ ABSL_GUARDED_BY(generation_mutex_);
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_SERVICE_SIMULATOR_WORLD_MANAGER_H_
