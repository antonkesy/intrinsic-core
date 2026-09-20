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

#ifndef INTRINSIC_SIMULATION_WORLD_WORLD_SERVICE_UPDATER_H_
#define INTRINSIC_SIMULATION_WORLD_WORLD_SERVICE_UPDATER_H_

#include <memory>
#include <string>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/simulation/world/world_updater.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"

namespace intrinsic {
namespace simulation {

// This world updater takes poses and sends them to the world service to update
// the given world ID. Takes ownership of the connection to the world service
// to maintain a separate channel, as this may be a more high-frequency
// operation than we might want in general.
class WorldServiceUpdater : public WorldUpdater {
 public:
  static constexpr absl::Duration kDefaultTimeBetweenUpdates =
      absl::Milliseconds(33);

  using ObjectWorldServiceInterface =
      intrinsic_proto::world::ObjectWorldService::StubInterface;

  // Runs a sanity check that 'target_world_id' exists in 'object_world_service'
  // passed. If this check passes, then the returned pointer should update at
  // most every 'time_between_updates' provided poses from 'generator'.
  static absl::StatusOr<std::unique_ptr<WorldServiceUpdater>> Create(
      absl::string_view target_world_id,
      ObjectWorldServiceInterface* object_world_service,
      UpdateGenerator* generator, absl::Duration time_between_updates);

  static absl::StatusOr<std::unique_ptr<WorldServiceUpdater>> Create(
      absl::string_view target_world_id,
      ObjectWorldServiceInterface* object_world_service,
      UpdateGenerator* generator) {
    return Create(target_world_id, object_world_service, generator,
                  kDefaultTimeBetweenUpdates);
  }

  // Changes the world ID to which we send world updates.
  void SetTargetWorldId(absl::string_view id);

  // Returns the world ID that's being updated by this updater.
  std::string GetTargetWorldId();

 private:
  WorldServiceUpdater(absl::string_view target_world_id,
                      ObjectWorldServiceInterface* object_world_service,
                      UpdateGenerator* generator,
                      absl::Duration time_between_updates);

  void OnWorldUpdates(const ObjectWorldUpdates& updates) final;

  void UpdateObjectWorldService(const ObjectWorldUpdates& updates);

  ObjectWorldServiceInterface* object_world_service_;  // externally owned.

  absl::Mutex target_world_id_mutex_;
  std::string target_world_id_ ABSL_GUARDED_BY(target_world_id_mutex_);
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_WORLD_WORLD_SERVICE_UPDATER_H_
