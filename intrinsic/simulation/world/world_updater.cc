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

#include "intrinsic/simulation/world/world_updater.h"

#include <functional>
#include <optional>
#include <utility>
#include <variant>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/util/thread/stop_token.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/world/proto/world_updates.pb.h"

namespace intrinsic {
namespace simulation {

namespace {

constexpr int kNumBufferedUpdates = 1024;

}  // namespace

UpdateGenerator::UpdateGenerator() : pending_updates_(kNumBufferedUpdates) {}

bool UpdateGenerator::HasPendingUpdate() { return !pending_updates_.IsEmpty(); }

std::optional<std::variant<WorldUpdate, ObjectWorldUpdate>>
UpdateGenerator::NextWorldUpdate(absl::Time deadline) {
  absl::Duration timeout = deadline - absl::Now();
  if (auto pending_update = pending_updates_.Dequeue(timeout);
      pending_update.ok()) {
    return *pending_update;
  }
  return {};
}

void UpdateGenerator::PublishUpdate(
    std::variant<WorldUpdate, ObjectWorldUpdate> new_update,
    absl::Duration enqueue_timeout) {
  if (auto status =
          pending_updates_.Enqueue(std::move(new_update), enqueue_timeout);
      !status.ok()) {
    LOG_EVERY_N_SEC(WARNING, 3) << status;
  }
}

////////////////////////////////////////////////////////////////////////////////

WorldUpdater::WorldUpdater(UpdateGenerator* generator,
                           absl::Duration time_between_updates)
    : generator_(generator), time_between_updates_(time_between_updates) {}

bool WorldUpdater::Start() {
  if (update_monitor_thread_.joinable()) {
    return false;
  }
  update_monitor_thread_ =
      Thread(std::bind_front(&WorldUpdater::ThreadMain, this));
  return true;
}

bool WorldUpdater::Stop() {
  if (update_monitor_thread_.joinable()) {
    update_monitor_thread_.request_stop();
    update_monitor_thread_.join();
    return true;
  }
  return false;
}

bool WorldUpdater::IsRunning() const {
  return update_monitor_thread_.joinable();
}

void WorldUpdater::ResetUpdateGenerator(UpdateGenerator* new_generator) {
  CHECK_NE(new_generator, nullptr);
  absl::MutexLock reset_generator_lock(reset_generator_mutex_);
  absl::MutexLock generator_lock(generator_mutex_);
  if (generator_ == new_generator) {
    return;
  }
  generator_ = new_generator;
  if (!IsRunning()) {
    return;
  }
  main_thread_generator_updated_.Wait(&generator_mutex_);
}

WorldUpdater::~WorldUpdater() {
  if (IsRunning()) {
    LOG(WARNING) << "Destroying WorldUpdater before Stop() is called! "
                 << "Attempting to stop child thread now.";
    Stop();
  }
}

void WorldUpdater::ThreadMain(StopToken stop_token) {
  ObjectWorldUpdates world_updates;
  absl::Time time_next_update_sent = absl::InfinitePast();
  UpdateGenerator* current_generator = nullptr;
  while (!stop_token.stop_requested()) {
    {
      absl::MutexLock generator_lock(generator_mutex_);
      if (current_generator != generator_) {
        current_generator = generator_;
        main_thread_generator_updated_.SignalAll();
      }
    }
    CHECK_NE(current_generator, nullptr);

    // Collect next pending update
    std::optional<std::variant<WorldUpdate, ObjectWorldUpdate>> new_update;
    new_update = current_generator->NextWorldUpdate(time_next_update_sent);

    const bool got_update = new_update.has_value();
    if (got_update) {
      if (std::holds_alternative<ObjectWorldUpdate>(new_update.value())) {
        *world_updates.add_updates() =
            std::get<ObjectWorldUpdate>(new_update.value());
      } else if (std::holds_alternative<WorldUpdate>(new_update.value())) {
        *world_updates.add_entity_updates() =
            std::get<WorldUpdate>(new_update.value());
      } else {
        LOG(FATAL) << "Got an unknown variant index";
      }
    }

    // If it's too early to send the updates, then spin.
    if (absl::Now() < time_next_update_sent) {
      continue;
    }

    OnWorldUpdates(world_updates);
    world_updates.Clear();
    time_next_update_sent = absl::Now() + time_between_updates_;
  }

  // If we ended early send the last updates.
  if (!world_updates.updates().empty()) {
    absl::SleepFor(time_next_update_sent - absl::Now());
    OnWorldUpdates(world_updates);
  }
}

}  // namespace simulation
}  // namespace intrinsic
