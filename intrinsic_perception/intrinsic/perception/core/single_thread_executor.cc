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

#include "intrinsic/perception/core/single_thread_executor.h"

#include "absl/functional/any_invocable.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/synchronization/notification.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {

SingleThreadExecutor::SingleThreadExecutor()
    : worker_thread_([this] {
        while (true) {
          auto task = queue_.Dequeue();
          if (!task.ok()) {
            // Queue is closed AND completely drained.
            return;
          }
          (*std::move(task))();
        }
      }) {}

SingleThreadExecutor::~SingleThreadExecutor() {
  // Prevent new tasks from being enqueued and wake worker if queue is empty.
  queue_.Close();
  // worker_thread_ joins on destruction, ensuring all remaining tasks in
  // queue_ finish executing before the destructor returns.
}

absl::Status SingleThreadExecutor::WaitForOutstandingTasks() const {
  absl::Notification done;
  INTR_RETURN_IF_ERROR(Schedule([&done] { done.Notify(); }));
  done.WaitForNotification();
  return absl::OkStatus();
}

absl::Status SingleThreadExecutor::Schedule(
    absl::AnyInvocable<void() &&> task) const {
  LOG_EVERY_N_SEC(INFO, 5) << "[SingleThreadExecutor] Queue size: "
                           << queue_.GetSize();
  return queue_.Enqueue(std::move(task));
}

}  // namespace intrinsic::perception
