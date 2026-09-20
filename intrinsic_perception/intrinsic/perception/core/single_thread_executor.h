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

#ifndef INTRINSIC_PERCEPTION_CORE_SINGLE_THREAD_EXECUTOR_H_
#define INTRINSIC_PERCEPTION_CORE_SINGLE_THREAD_EXECUTOR_H_

#include "absl/functional/any_invocable.h"
#include "absl/status/status.h"
#include "intrinsic/util/thread/concurrent_queue.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic::perception {

// Executes tasks asynchronously and sequentially on a single dedicated
// background thread.
class SingleThreadExecutor {
 public:
  SingleThreadExecutor();
  virtual ~SingleThreadExecutor();

  // Waits until all outstanding scheduled tasks have been executed.
  virtual absl::Status WaitForOutstandingTasks() const;

  // Schedules a task to be executed on the background worker thread.
  virtual absl::Status Schedule(absl::AnyInvocable<void() &&> task) const;

 private:
  mutable ConcurrentQueue<absl::AnyInvocable<void() &&>> queue_{/*max_size=*/0};
  Thread worker_thread_;
};

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CORE_SINGLE_THREAD_EXECUTOR_H_
