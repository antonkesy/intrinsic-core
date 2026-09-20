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

#ifndef INTRINSIC_UTIL_CYCLIC_RUNNER_H_
#define INTRINSIC_UTIL_CYCLIC_RUNNER_H_

#include <functional>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/util/time/clock_steady.h"
#include "intrinsic/util/time/elapsed_timer.h"

namespace intrinsic {

// Runs a task in a cyclic manner on the current thread of execution.
class CyclicRunner {
 public:
  CyclicRunner();

  // Not copyable or movable
  CyclicRunner(const CyclicRunner&) = delete;
  CyclicRunner& operator=(const CyclicRunner&) = delete;

  // Starts running the `cyclic_task` every `period` duration, and blocks
  // until the CyclicRunner is Quit(). If Quit() is called before Start(), this
  // will return immediately, without running `cyclic_task`. If `cyclic_task`
  // overruns the specified `period` the next `cyclic_task` will execute as soon
  // as possible. If `period` < absl::Duration(), a zero length duration,
  // returns an error immediately without running `cyclic_task`. If already
  // Start()ed returns an error.
  //
  // Start should only be called on the thread of execution where the
  // `cyclic_task` will run.
  absl::Status Start(const std::function<void()>& cyclic_task,
                     absl::Duration period);

  // Stops running the `cyclic_task` at the start of the next cycle. In ideal
  // circumstances, this may take up to `period` time to occur. If `cyclic_task`
  // never completes, this may fail to quit. This method is thread-safe.
  //
  // Note: It may be helpful to bind a callback to Quit prior to Start()ing the
  // loop.
  void Quit();

 private:
  void SetQuit() ABSL_LOCKS_EXCLUDED(mutex_);

  void RunCyclicTask(const std::function<void()>& cyclic_task,
                     absl::Duration period) ABSL_LOCKS_EXCLUDED(mutex_);

  // Blocks until `wait_time` has occurred.
  void Wait(ElapsedTimer* lap_timer) ABSL_LOCKS_EXCLUDED(mutex_);

  ClockSteady clock_;

  // Indicates that Start() has been called. Start() should only be called once.
  bool started_;

  // The period at which the `cyclic_task` is run.
  absl::Duration period_;

  // Used to wait until a timeout, similarly to std::condition_variable.
  absl::Mutex mutex_;

  // Set to true when ready to stop running `cyclic_task` and return from
  // Start().
  bool quit_ ABSL_GUARDED_BY(mutex_);

  // Set to true if `quit_` becomes true while while waiting.
  bool quit_now_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_UTIL_CYCLIC_RUNNER_H_
