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

#include "intrinsic/util/cyclic_runner.h"

#include <functional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/util/time/clock_steady.h"
#include "intrinsic/util/time/elapsed_timer.h"

namespace intrinsic {

CyclicRunner::CyclicRunner()
    : started_(false), quit_(false), quit_now_(false) {}

absl::Status CyclicRunner::Start(const std::function<void()>& cyclic_task,
                                 absl::Duration period) {
  if (started_) {
    return absl::FailedPreconditionError(
        "Start() has already been called on this CyclicRunner");
  }
  started_ = true;
  constexpr absl::Duration kZeroDuration = absl::Duration();
  if (period <= kZeroDuration) {
    return absl::InvalidArgumentError(
        absl::StrCat("`period` must be greater than a zero-length duration."
                     "`period` == ",
                     absl::ToInt64Nanoseconds(period), "nanoseconds"));
  }
  RunCyclicTask(cyclic_task, period);
  return absl::OkStatus();
}

void CyclicRunner::Quit() { SetQuit(); }

void CyclicRunner::SetQuit() {
  absl::MutexLock lock(mutex_);
  quit_ = true;
}

void CyclicRunner::RunCyclicTask(const std::function<void()>& cyclic_task,
                                 absl::Duration period) {
  // Quit immediately if `quit_` is set beforing starting the cyclic behavior.
  {
    absl::MutexLock lock(mutex_);
    if (quit_) {
      return;
    }
  }

  period_ = period;

  for (;;) {
    ElapsedTimer timer(&clock_);
    // run immediately on the first iteration
    cyclic_task();
    // Wait to run `cyclic_task` again.
    Wait(&timer);
    if (quit_now_) {
      return;
    }
  }
}

void CyclicRunner::Wait(ElapsedTimer* lap_timer) {
  for (;;) {
    // TODO(intrinsic-icon): This doesn't use the monotonic steady clock, it
    // instead uses absl's time. Consider replacing with a condition that waits
    // on a monotonic clock.
    {
      absl::MutexLock lock(mutex_);
      // re-calculate deadline in case the system clock adjusted backwards, we'd
      // really like to use the monotonic clock here, since we want to wait for
      // periodic elapsed times with a consistent period, even for relatively
      // short durations.
      mutex_.AwaitWithDeadline(absl::Condition(&quit_),
                               period_ - lap_timer->Elapsed() + absl::Now());
      // If Quit() was invoked while waiting return immediately and set
      // `quit_now_`. This copy is done to avoid needing to lock again in
      // RunCyclicTask() to check `quit_` again.
      if (quit_) {
        quit_now_ = true;
        return;
      }
    }
    // Check for spurious wakeups. Have we now actually waited for `period_`
    // duration, or did this wake-up early?
    if (period_ <= lap_timer->Elapsed()) {
      return;
    }
  }
}

}  // namespace intrinsic
