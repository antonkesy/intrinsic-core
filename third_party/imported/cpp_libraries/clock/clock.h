//
// Copyright 2019 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#ifndef CLOCK_CLOCK_H__
#define CLOCK_CLOCK_H__

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"

namespace util {

// An abstract interface representing a Clock, which is an object that can
// tell you the current time.
//
// This interface allows decoupling code that uses time from the code that
// creates a point in time.  You can use this to your advantage by injecting
// Clocks into interfaces rather than having implementations call absl::Now()
// directly.
//
// The Clock::RealClock() function returns a pointer (that you do not own)
// to the global realtime clock.
//
// Example:
//
//   bool IsWeekend(Clock* clock) {
//     absl::Time now = clock->TimeNow();
//     // ... code to check if 'now' is a weekend.
//   }
//
//   // Production code.
//   IsWeekend(Clock::RealClock());
//
//   // Test code:
//   MyTestClock test_clock(SATURDAY);
//   IsWeekend(&test_clock);
//
class Clock {
 public:
  // Returns a pointer to the global realtime clock.  The caller does not
  // own the returned pointer and should not delete it.  The returned clock
  // is thread-safe.
  static Clock* RealClock();

  virtual ~Clock() {}

  // Returns the current time.
  virtual absl::Time TimeNow() = 0;

  // See absl::Mutex::AwaitWithDeadline(). That function only works with real
  // clocks; this wrapper supports the same functionality for both real clocks
  // and simulated clocks.
  // Returns true if the condition became true, returns false if the deadline
  // was hit.
  virtual bool AwaitWithDeadline(absl::Mutex* mu,
                                 const absl::Condition& condition,
                                 absl::Time deadline) = 0;
};

// A simulated clock is a concrete Clock implementation that does not "tick"
// on its own.  Time is advanced by explicit calls to the AdvanceTime() or
// SetTime() functions.
//
// Example:
//   SimulatedClock sim_clock;
//   absl::Time now = sim_clock.TimeNow();
//   // now == absl::UnixEpoch()
//
//   now = sim_clock.TimeNow();
//   // now == absl::UnixEpoch() (still)
//
//   sim_clock.AdvanceTime(absl::Seconds(3));
//   now = sim_clock.TimeNow();
//   // now == absl::UnixEpoch() + absl::Seconds(3)
//
// This code is thread-safe.
class SimulatedClock : public Clock {
 public:
  explicit SimulatedClock(absl::Time t);
  SimulatedClock() : SimulatedClock(absl::UnixEpoch()) {}

  // Returns the simulated time.
  absl::Time TimeNow() override;

  // WARNING: not implemented yet, will fail with a fatal error.
  bool AwaitWithDeadline(absl::Mutex* mu, const absl::Condition& condition,
                         absl::Time deadline) override;

  // Sets the simulated time to the argument.
  void SetTime(absl::Time t);

  // Advances the simulated time by the specified duration.
  void AdvanceTime(absl::Duration d);

  // WARNING: not implemented yet, will fail with a fatal error.
  //
  // Waits until the given number of additional Sleep(), SleepUntil(), or
  // AwaitWithDeadline() calls, over the number previously waited for, have been
  // initiated. This should be used in conjunction with AdvanceTime() to block
  // until all worker threads complete their jobs and fall asleep once again.
  // It is erroneous to await fewer calls than can possibly be initiated before
  // the next AdvanceTime().
  void WaitUntilThreadsAsleep(int num_calls);

 private:
  template <class T>
  void UpdateTime(const T& now_updater) ABSL_LOCKS_EXCLUDED(lock_);

  absl::Mutex lock_;
  absl::Time now_ ABSL_GUARDED_BY(lock_);
};

}  // namespace util

#endif  // CLOCK_CLOCK_H__
