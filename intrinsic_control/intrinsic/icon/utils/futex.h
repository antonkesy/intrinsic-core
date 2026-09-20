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

#ifndef INTRINSIC_ICON_UTILS_FUTEX_H_
#define INTRINSIC_ICON_UTILS_FUTEX_H_

#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <climits>
#include <ctime>

#include "absl/base/attributes.h"
#include "intrinsic/icon/testing/realtime_annotations.h"

namespace intrinsic::icon {

/**
 * wait while the value at addr is equal to val
 * @param addr the address
 * @param val the value
 * @param timeout timeout value (defaults to infinte wait)
 * @return 0 on success, -1 on error (see man 2 futex)
 */
inline int FutexWait(int* addr, int val,
                     const struct timespec* timeout = nullptr)
    INTRINSIC_SUPPRESS_REALTIME_CHECK {
  return syscall(SYS_futex, addr, FUTEX_WAIT, val, timeout, nullptr, 0);
}

/**
 * wait while the value at addr is equal to val
 * @param addr the address
 * @param val the value
 * @param deadline deadline value (in CLOCK_REALTIME). Defaults to infinite wait
 *                 if omitted.
 * @return 0 on success, -1 on error (see man 2 futex)
 */
inline int FutexWaitDeadline(int* addr, int val,
                             const struct timespec* deadline = nullptr)
    INTRINSIC_SUPPRESS_REALTIME_CHECK {
  // FUTEX_WAIT_BITSET with FUTEX_BITSET_MATCH_ANY as the final parameter is
  // equivalent to FUTEX_WAIT. The only difference is that it takes an "absolute
  // timeout", or deadline, instead of the "relative timeout" FUTEX_WAIT does.
  //
  // We need to add FUTEX_CLOCK_REALTIME because otherwise the deadline is
  // evaluated in CLOCK_MONOTONIC. CLOCK_REALTIME is more convenient to specify
  // deadlines in.
  return syscall(SYS_futex, addr, FUTEX_WAIT_BITSET | FUTEX_CLOCK_REALTIME, val,
                 deadline, nullptr, FUTEX_BITSET_MATCH_ANY);
}

/**
 * wake up a single waiter
 * @param addr The address of the futex
 * @return 0 on success, -1 on error (see man 2 futex)
 */
inline int ABSL_DEPRECATED(
    "If you don't need FutexWakeAll, use intrinsic::icon::BinaryFutex::Post "
    "from "
    "intrinsic/icon/interprocess/remote_trigger/binary_futex.h instead. "
    "There, `Wait` is optimized and treats errors better.")
    FutexWake(int* addr) INTRINSIC_SUPPRESS_REALTIME_CHECK {
  return syscall(SYS_futex, addr, FUTEX_WAKE, 1, nullptr, nullptr, 0);
}

/**
 * wake up all waiters
 * @param addr The address of the futex
 * @return 0 on success, -1 on error (see man 2 futex)
 */
inline int FutexWakeAll(int* addr) INTRINSIC_SUPPRESS_REALTIME_CHECK {
  return syscall(SYS_futex, addr, FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
}

/**
 * decrement the futex
 * @param addr The address of the futex
 */
inline void FutexDecrement(int* addr) { __sync_fetch_and_add(addr, -1); }

/**
 * Reset the futex to 0
 * @param addr The address of the futex
 */
inline void FutexReset(int* addr) { __sync_fetch_and_and(addr, 0); }

/**
 * increment the futex
 * @param addr The address of the futex
 */
inline void FutexIncrement(int* addr) {
  int val = __sync_fetch_and_add(addr, 1);
  // if the futex is coming from the empty state, wake any waiters
  if (val == 0) FutexWake(addr);
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_UTILS_FUTEX_H_
