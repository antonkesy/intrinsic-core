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

#ifndef INTRINSIC_ICON_CONTROL_INITIALIZED_ASYNC_BUFFER_H_
#define INTRINSIC_ICON_CONTROL_INITIALIZED_ASYNC_BUFFER_H_

#include <errno.h>
#include <string.h>

#include <ctime>
#include <optional>

#include "absl/log/log.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/futex.h"

namespace intrinsic::icon {

// InitializedAsyncBuffer implements a single-producer/multi-consumer buffer
// container that is realtime safe on the producer side, and thread safe on the
// consumer side.
// The producer must be single-threaded!
//
// It wraps intrinsic::AsyncBuffer's triple buffered container implementation,
// and extends it by allowing one or more consumers to efficiently wait until an
// initial value is written to the buffer.
//
// Note: Make sure that no consumers are waiting in GetCurrentValue() when the
// class is destroyed. `Shutdown()` can be called to wake up any current
// waiters, but the actual destruction logic must be handled by the caller.
template <typename T>
class InitializedAsyncBuffer {
 public:
  // Forwards `args` to the ctor for the contained type. Useful for
  // InitializedAsyncBuffers that hold types without a default ctor.
  template <typename... InitArgs>
  explicit InitializedAsyncBuffer(const InitArgs&... args);

  // Returns value held in the currently active buffer to the consumer. This may
  // be the same value which was returned to last call to GetActiveBuffer().
  //
  // If no buffer has been committed since construction or the last call to
  // Clear(), blocks until `deadline` or a buffer is committed.
  // Returns the value of the active buffer if one becomes available before
  // `deadline`, nullopt otherwise.
  // Returns nullopt if the buffer has been shutdown.
  std::optional<T> GetCurrentValue(absl::Time deadline)
      INTRINSIC_NON_REALTIME_ONLY;

  // Commits the free buffer by swapping it with the mailbox buffer.
  //
  // Each call to this function must be preceded by a call to GetFreeBuffer()
  // or else it has no effect.
  // Returns true if this call was preceded by a call to GetFreeBuffer(), false
  // otherwise.
  bool CommitFreeBuffer();

  // Returns a free buffer to the producer. This buffer can be modified until
  // the producer calls CommitFreeBuffer().
  T* GetFreeBuffer();

  // Called by the producer to return the InitializedAsyncBuffer to the
  // "uninitialized" state. Thread-safe and realtime safe.
  void Clear();

  // Called by the producer to signal to all consumers that there will not be
  // any new values. Wakes up any current waiters. Thread-safe,
  // realtime safe and can be called repeatedly.
  void Shutdown();

 private:
  // Used to lock the buffer on the consumer side so that concurrent calls to
  // GetCurrentValue() do not invalidate each other's buffer pointers. No thread
  // annotation because the producer side must not block and is expected to be
  // single-threaded.
  absl::Mutex active_buffer_mutex_;
  intrinsic::AsyncBuffer<T> buffer_;
  // Used as a futex – 0 means that nothing has been committed to the
  // AsyncBuffer since the last call to Clear(). 1 means that a value has been
  // committed. -1 means that the AsyncBuffer has been shutdown.
  int initialized_ = 0;
};

template <typename T>
template <typename... InitArgs>
inline InitializedAsyncBuffer<T>::InitializedAsyncBuffer(
    const InitArgs&... args)
    : buffer_(args...) {}

template <typename T>
std::optional<T> InitializedAsyncBuffer<T>::GetCurrentValue(
    absl::Time deadline) {
  while (true) {
    timespec deadline_ts = absl::ToTimespec(deadline);
    int wait_result = FutexWaitDeadline(&initialized_, 0, &deadline_ts);
    if (wait_result == 0) {
      if (initialized_ == 0) {
        // Spurious wakeup, go back to sleep;
        continue;
      }
      // No errors, end the loop.
      break;
    }
    if (errno == EAGAIN) {
      // The futex was changed before we started to wait, end the loop.
      break;
    }
    if (errno == ETIMEDOUT) {
      // We have a timeout. Return nullopt without a log message.
      return std::nullopt;
    }
    // Other error, log and return.
    LOG(ERROR) << "Got error '" << strerror(errno)
               << "' waiting for 'initialized' futex. (Futex value is, "
               << initialized_ << ", wait_result=" << wait_result << ")";
    return std::nullopt;
  }
  if (initialized_ == -1) {
    // We need to return here in case a value never was never set before the
    // shutdown.
    return std::nullopt;
  }
  absl::MutexLock lock(active_buffer_mutex_);
  T* buffer_ptr;
  (void)buffer_.GetActiveBuffer(&buffer_ptr);
  return *buffer_ptr;
}

template <typename T>
bool InitializedAsyncBuffer<T>::CommitFreeBuffer() {
  bool committed = buffer_.CommitFreeBuffer();
  if (committed) {
    // Only wake consumers if `initialized_` is modified.
    if (__sync_bool_compare_and_swap(&initialized_, /*expected=*/0,
                                     /*desired=*/1)) {
      FutexWakeAll(&initialized_);
    }
  }
  return committed;
}

template <typename T>
T* InitializedAsyncBuffer<T>::GetFreeBuffer() {
  return buffer_.GetFreeBuffer();
}

template <typename T>
void InitializedAsyncBuffer<T>::Clear() {
  FutexReset(&initialized_);
}

template <typename T>
void InitializedAsyncBuffer<T>::Shutdown() {
  // Set the value to -1, which is a special value that GetCurrentValue()
  // recognizes as "shutdown", regardless of the current value. We do not care
  // about the return value, which is the current value of `initialized_`,
  // because we want to shutdown in any case and the current state is irrelevant
  // for that.
  __sync_lock_test_and_set(&initialized_, -1);
  FutexWakeAll(&initialized_);
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_INITIALIZED_ASYNC_BUFFER_H_
