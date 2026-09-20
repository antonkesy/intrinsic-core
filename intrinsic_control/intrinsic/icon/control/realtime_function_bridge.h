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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_FUNCTION_BRIDGE_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_FUNCTION_BRIDGE_H_

#include <atomic>
#include <cstddef>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "intrinsic/icon/interprocess/binary_futex_condition_variable.h"
#include "intrinsic/icon/interprocess/lockable_binary_futex.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/platform/common/buffers/rt_queue.h"
#include "intrinsic/platform/common/buffers/rt_queue_multi_writer.h"
#include "intrinsic/util/status/status_macros.h"

// IWYU pragma: no_forward_declare absl::FunctionRef

namespace intrinsic::icon {

// Models the current state of a RealtimeFunctionBridge call.
// Calls start out on kNone, and as soon as they are serviced, they can
// transition to kRunning for 0 or more realtime cycles, and then continue to
// either kDone or kError.
enum class RealtimeFunctionState { kNone, kRunning, kDone, kError };

// Dummy declaration to allow function-type specialization below.
template <class T>
class RealtimeFunctionBridge;

// Encapsulates the synchronization and other mechanics required to "call" a
// function in a non-realtime thread and have it execute "piecewise" in a
// realtime thread.
template <class ReturnT, class... ArgTs>
class RealtimeFunctionBridge<ReturnT(ArgTs...)> {
 public:
  // The return type must be default-constructible so that variable to hold the
  // return value can be prepared outside of the realtime thread that generates
  // the actual value.
  static_assert(
      std::is_default_constructible_v<ReturnT>,
      "Return type for RealtimeFunctionBridge must be default-constructible.");

  explicit RealtimeFunctionBridge(
      std::optional<size_t> queue_capacity = std::nullopt);

  // Creates a `FunctionData` object, sends a pointer to that object to the
  // non-realtime thread via `RealtimeQueue`, then blocks until the realtime
  // thread sets its `RealtimeFunctionState` is either kDone or kError.
  template <typename... CallArgs>
  absl::StatusOr<ReturnT> Call(CallArgs&&... args);

  // Closes the bridge. Any further calls to `Call()` will return an
  // `absl::AbortedError`.
  void Close();

  struct FunctionData {
    std::tuple<ArgTs...> args;
    LockableBinaryFutex mutex = LockableBinaryFutex{/*private_futex=*/true};
    BinaryFutexConditionVariable condition_variable =
        BinaryFutexConditionVariable(/*private_futex=*/true);
    ReturnT return_value ABSL_GUARDED_BY(mutex);
    RealtimeFunctionState state ABSL_GUARDED_BY(mutex) =
        RealtimeFunctionState::kNone;
  };

  // Wraps FunctionData so that a realtime thread can execute a call request
  // from non-realtime cyclically.
  // Any state that persists across realtime cycles must live outside the
  // RealtimeFunctionHandle.
  //
  // Note that this class itself is *not* thread safe, but it is thread-safe
  // w.r.t. the Call() method above.
  class RealtimeFunctionHandle {
   public:
    explicit RealtimeFunctionHandle(FunctionData* data) : data_(data) {}

    // Moveable, not copyable (anything else would lead to confusion about the
    // mutex in data_).
    RealtimeFunctionHandle(RealtimeFunctionHandle&& other) noexcept
        : data_(std::exchange(other.data_, nullptr)) {}

    RealtimeFunctionHandle& operator=(RealtimeFunctionHandle&& rhs) noexcept {
      this->data_ = std::exchange(rhs.data_, nullptr);
      return *this;
    }

    RealtimeFunctionHandle(const RealtimeFunctionHandle& other) = delete;

    RealtimeFunctionHandle& operator=(const RealtimeFunctionHandle& rhs) =
        delete;

    // If the RealtimeFunctionHandle goes out of scope before finishing, it
    // makes one attempt to send a RealtimeFunctionState::kError status to the
    // non-realtime context.
    ~RealtimeFunctionHandle();

    // Returns a const pointer to the function arguments for this call, or
    // nullptr if the call has finished (absl::optional would require making a
    // copy to return, and we cannot assume `ArgTs` are copyable in a
    // realtime-safe way).
    const std::tuple<ArgTs...>* Args() const;

    // Returns a pointer to the function arguments for this call, or
    // nullptr if the call has finished (absl::optional would require making a
    // copy to return, and we cannot assume `ArgTs` are copyable in a
    // realtime-safe way).
    std::tuple<ArgTs...>* Args();

    // Attempts to acquire the mutex for this handle without blocking, and if
    // successful, invokes `function_body`.
    // The argument to `function_body` is a reference to the return value of
    // the function call.
    //
    // If `function_body` is invoked, its return value determines the new call
    // state.
    //
    // If, after invoking `function_body`, the call state is kDone or kError,
    // the realtime thread is notified of the result.
    // Subsequent calls to TryLockAndExecute() will not invoke `function_body`
    // or change the call state.
    //
    // Returns true if the lock for the function data is successfully acquired
    // and `function_body` was invoked.
    bool TryLockAndExecute(
        absl::FunctionRef<RealtimeFunctionState(ReturnT&)> function_body);

    // Returns the current state of the function call.
    RealtimeFunctionState State() const;

    // Returns true if the current state is kDone or kError.
    bool Finished() const;

   private:
    FunctionData* data_ = nullptr;
    // Cache the state so we can set it to kDidNotTick if we fail to acquire
    // data_.mutex.
    RealtimeFunctionState state_ = RealtimeFunctionState::kNone;
  };

  // Returns a RealtimeFunctionHandler if there is an outstanding call from the
  // non-realtime thread, absl::nullopt otherwise.
  //
  // Note that any previous handles remain valid, i.e. it's possible to handle
  // concurrent calls - just make sure RealtimeFunctionHandles are finished
  // before they go out of scope. The RealtimeFunctionHandle destructor makes a
  // last-ditch effort to at least finish with an error state, but cannot
  // guarantee that even that succeeds, so letting unfinished
  // RealtimeFunctionHandles go out of scope can cause the corresponding
  // invocation of Call() to hang indefinitely.
  std::optional<RealtimeFunctionHandle> NextRealtimeHandle();

 private:
  RealtimeQueue<FunctionData*> queue_;
  RealtimeQueueMultiWriter<FunctionData*> writer_;
  std::atomic<bool> closed_ = false;
};

// Dummy declaration to allow function-type specialization below.
template <class T>
class SimpleRealtimeFunctionBridge;

// Wraps RealtimeFunctionBridge for cases where the realtime function is
// guaranteed to finish executing in a single cycle. The computation still
// happens on the realtime thread, with full control over *when* in the cycle
// calls are serviced, so there's no danger of race conditions. The advantage of
// SimpleRealtimeFunctionBridge is that there's no need to manage persistent
// execution state like one would for a full RealtimeFunctionBridge.
template <class ReturnT, class... ArgTs>
class SimpleRealtimeFunctionBridge<ReturnT(ArgTs...)> {
 public:
  // The return type must be default-constructible so that variable to hold the
  // return value can be prepared outside of the realtime thread that generates
  // the actual value.
  static_assert(
      std::is_default_constructible_v<ReturnT>,
      "Return type for RealtimeFunctionBridge must be default-constructible.");

  explicit SimpleRealtimeFunctionBridge(
      std::optional<size_t> queue_capacity = std::nullopt)
      : bridge_(queue_capacity) {}

  // Creates a `FunctionData` object, sends a pointer to that object to the
  // non-realtime thread via `RealtimeQueue`, then blocks until the realtime
  // thread calls `ServiceCall()`.
  template <typename... CallArgs>
  absl::StatusOr<ReturnT> Call(CallArgs&&... args) {
    return bridge_.Call(std::forward<CallArgs>(args)...);
  }

  void Close() { bridge_.Close(); }

  // If there is an outstanding call, immediately invokes `handler` with the
  // call's arguments, and forwards its return value.
  //
  // Returns true if `handler` was invoked.
  bool ServiceCall(absl::FunctionRef<ReturnT(ArgTs... args)> handler) {
    if (!handle_.has_value()) {
      handle_ = bridge_.NextRealtimeHandle();
    }
    if (!handle_.has_value()) {
      return false;
    }
    bool ticked = false;
    if (handle_->Args() == nullptr) {
      INTRINSIC_RT_LOG_THROTTLED(ERROR)
          << "No arguments in RealtimeFunctionHandle, returning an error.";
      (void)handle_->TryLockAndExecute(
          [](ReturnT& return_value) { return RealtimeFunctionState::kError; });
    } else {
      // The return value here is equivalent to handle_->Finished() in this
      // case.
      ticked =
          handle_->TryLockAndExecute([this, &handler](ReturnT& return_value) {
            return_value =
                std::apply(std::move(handler), std::move(*handle_->Args()));
            return RealtimeFunctionState::kDone;
          });
    }
    if (handle_->Finished()) {
      handle_ = std::nullopt;
    }
    return ticked;
  }

 private:
  RealtimeFunctionBridge<ReturnT(ArgTs...)> bridge_;
  std::optional<typename RealtimeFunctionBridge<ReturnT(
      ArgTs...)>::RealtimeFunctionHandle>
      handle_;
};

template <class ReturnT, class... ArgTs>
inline RealtimeFunctionBridge<ReturnT(ArgTs...)>::RealtimeFunctionBridge(
    std::optional<size_t> queue_capacity)
    : queue_(queue_capacity), writer_(*queue_.writer()) {}

template <class ReturnT, class... ArgTs>
template <typename... CallArgs>
inline absl::StatusOr<ReturnT> RealtimeFunctionBridge<ReturnT(ArgTs...)>::Call(
    CallArgs&&... args) {
  if (closed_) {
    return absl::AbortedError("RealtimeFunctionBridge is closed.");
  }
  FunctionData function_data{.args = {std::forward<CallArgs>(args)...},
                             .state = RealtimeFunctionState::kNone};

  BinaryFutexLock lock(&function_data.mutex);
  INTR_RETURN_IF_ERROR(writer_.Insert(&function_data));

  // Wait for the realtime thread to signal that it's done. We cannot use
  // absl::Notification here, because its Notify() method uses a MutexLock,
  // which could block the realtime thread.
  INTRINSIC_RT_RETURN_IF_ERROR(function_data.condition_variable.Await(
      &function_data.mutex,
      absl::Condition(
          +[](RealtimeFunctionState* state) {
            return *state == RealtimeFunctionState::kDone ||
                   *state == RealtimeFunctionState::kError;
          },
          &function_data.state)));

  if (function_data.state == RealtimeFunctionState::kError) {
    return absl::UnknownError("Realtime function call failed");
  }
  if (function_data.state != RealtimeFunctionState::kDone) {
    return absl::InternalError(
        "Realtime function call finished with invalid state.");
  }

  return std::move(function_data.return_value);
}

template <class ReturnT, class... ArgTs>
void RealtimeFunctionBridge<ReturnT(ArgTs...)>::Close() {
  closed_ = true;
}

template <class ReturnT, class... ArgTs>
inline std::optional<
    typename RealtimeFunctionBridge<ReturnT(ArgTs...)>::RealtimeFunctionHandle>
RealtimeFunctionBridge<ReturnT(ArgTs...)>::NextRealtimeHandle() {
  auto* maybe_function_data = queue_.reader()->Front();
  if (maybe_function_data == nullptr) {
    // No KeepFront() needed because there's no element to keep.
    return std::nullopt;
  }
  RealtimeFunctionHandle handle(*maybe_function_data);
  queue_.reader()->DropFront();
  return std::make_optional(std::move(handle));
}

template <class ReturnT, class... ArgTs>
inline const std::tuple<ArgTs...>* RealtimeFunctionBridge<
    ReturnT(ArgTs...)>::RealtimeFunctionHandle::Args() const {
  if (data_ == nullptr) {
    return nullptr;
  }
  return &data_->args;
}

template <class ReturnT, class... ArgTs>
inline std::tuple<ArgTs...>*
RealtimeFunctionBridge<ReturnT(ArgTs...)>::RealtimeFunctionHandle::Args() {
  if (data_ == nullptr) {
    return nullptr;
  }
  return &data_->args;
}

template <class ReturnT, class... ArgTs>
inline RealtimeFunctionBridge<
    ReturnT(ArgTs...)>::RealtimeFunctionHandle::~RealtimeFunctionHandle() {
  if (Finished() || data_ == nullptr) {
    return;
  }
  if (!data_->mutex.TryLock()) {
    INTRINSIC_RT_LOG(WARNING)
        << "Cannot send status to non-realtime context, caller will hang!";
    return;
  }

  data_->state = RealtimeFunctionState::kError;
  if (auto status = data_->condition_variable.NotifyOne(); !status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Failed to notify non-realtime thread:" << status.message();
  }
  if (auto status = data_->mutex.Unlock(); !status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Failed to unlock mutex:" << status.message();
  }
}

template <class ReturnT, class... ArgTs>
inline bool RealtimeFunctionBridge<ReturnT(ArgTs...)>::RealtimeFunctionHandle::
    TryLockAndExecute(
        absl::FunctionRef<RealtimeFunctionState(ReturnT&)> function_body) {
  if (data_ == nullptr) {
    // We're either done or in an error state, don't call function_body.
    return false;
  }
  // Make a local copy of the data pointer so that we can drop the pointer and
  // only unlock the mutex afterwards.
  FunctionData* data = data_;
  if (!data->mutex.TryLock()) {
    return false;
  }

  // data.mutex is locked, we can freely access the values in data.
  data->state = function_body(data->return_value);
  state_ = data->state;

  if (Finished()) {
    // Drop the FunctionData pointer – after we release the lock above, the
    // non-realtime thread can immediately delete the object it points to.
    data_ = nullptr;
  }
  // The next two functions should never fail and there is no error return
  // value. So we just print the error message.
  if (auto status = data->condition_variable.NotifyOne(); !status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Failed to notify non-realtime thread:" << status.message();
  }
  if (auto status = data->mutex.Unlock(); !status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Failed to unlock mutex:" << status.message();
  }
  return true;
}

template <class ReturnT, class... ArgTs>
inline RealtimeFunctionState RealtimeFunctionBridge<
    ReturnT(ArgTs...)>::RealtimeFunctionHandle::State() const {
  return state_;
}

template <class ReturnT, class... ArgTs>
inline bool RealtimeFunctionBridge<
    ReturnT(ArgTs...)>::RealtimeFunctionHandle::Finished() const {
  return state_ == RealtimeFunctionState::kDone ||
         state_ == RealtimeFunctionState::kError;
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_FUNCTION_BRIDGE_H_
