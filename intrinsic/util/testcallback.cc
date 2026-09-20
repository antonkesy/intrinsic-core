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

#include "intrinsic/util/testcallback.h"

#include <atomic>
#include <string>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/base/const_init.h"
#include "absl/base/no_destructor.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/functional/any_invocable.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::testing {

ABSL_CONST_INIT std::atomic<bool> enable_test_callbacks{false};
ABSL_CONST_INIT absl::Mutex test_callback_map_mutex(absl::kConstInit);

// go/totw/110#the-fix-safe-initialization-no-destruction
absl::flat_hash_map<std::string, absl::AnyInvocable<void()>>&
GetTestCallbackMap() ABSL_EXCLUSIVE_LOCKS_REQUIRED(test_callback_map_mutex) {
  static absl::NoDestructor<
      absl::flat_hash_map<std::string, absl::AnyInvocable<void()>>>
      test_callback_map;
  return *test_callback_map;
}

void EnableCallbacksTestOnly() {
  enable_test_callbacks.store(true, std::memory_order_relaxed);
}

void NotifyTestCallbackIfNeeded(absl::string_view label) {
  if (!enable_test_callbacks) return;
  {
    absl::MutexLock lock(test_callback_map_mutex);
    auto it = GetTestCallbackMap().find(label);
    if (it == GetTestCallbackMap().end()) return;
    it->second();
  }
}

absl::Status RegisterCallbackTestOnly(absl::string_view label,
                                      absl::AnyInvocable<void()> callback) {
  absl::MutexLock lock(test_callback_map_mutex);
  if (GetTestCallbackMap().contains(label)) {
    return absl::AlreadyExistsError(
        absl::StrCat("Callback '", label, "' already registered"));
  }
  GetTestCallbackMap()[label] = std::move(callback);
  return absl::OkStatus();
}

absl::Status UnregisterCallbackTestOnly(absl::string_view label) {
  absl::MutexLock lock(test_callback_map_mutex);
  if (!GetTestCallbackMap().contains(label)) {
    return absl::NotFoundError("Callback not registered");
  }
  GetTestCallbackMap().erase(label);
  return absl::OkStatus();
}

CallbackCleanup::~CallbackCleanup() {
  if (label_.has_value()) {
    (void)UnregisterCallbackTestOnly(*label_);
  }
}

absl::StatusOr<CallbackCleanup> RegisterScopedCallbackTestOnly(
    absl::string_view label, absl::AnyInvocable<void()> callback) {
  INTR_RETURN_IF_ERROR(RegisterCallbackTestOnly(label, std::move(callback)));
  return CallbackCleanup(label);
}

}  // namespace intrinsic::testing
