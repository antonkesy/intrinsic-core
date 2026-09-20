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

#ifndef INTRINSIC_UTIL_TESTCALLBACK_H_
#define INTRINSIC_UTIL_TESTCALLBACK_H_

#include <optional>
#include <string>

#include "absl/functional/any_invocable.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

namespace intrinsic::testing {

// Allow a unit test to receive callbacks from production code.
//
// Typical usage example:
//
// In production code:
//   NotifyTestCallbackIfNeeded("intrinsic::icon::session_deleted");
//
// In a unit test:
//   TEST(MyTest, MyTestCase) {
//     EnableCallbacksTestOnly();
//     absl::Notification notification;
//     RegisterCallbackTestOnly("intrinsic::icon::session_deleted",
//         [&notification] { notification.Notify(); });
//     ...
//     EXPECT_TRUE(notification.HasBeenNotified());
//   }

// If any test has registered a callback for the given label, call it.
// For use in production code.
// Real-time safe (and no-op) if EnableCallbacksTestOnly() has not been called.
void NotifyTestCallbackIfNeeded(absl::string_view label);

// Enable testing callbacks for this process.
// Typically, call this in the beginning of a unit test.
// Use in tests only.
void EnableCallbacksTestOnly();

// Register a callback for a label, typically in the beginning of a unit test.
// Captures in `callback` live until the callback is unregistered.
// A test callbacks must be very short, because they are executed in one
// critical section. A test callback must not call other test callbacks to avoid
// deadlocks. Use in tests only.
absl::Status RegisterCallbackTestOnly(absl::string_view label,
                                      absl::AnyInvocable<void()> callback);

// Unregister a callback for a label.
// This is needed if production code could call NotifyTestCallbackIfNeeded()
// after the callback's captures or references would be destroyed.
// Use in tests only.
absl::Status UnregisterCallbackTestOnly(absl::string_view label);

class CallbackCleanup {
 public:
  CallbackCleanup() : label_(std::nullopt) {}
  explicit CallbackCleanup(absl::string_view label)
      : label_(std::string(label)) {}
  CallbackCleanup(const CallbackCleanup& other) = delete;
  CallbackCleanup& operator=(const CallbackCleanup& other) = delete;
  CallbackCleanup(CallbackCleanup&& other) = default;
  CallbackCleanup& operator=(CallbackCleanup&& other) = default;
  ~CallbackCleanup();

 private:
  std::optional<std::string> label_;
};

absl::StatusOr<CallbackCleanup> RegisterScopedCallbackTestOnly(
    absl::string_view label, absl::AnyInvocable<void()> callback);

}  // namespace intrinsic::testing

#endif  // INTRINSIC_UTIL_TESTCALLBACK_H_
