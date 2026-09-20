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

#include "intrinsic/icon/testing/malloc_test.h"

#include <cstdlib>
#include <mutex>  // NOLINT
#include <optional>

#include "absl/base/attributes.h"
#include "absl/log/check.h"
#include "intrinsic/icon/utils/malloc_guard.h"
#include "intrinsic/icon/utils/realtime_stack_trace.h"

namespace intrinsic::icon {

ABSL_CONST_INIT thread_local volatile bool kPrintNextMallocStackTrace = false;
void PrintNextStackTrace() { kPrintNextMallocStackTrace = true; }

namespace {

// once-flag to only call CheckMallocHandlerInstalledOrDie() once
std::once_flag kEnsureMallocHandlerInstalledFlag;

// This keeps track of allocations that we don't want to count. For example, if
// we know a certain operation incurs mallocs *only in tests*, then we can
// ignore those and still use EXPECT_NO_ALLOCATIONS at the end.
ABSL_CONST_INIT thread_local volatile size_t kMallocCountOffset = 0;

// Global variable to make sure that the compiler doesn't
// optimize the code
void* volatile kEnsureHooksRegisteredMallocPtr;
int* volatile kEnsureHooksRegisteredNewPtr;
void* kEnsureHooksRegisteredPosixMemalignPtr;

void CheckMallocHandlerInstalledOrDie() {
  CHECK(InstallMallocGuardHooks()) << "Failed to install malloc guard hooks";
  ResetThreadLocalMallocViolations();
  kMallocCountOffset = 0;

  {
    ScopedThreadLocalReaction scoped_reaction(
        MallocGuardReaction::kStoreViolation);
    MallocGuard test_guard;

    // do a malloc
    kEnsureHooksRegisteredMallocPtr = malloc(32);

    // ensure we saw the malloc
    CHECK_EQ(GetThreadLocalMallocViolations().num_violations, 1)
        << "error: malloc() handler not installed correctly: "
           "call to raw malloc did not increment counter.";
    CHECK_EQ(GetThreadLocalMallocViolations().allocated_bytes, 32)
        << "error: malloc() handler not installed correctly: "
           "call to raw malloc did not count 32 bytes of requested memory.";

    // free the memory
    free(const_cast<void*>(kEnsureHooksRegisteredMallocPtr));
    // do a fake new
    kEnsureHooksRegisteredNewPtr = new int;

    // ensure we saw the new
    CHECK_EQ(GetThreadLocalMallocViolations().num_violations, 2)
        << "error: malloc() handler not installed "
           "correctly: call to new did not increment "
           "counter.";
    CHECK_EQ(GetThreadLocalMallocViolations().allocated_bytes, 32 + sizeof(int))
        << "error: malloc() handler not installed "
           "correctly: call to new did not count "
        << sizeof(int) << "new bytes for an int.";
    // free the memory
    delete kEnsureHooksRegisteredNewPtr;

    // check posix aligned memory allocation
    constexpr size_t kAlignBytes = 16;
    constexpr size_t kSize = 32;
    CHECK_EQ(0, posix_memalign(&kEnsureHooksRegisteredPosixMemalignPtr,
                               kAlignBytes, kSize));

    // ensure we saw the malloc
    CHECK_EQ(GetThreadLocalMallocViolations().num_violations, 3)
        << "error: posix_memalign() handler not installed "
           "correctly: direct call to posix_memalign did not "
           "increment counter.";
    // free the memory
    free(kEnsureHooksRegisteredPosixMemalignPtr);

    // check realloc is caught
    kEnsureHooksRegisteredMallocPtr = malloc(32);
    kEnsureHooksRegisteredMallocPtr =
        realloc(const_cast<void*>(kEnsureHooksRegisteredMallocPtr), 64);
    // ensure we saw the realloc and malloc
    CHECK_EQ(GetThreadLocalMallocViolations().num_violations, 5)
        << "error: realloc() handler not installed correctly: "
           "call to raw realloc did not increment counter.";

    // free the memory
    free(const_cast<void*>(kEnsureHooksRegisteredMallocPtr));
  }
}

thread_local std::optional<ScopedThreadLocalReaction> test_reaction;
thread_local std::optional<MallocGuard> test_guard;

void EnsureThreadGuarded() {
  if (!test_reaction) {
    MallocGuardReaction reaction =
        kPrintNextMallocStackTrace
            ? MallocGuardReaction::kStoreViolationWithTrace
            : MallocGuardReaction::kStoreViolation;
    test_reaction.emplace(reaction);
  }
  if (!test_guard) {
    test_guard.emplace();
  }
}

}  // namespace

void MallocCounterInit() {
  // ensure the malloc handler has been installed
  std::call_once(kEnsureMallocHandlerInstalledFlag,
                 CheckMallocHandlerInstalledOrDie);

  EnsureThreadGuarded();

  ResetThreadLocalMallocViolations();
  kMallocCountOffset = 0;
}

size_t MallocCounterCount() {
  // ensure the malloc handler has been installed
  std::call_once(kEnsureMallocHandlerInstalledFlag,
                 CheckMallocHandlerInstalledOrDie);

  EnsureThreadGuarded();

  if (kPrintNextMallocStackTrace) {
    kPrintNextMallocStackTrace = false;
    LogRtErrorStackTrace();
  }

  size_t total_violations = GetThreadLocalMallocViolations().num_violations;
  CHECK_GE(total_violations, kMallocCountOffset);
  size_t count = total_violations - kMallocCountOffset;
  return count;
}

void MallocCounterSubtract(size_t count) {
  // ensure the malloc handler has been installed
  std::call_once(kEnsureMallocHandlerInstalledFlag,
                 CheckMallocHandlerInstalledOrDie);
  kMallocCountOffset += count;
}

}  // namespace intrinsic::icon
