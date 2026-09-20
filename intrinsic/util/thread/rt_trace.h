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

#ifndef INTRINSIC_UTIL_THREAD_RT_TRACE_H_
#define INTRINSIC_UTIL_THREAD_RT_TRACE_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/release/source_location.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/util/thread/rt_trace_internal.h"

// This file defines realtime compatible helper macros to annotate code for
// tracing.
//
// A trace is a sequence of events that can be used to analyze the program flow
// and timing once execution has finished.
//
// Tracing should already be ready to use within the ICON stack.
// Pass the flag `enable_tracing` to the ICON binary to enable it.
// The flag `start_tracing` needs to be used until TODO(b/216627485) is resolved
// and tracing can be started and paused using the ICON client API.
//
// Basic usage:
//
// 1. Enable tracing
// 2. Initialize the thread local tracers before entering realtime operation.
// 3. Start tracing
// 4. Optional: Pause tracing once you captured all interesting events.
//    Optional: Loop back to 3.
// 5. Stop the process to ensure all traces are written into the trace
//    directory defined by the flag `trace_dir`.
//
// The trace files consist of a list of per thread 'traceEvents'.
// intrinsic/util/thread/combine_traces.sh combines them into a
// single file that's compatible with the Chrome Trace Format
// (go/trace-event-format). Analyze the resulting file using
// https://ui.perfetto.dev/.
//
// The design is explained in go/intrinsic-icon-tracing-design.
namespace intrinsic::tracing {

// The maximum string size of the name of a trace.
// Useful for: FixedStrCat<tracing::kMaxNameLength>("abc", "def")
constexpr size_t kMaxNameLength = TraceEntry::kMaxNameLength;

// A ScopedTrace is automatically written to the tracer when destructed.
// A ScopedTrace that was started while tracing was active is still written when
// tracing is paused. When in doubt, use the macro defined below.
class ScopedTrace {
 public:
  // Creates a ScopedTrace. Stores the creation timestamp and
  // logs the trace on destruction.
  // An oversized `name` is silently shortened to fit.
  // Nothing is done if tracer == nullptr, or tracing is not active.
  explicit ScopedTrace(
      absl::string_view name,
      TracerInterface* tracer = Tracer::GetThreadLocalTracerOrNullptr(),
      intrinsic::SourceLocation location = intrinsic::SourceLocation::current())
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Creates a ScopedTrace. Stores the creation timestamp and
  // logs the trace on destruction.
  // `args` must have alpha-numeric types. Concatenates them to a FixedString
  // and assigns the concatenation to the `name_` field. Truncates an oversized
  // resulting string silently to fit into `name_`. The `name_` field is used to
  // identify the trace in perfetto.
  //
  // Does nothing if tracer == nullptr, or tracing is not active. This means
  // that passing in multiple arguments does not introduce significant overhead
  // if tracing is not active. If tracing is active, all `args` are
  // copied/formatted.
  template <typename... AV>
  explicit ScopedTrace(TracerInterface* tracer,
                       intrinsic::SourceLocation location,
                       const AV&... args) INTRINSIC_CHECK_REALTIME_SAFE {
    // Do nothing if tracer == nullptr, or tracing is not active.
    if (!tracer || !tracer->IsTracingActive()) {
      return;
    }
    tracer_ = tracer;
    name_ = icon::FixedStrCat<TraceEntry::kMaxNameLength>(args...);
    start_nsec_ = tracer->GetTimeInNsec();
    location_ = std::move(location);
  }

  ~ScopedTrace() INTRINSIC_CHECK_REALTIME_SAFE;

 private:
  TracerInterface* tracer_ = nullptr;
  intrinsic::SourceLocation location_;
  uint64_t start_nsec_;
  // The name of the event. An oversized string is silently shortened to fit.
  icon::FixedString<TraceEntry::kMaxNameLength> name_;
};

// Logs a `name`ed Event without duration.
//
// An oversized `name` is silently shortened to fit.
// When in doubt, use the macro defined below.
//
// Nothing is done if tracer == nullptr, or tracing is not active.
void LogInstantEvent(
    absl::string_view name,
    TracerInterface* tracer = Tracer::GetThreadLocalTracerOrNullptr(),
    intrinsic::SourceLocation location = intrinsic::SourceLocation::current())
    INTRINSIC_CHECK_REALTIME_SAFE;

// Logs the name and value of `var` without duration.
// The parameter `name` identifies the counter, and `var_name` is the name of
// the variable.
//
// May silently truncate `name` and/or `var_name`.
//
// When in doubt, use the macro defined below.
//
// Nothing is done if tracer == nullptr, or tracing is not active.
template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
void LogCounterEvent(
    absl::string_view name, absl::string_view var_name, T var,
    TracerInterface* tracer = Tracer::GetThreadLocalTracerOrNullptr(),
    intrinsic::SourceLocation location = intrinsic::SourceLocation::current())
    INTRINSIC_CHECK_REALTIME_SAFE {
  if (!tracer || !tracer->IsTracingActive()) {
    return;
  }
  tracer->AddTrace(
      {.type = TraceEntry::EntryType::kCounterEvent,
       .start_nsec = tracer->GetTimeInNsec(),
       .name = name,
       .args_key = var_name,
       .args_value = icon::FixedStrCat<TraceEntry::kMaxArgsLength>(var),
       .location = std::move(location)});
}

// Helper to create and log a ScopedTrace. Useful to track events and their
// duration.
//
// A scoped trace logs
// * `name`
// * location
// * timestamp of creation and destruction
// May silently truncate `name`.
// You can only use this macro once in any given scope.
// Nothing is done if tracing is not enabled, initialized and active.
#define INTRINSIC_TRACE_SCOPED(name) \
  intrinsic::tracing::ScopedTrace kScopedTrace(name)

// Helper to create and log a ScopedTrace with multiple alpha numeric arguments.
// Processed the arguments only if tracing is active. Useful to track events
// and their duration.
//
// A scoped trace logs
// * all var args must be alpha-numeric types and are concatenated as a string
// * location
// * timestamp of creation and destruction
// May silently truncate the concatenation of the varargs.
// You can only use this macro once in any given scope.
// Nothing is done if tracing is not enabled, initialized and active.
#define INTRINSIC_TRACE_VA_SCOPED(...)                             \
  intrinsic::tracing::ScopedTrace kScopedTrace(                    \
      intrinsic::tracing::Tracer::GetThreadLocalTracerOrNullptr(), \
      intrinsic::SourceLocation::current(), __VA_ARGS__)

// Helper to create and log an InstantEvent. Useful to track events without a
// duration.
//
// Logs
// * `name`
// * timestamp
// * location
// of the event.
// May silently truncate `name`.
//
// Nothing is done if tracing is not enabled, initialized and active.
#define INTRINSIC_TRACE_INSTANT(name) intrinsic::tracing::LogInstantEvent(name)

// Helper to create and log a CounterEvent. Useful to track the numeric value of
// a variable over time.
//
// Logs
// * name of `var`
// * value of `var`
// * timestamp
// * location of the event.
//
// Perfetto groups CounterEvents based on thread, location and variable
// name. May silently truncate `name`.
//
// Nothing is done if tracing is not enabled, initialized and active.
#define INTRINSIC_TRACE_COUNTER(var) \
  intrinsic::tracing::LogCounterEvent(#var, #var, var)

// Helper to create and log a CounterEvent with a custom name. Useful to track
// the numeric value of a variable over time.
//
// Logs
// * `name` of the counter
// * name of `var`
// * value of `var`
// * timestamp
// * location of the event.
//
// Perfetto groups CounterEvents based on the name, thread, location and
// variable name. May silently truncate `name`.
//
// Nothing is done if tracing is not enabled, initialized and active.
#define INTRINSIC_TRACE_NAMED_COUNTER(name, var) \
  intrinsic::tracing::LogCounterEvent(name, #var, var)

}  // namespace intrinsic::tracing

#endif  // INTRINSIC_UTIL_THREAD_RT_TRACE_H_
