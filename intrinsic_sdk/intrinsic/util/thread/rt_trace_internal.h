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

#ifndef INTRINSIC_UTIL_THREAD_RT_TRACE_INTERNAL_H_
#define INTRINSIC_UTIL_THREAD_RT_TRACE_INTERNAL_H_

#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/flags/declare.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/release/source_location.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/platform/common/buffers/realtime_write_queue.h"
#include "intrinsic/util/thread/thread.h"
#include "riegeli/base/object.h"
#include "riegeli/bytes/fd_writer.h"

// Declare the flag so that it can be set in tests.
// This is necessary as `GetTracePathBase` is static. See
// go/flags#declaring_flags
//
// Define the top level directory ICON traces should be stored in. Traces are
// stored in a subdirectory of this folder. It is created if it doesn't exist.
ABSL_DECLARE_FLAG(std::string, trace_dir);

// This file contains the internal API of the realtime tracing implementation.
//
// Generally Traces should be generated following the documentation in
// google3/intrinsic/utils/rt_trace.h and a normal user should not need
// to interact with the low-level API in this file.
//
// The following needs to be done to generate a trace:
//
// 1. Enable tracing for this process:
//    Tracer::EnableTracing();
// 2. Initialize a thread local tracer before entering realtime operation:
//    INTR_RETURN_IF_ERROR(Tracer::InitializeThreadLocalTracer("thread_name"))
// 3. Start tracing for all threads in the process:
//    Tracer::StartTracing();
// 4. Optional: Pause tracing once you captured all interesting events:
//    Tracer::PauseTracing();
// 5. Stop the process to ensure all traces are written into the trace
//    directory defined by the flag `trace_dir`.
//
// Creates one output file for every traced thread.
//
// The files consist of a list of 'traceEvents'
// Use the helper (google3/intrinsic/utils/combine_traces.sh) to make
// them compatible with the Chrome Trace Format (go/trace-event-format).
// It combines the traces of all threads and prepends:
// {
//   "displayTimeUnit": "ns",
//   "traceEvents": [
//
// as well as appends:
//    ]
// }
namespace intrinsic::tracing {

// Common realtime compatible representation of a single trace event.
struct TraceEntry {
  // Maximum length of the 'name' string.
  static constexpr size_t kMaxNameLength = 64;
  // Maximum length of the 'args' string.
  static constexpr size_t kMaxArgsLength = 32;
  // All implemented event types.
  enum class EntryType {
    kNone,
    kCompleteEvent,
    kInstantEvent,
    kCounterEvent,
    kMetadata,
  };

  EntryType type = EntryType::kNone;
  // Timestamp of the start of the event in nanoseconds.
  uint64_t start_nsec = 0;
  // Timestamp of the end of the event in nanoseconds.
  uint64_t end_nsec = 0;
  // The name of the event. An oversized string is silently shortened to fit.
  icon::FixedString<kMaxNameLength> name;
  // Additional arguments of the event. An oversized string is silently
  // shortened to fit. The parameters `args_key` and `args_value` are used to
  // store a single {key, value} pair that is then exported to the `args` JSON
  // field.
  icon::FixedString<kMaxArgsLength> args_key;
  icon::FixedString<kMaxArgsLength> args_value;

  // The source location of the event.
  intrinsic::SourceLocation location;

  // Pid and tid identify the thread the event was generated in.
  // By default (if the value is '0') they are set by the tracer.
  pid_t pid = 0;
  pid_t tid = 0;
};

// Interface for the event tracer.
// Added to enable testing.
class TracerInterface {
 public:
  // Returns the current time in nanoseconds.
  virtual uint64_t GetTimeInNsec() INTRINSIC_CHECK_REALTIME_SAFE = 0;

  // Add a TraceEntry to the event log.
  virtual void AddTrace(TraceEntry entry) INTRINSIC_CHECK_REALTIME_SAFE = 0;

  // Returns true if tracing is active, otherwise false.
  virtual bool IsTracingActive() INTRINSIC_CHECK_REALTIME_SAFE = 0;

  virtual ~TracerInterface() = default;
};

// Reads trace entries from a queue, encodes them and writes them into the file
// defined using `InitializeAndRun`.
class TraceItemCollector {
 public:
  explicit TraceItemCollector(
      RealtimeWriteQueue<TraceEntry>::NonRtReader& reader)
      : reader_(reader) {}

  // Initializes the TraceItemCollector by
  // * creating the trace output file `filename`
  // * spawning a non-realtime thread that consumes the trace
  //   entries passed into the reader.
  absl::Status InitializeAndRun(absl::string_view filename);

  // Calls thread.join() on the collector thread. Returns when the
  // `to_collector_queue_` is closed by the writer.
  // no-op if no collector was started.
  void WaitForShutdown();

 private:
  RealtimeWriteQueue<TraceEntry>::NonRtReader& reader_;
  // Non realtime thread that doesn't initialize a thread local tracer.
  Thread collector_thread_;
  riegeli::FdWriter<> writer_{riegeli::kClosed};
};

// Implements a realtime compatible event tracer.
//
// There is exactly one Tracer for every thread. This tracer starts a
// non-realtime collector thread. The collector thread accumulates the realtime
// TraceEntries, converts them to JSON and writes them into a file.
class Tracer : public TracerInterface {
 public:
  // Signals the shutdown to the collector and waits until data is written.
  ~Tracer() override;

  // Enables tracing for all threads of this process. Must be called before
  // calling `InitializeThreadLocalTracer` in the threads.
  static void EnableTracing() INTRINSIC_CHECK_REALTIME_SAFE;

  // Changes the state of the internal `tracing_enabled` variable.
  // Disables tracing for all newly started threads - can have unintended
  // consequences if a tracing thread is already running.
  static void DisableTracingTestonly() INTRINSIC_CHECK_REALTIME_SAFE;

  // Initializes a thread local tracer (and collector) instance but only if
  // EnableTracing() was called before.
  static absl::Status InitializeThreadLocalTracer(absl::string_view thread_name)
      INTRINSIC_NON_REALTIME_ONLY;

  // Returns the thread local tracer instance or a nullptr.
  // WARNING: Creates the thread local tracer on the first call if tracing is
  // enabled and `InitializeThreadLocalTracer` was not called before.
  // Suppress real-time analysis because only first call does a syscall.
  static Tracer* GetThreadLocalTracerOrNullptr()
      INTRINSIC_SUPPRESS_REALTIME_CHECK;

  // Returns the current time in nanoseconds.
  uint64_t GetTimeInNsec() override INTRINSIC_CHECK_REALTIME_SAFE;

  // Sends a TraceEntry to the collector by adding it into the queue.
  // An error is logged, if the TraceEntry couldn't be inserted into the queue.
  // pid, tid is "injected" by the tracer
  void AddTrace(TraceEntry entry) override INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns true if tracing is active. Returns false when tracing is paused.
  // Tracing can be started/paused using `StartTracing` and `PauseTracing`.
  bool IsTracingActive() override INTRINSIC_CHECK_REALTIME_SAFE;

  // Starts tracing for all tracers.
  static void StartTracing() INTRINSIC_CHECK_REALTIME_SAFE;

  // Pauses tracing for all tracers.
  static void PauseTracing() INTRINSIC_CHECK_REALTIME_SAFE;

  // Path to the base folder where all traces for this session are collected.
  static const std::string& GetTracePathBase();

  // Provides thread safe access to the global tracing state.
  static bool IsTracingEnabled();

 private:
  pid_t pid_;
  pid_t tid_;

  bool is_initialized_ = false;

  // Closing the queue signals the collector thread to finalize the exported
  // data.
  RealtimeWriteQueue<TraceEntry> to_collector_queue_;
  TraceItemCollector collector_;

  // Should not be called directly, use `GetThreadLocalTracerOrNullptr` instead.
  Tracer();

  absl::Status InternalInit();
};

}  // namespace intrinsic::tracing

#endif  // INTRINSIC_UTIL_THREAD_RT_TRACE_INTERNAL_H_
