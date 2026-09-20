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

#include "intrinsic/util/thread/rt_trace_internal.h"

#include <sched.h>
#include <unistd.h>

#include <atomic>
#include <chrono>  // NOLINT
#include <cstdint>
#include <string>

#include "absl/base/attributes.h"
#include "absl/base/no_destructor.h"
#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/icon/release/source_location.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/platform/common/buffers/realtime_write_queue.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/sysinfo.h"
#include "intrinsic/util/thread/thread_options.h"
#include "intrinsic/util/thread/thread_utils.h"
#include "nlohmann/json.hpp"
#include "ortools/base/filesystem.h"
#include "ortools/base/options.h"
#include "ortools/base/path.h"
#include "riegeli/bytes/fd_writer.h"

ABSL_FLAG(std::string, trace_dir, "/tmp/icon_traces",
          "The top level directory ICON traces should be stored in. Is "
          "created if it doesn't exist. Traces are stored in a subdirectory.");

namespace intrinsic::tracing {

namespace details {

// Gets set to `true` for all threads when Tracer::Enable() is called at least
// once.
ABSL_CONST_INIT static std::atomic<bool> tracing_enabled{false};
// Traces are only created and written if tracing is active.
ABSL_CONST_INIT static std::atomic<bool> tracing_active{false};
// Conversion factor from nano to micro.
static constexpr double kNanoToMicro = 1e-3;

}  // namespace details

namespace {
// pid, tid is "injected" by the tracer
TraceEntry CreateThreadNameEntry(absl::string_view thread_name) {
  return {.type = TraceEntry::EntryType::kMetadata,
          .name = absl::string_view("thread_name"),
          .args_key = {"name"},
          .args_value = thread_name};
}
// Convert the data into the Trace Event Format go/trace-event-format.
// We populate the "ts" values of events with nanosecond values rather than
// microseconds.
// We need to add the key-value pair "displayTimeUnit": "ns" to
// the top-level JSON to tell the Perfetto viewer about this.
// As an alternative, we could use floating point milliseconds with the format
// string in https://github.com/namhyung/uftrace/pull/57/files.
std::string ToJSON(const TraceEntry& entry) {
  switch (entry.type) {
    case TraceEntry::EntryType::kMetadata: {
      return absl::StrCat(
          "{",  //
          "\"name\":\"", ::nlohmann::detail::escape(std::string(entry.name)),
          "\", ",                       //
          "\"ph\":\"M\", ",             //
          "\"pid\":", entry.pid, ", ",  //
          "\"tid\":", entry.tid, ", ",  //
          "\"args\":{", "\"",
          ::nlohmann::detail::escape(std::string(entry.args_key)), "\":\"",
          ::nlohmann::detail::escape(std::string(entry.args_value)), "\"",
          "}",  //
          //   No trailing ',' for the final parameter.
          "},");
    }
    case TraceEntry::EntryType::kCompleteEvent: {
      uint64_t duration_nsec = 0;
      // Duration must not be zero or negative.
      // If the duration is zero because the timestamps match,
      // consider switching to an event without duration (e.g kInstantEvent).
      if (entry.start_nsec >= entry.end_nsec) {
        duration_nsec = 1;
        LOG_EVERY_N_SEC(ERROR, 10)
            << "Got CompleteEvent[" << absl::string_view(entry.name) << "@"
            << entry.location.file_name() << ":" << entry.location.line()
            << "] with invalid timestamps! start_nsec[" << entry.start_nsec
            << "], end_nsec[" << entry.end_nsec << "].";
      } else {
        duration_nsec = entry.end_nsec - entry.start_nsec;
      }
      // Use double for duration to keep the nanosecond resolution since
      // perfetto can deal with it and durations are usually not so large that
      // the double-string representation is a problem, but timestamps should be
      // kept as integer to keep precision when converting to string.
      const double duration_usec = details::kNanoToMicro * duration_nsec;
      const uint64_t start_usec = details::kNanoToMicro * entry.start_nsec;
      return absl::StrCat(
          "{",  //
          "\"name\":\"", ::nlohmann::detail::escape(std::string(entry.name)),
          "@", entry.location.file_name(), ":", entry.location.line(),
          "\", ",                                           //
          "\"ph\":\"X\", ",                                 //
          "\"pid\":", entry.pid, ", ",                      //
          "\"tid\":", entry.tid, ", ",                      //
          "\"cat\":\"", entry.pid, "_", entry.tid, "\", ",  //
          "\"ts\":", start_usec, ", ",                      //
          "\"args\":{", "\"source_location\": ", "\"",
          entry.location.file_name(), ":", entry.location.line(), "\"",
          "},",  //
          //   No trailing ',' for the final parameter.
          "\"dur\":", duration_usec,  //
          "},");
    }
    case TraceEntry::EntryType::kInstantEvent: {
      const uint64_t start_usec = details::kNanoToMicro * entry.start_nsec;
      return absl::StrCat("{",  //
                          "\"name\":\"",
                          ::nlohmann::detail::escape(std::string(entry.name)),
                          "\", ",                                           //
                          "\"ph\":\"i\", ",                                 //
                          "\"cat\":\"", entry.pid, "_", entry.tid, "\", ",  //
                          "\"pid\":", entry.pid, ", ",                      //
                          "\"tid\":", entry.tid, ", ",                      //
                          "\"ts\":", start_usec, ", ",                      //
                          "\"args\":{", "\"source_location\": ", "\"",
                          entry.location.file_name(), ":",
                          entry.location.line(), "\"", "},",  //
                          //   No trailing ',' for the final parameter.
                          "\"s\":\"t\" ",  //
                          "},");
    }
    // Metadata of Counter Events is unfortunately not shown in perfetto.
    case TraceEntry::EntryType::kCounterEvent: {
      const uint64_t start_usec = details::kNanoToMicro * entry.start_nsec;
      return absl::StrCat(
          "{",  //
          "\"name\":\"", ::nlohmann::detail::escape(std::string(entry.name)),
          "@", entry.location.file_name(), ":", entry.location.line(),
          "\", ",                                           //
          "\"ph\":\"C\", ",                                 //
          "\"pid\":", entry.pid, ", ",                      //
          "\"tid\":", entry.tid, ", ",                      //
          "\"cat\":\"", entry.pid, "_", entry.tid, "\", ",  //
          // CounterEvents are process local. Specifying 'id' to
          // differentiate them.
          "\"id\":", entry.tid, ", ",   //
          "\"ts\":", start_usec, ", ",  //
          // Adding string escaped {key,value} for the tracked
          // variable to args, then adding the source location.
          "\"args\":{", "\"",
          ::nlohmann::detail::escape(std::string(entry.args_key)), "\":\"",
          ::nlohmann::detail::escape(std::string(entry.args_value)), "\"",
          "}",  // No trailing ',' for the final parameter.
          "},");
    }
    case TraceEntry::EntryType::kNone: {
      LOG(ERROR) << "Got TraceEntry[" << absl::string_view(entry.name) << "@"
                 << entry.location.file_name() << ":" << entry.location.line()
                 << "] with type 'kNone'!";

      return "";
    }
  }
}

}  // namespace

absl::Status TraceItemCollector::InitializeAndRun(absl::string_view filename) {
  INTRINSIC_ASSERT_NON_REALTIME();
  // Ensure that filename remains valid.
  writer_.Reset(filename);
  if (!writer_.ok()) {
    return absl::InternalError(
        absl::StrCat("Failed to open file writer for '", filename, "'"));
  }
  LOG(INFO) << "Successfully opened the file writer for '" << writer_.filename()
            << "'";

  INTR_ASSIGN_OR_RETURN(
      collector_thread_,
      CreateThread(
          // TODO(b/218310066): Enforce non-realtime priority and scheduler.
          ThreadOptions()
              .SetSkipInitializingThreadLocalTracer()
              .SetName("collector")
              .SetNormalPriorityAndScheduler(),
          [this]() {
            INTRINSIC_ASSERT_NON_REALTIME();
            TraceEntry entry;
            bool shutdown_requested = false;
            while (!shutdown_requested) {
              switch (reader_.Read(entry)) {
                case ReadResult::kClosed: {
                  // The queue is closed. This means a shutdown was requested.
                  shutdown_requested = true;
                  continue;
                }
                case ReadResult::kDeadlineExceeded: {
                  // Can only happen when using ReadWithDeadline.
                  LOG(WARNING) << "Exceeded deadline while reading from the "
                                  "queue. Retrying";
                  continue;
                }
                case ReadResult::kConsumed: {
                  // Successfully read a new entry. Encode and place it into the
                  // file buffer.
                  writer_.Write(ToJSON(entry));
                  break;
                }
              }
              // Flushes the output buffer if tracing is paused. Otherwise
              // entries may not be written to disk until FLush is called on
              // shutdown. Flushing should not create too much overhead, as it
              // is assumed that only a finite number of entries is read from
              // the queue after tracing is paused. If flush is not called here,
              // ICON needs to cleanly shutdown before copying the trace entry
              // files, otherwise entries may be missing.
              if (!details::tracing_active) {
                if (!writer_.Flush()) {
                  LOG(ERROR)
                      << "Failed to flush the output buffer '"
                      << writer_.filename() << "' Status:" << writer_.status();
                }
              }
            }
            // Ensure all data is was written before closing the file.
            if (!writer_.Flush()) {
              LOG(ERROR) << "Failed to flush the output buffer '"
                         << writer_.filename()
                         << "' Status:" << writer_.status();
            }
            if (!writer_.Close()) {
              LOG(ERROR) << "Failed to close the output buffer '"
                         << writer_.filename()
                         << "' Status:" << writer_.status();
            }
            return;
          }));
  return absl::OkStatus();
}

void TraceItemCollector::WaitForShutdown() {
  if (collector_thread_.joinable()) {
    LOG(INFO) << "Waiting for collector thread to shut down.";
    collector_thread_.join();
    LOG(INFO) << "Collector finished shutting down.";
  }
}

Tracer::Tracer() : collector_(to_collector_queue_.Reader()) {
  pid_ = getpid();
  tid_ = GetTID();
}

absl::Status Tracer::InternalInit() {
  std::string filename = absl::StrCat(GetTracePathBase(), "/", "pid", pid_, "-",
                                      "tid", tid_, ".trace.json");

  INTR_RETURN_IF_ERROR(
      file::RecursivelyCreateDir(file::Dirname(filename), file::Defaults()));

  return collector_.InitializeAndRun(filename);
}

Tracer::~Tracer() {
  // Signal shutdown to Collector
  to_collector_queue_.Writer().Close();

  // Wait for the collector to finish writing all data.
  collector_.WaitForShutdown();
}
uint64_t Tracer::GetTimeInNsec() {
  const auto now = std::chrono::high_resolution_clock::now();

  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             now.time_since_epoch())
      .count();
}

void Tracer::AddTrace(TraceEntry entry) {
  if (!is_initialized_) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Trying to add a trace while the tracer is not initialized!";
    return;
  }

  if (entry.pid == 0) {
    entry.pid = pid_;
  }
  if (entry.tid == 0) {
    entry.tid = tid_;
  }
  // Not checking if tracing is active/paused so that a CompleteEvent that was
  // started while tracing was enabled can still be added.
  if (!to_collector_queue_.Writer().Write(entry)) {
    INTRINSIC_RT_LOG_THROTTLED(ERROR)
        << "Failed to send trace entry to collector.";
  }
}

bool Tracer::IsTracingActive() {
  return details::tracing_active.load(std::memory_order_acquire);
}

// static
void Tracer::StartTracing() {
  details::tracing_active.store(true, std::memory_order_release);
}

void Tracer::PauseTracing() {
  details::tracing_active.store(false, std::memory_order_release);
}

// static
bool Tracer::IsTracingEnabled() {
  return details::tracing_enabled.load(std::memory_order_acquire);
}

// static
void Tracer::EnableTracing() {
  details::tracing_enabled.store(true, std::memory_order_release);
}

// static
void Tracer::DisableTracingTestonly() {
  details::tracing_enabled.store(false, std::memory_order_release);
}

// static
absl::Status Tracer::InitializeThreadLocalTracer(
    absl::string_view thread_name) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (!IsTracingEnabled()) {
    return absl::OkStatus();
  }

  auto tracer = GetThreadLocalTracerOrNullptr();
  if (tracer == nullptr) {
    return absl::InternalError(
        "Tracing is enabled, but no tracer was created!");
  }

  // Happens when InitializeThreadLocalTracer is called multiple times.
  if (tracer->is_initialized_) {
    LOG(INFO) << "Skipping additional InitializeThreadLocalTracer for thread["
              << thread_name << "] pid: " << tracer->pid_
              << " tid: " << tracer->tid_;
    return absl::OkStatus();
  }

  tracer->is_initialized_ = true;

  INTR_RETURN_IF_ERROR(tracer->InternalInit());

  // Add a metadata entry to map pid, tid to thread_name.
  tracer->AddTrace(CreateThreadNameEntry(thread_name));

  LOG(INFO) << "Tracing is enabled for thread[" << thread_name
            << "] pid: " << tracer->pid_ << " tid: " << tracer->tid_;

  return absl::OkStatus();
}

// static
Tracer* Tracer::GetThreadLocalTracerOrNullptr() {
  if (!IsTracingEnabled()) {
    return nullptr;
  }

  // Initialize the thread local tracer exactly once. go/totw/110.
  // The destructor needs to be called to ensure that all traces are
  // written. Unlike with global static objects, this is safe to do for a
  // thread_local object.
  static thread_local Tracer tracer;
  return &tracer;
}

// static
const std::string& Tracer::GetTracePathBase() {
  // go/totw/110
  static const absl::NoDestructor<std::string> path(absl::StrCat(
      absl::GetFlag(FLAGS_trace_dir), "/", absl::FormatTime(absl::Now())));
  return *path;
}

}  // namespace intrinsic::tracing
