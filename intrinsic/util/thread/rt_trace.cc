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

#include "intrinsic/util/thread/rt_trace.h"

#include <utility>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/release/source_location.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/util/thread/rt_trace_internal.h"

namespace intrinsic::tracing {

ScopedTrace::ScopedTrace(absl::string_view name, TracerInterface* tracer,
                         intrinsic::SourceLocation location) {
  // Do nothing if tracer == nullptr, or tracing is not active.
  if (!tracer || !tracer->IsTracingActive()) {
    return;
  }

  tracer_ = tracer;
  name_ = name;
  start_nsec_ = tracer->GetTimeInNsec();
  location_ = std::move(location);
}

ScopedTrace::~ScopedTrace() {
  // Not checking if tracing is active, as the ScopedTrace was started while
  // tracing was active.
  if (!tracer_) {
    return;
  }

  tracer_->AddTrace({.type = TraceEntry::EntryType::kCompleteEvent,
                     .start_nsec = start_nsec_,
                     .end_nsec = tracer_->GetTimeInNsec(),
                     .name = name_,
                     .location = std::move(location_)});
}

void LogInstantEvent(absl::string_view name, TracerInterface* tracer,
                     intrinsic::SourceLocation location) {
  if (!tracer || !tracer->IsTracingActive()) {
    return;
  }
  tracer->AddTrace({.type = TraceEntry::EntryType::kInstantEvent,
                    .start_nsec = tracer->GetTimeInNsec(),
                    .name = name,
                    .location = std::move(location)});
}

}  // namespace intrinsic::tracing
