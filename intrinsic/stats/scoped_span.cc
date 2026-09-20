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

#include "intrinsic/stats/scoped_span.h"

#include <memory>
#include <utility>

#include "absl/strings/string_view.h"
#include "grpcpp/server_context.h"
#include "intrinsic/stats/tracing_utils.h"
#include "opentelemetry/context/context.h"

namespace intrinsic::stats {

namespace trace_api = opentelemetry::trace;

ScopedSpan::ScopedSpan(std::shared_ptr<trace_api::Span> span)
    : span_(std::move(span)) {
  if (span_) {
    scope_ = std::make_unique<trace_api::Scope>(span_);
  }
}

ScopedSpan::ScopedSpan(absl::string_view name,
                       const trace_api::StartSpanOptions& options)
    : ScopedSpan(GetTracer()->StartSpan(name, options)) {}

ScopedSpan::ScopedSpan(absl::string_view name,
                       const std::shared_ptr<trace_api::Span>& parent,
                       const trace_api::StartSpanOptions& options) {
  trace_api::StartSpanOptions local_options = options;
  if (parent && parent->GetContext().IsValid()) {
    // Propagate provider parent context
    local_options.parent = parent->GetContext();
  } else {
    // Create new root span
    local_options.parent = opentelemetry::context::Context().SetValue(
        trace_api::kIsRootSpanKey, true);
  }
  span_ = GetTracer()->StartSpan(name, local_options);
  scope_ = std::make_unique<trace_api::Scope>(span_);
}

ScopedSpan::ScopedSpan(absl::string_view name, grpc::ServerContext* context)
    : ScopedSpan(name, GetSpanFromServerContext(context)) {}

ScopedSpan::~ScopedSpan() {
  scope_.reset();
  if (span_) {
    span_->End();
  }
}

void ScopedSpan::AddAttribute(
    absl::string_view key,
    const opentelemetry::common::AttributeValue& value) const {
  if (span_) {
    span_->SetAttribute(key, value);
  }
}

trace_api::SpanContext ScopedSpan::context() const {
  return span_ ? span_->GetContext() : trace_api::SpanContext::GetInvalid();
}

}  // namespace intrinsic::stats
