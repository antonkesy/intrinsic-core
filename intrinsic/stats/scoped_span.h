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

#ifndef INTRINSIC_STATS_SCOPED_SPAN_H_
#define INTRINSIC_STATS_SCOPED_SPAN_H_

#include <memory>

#include "absl/strings/string_view.h"
#include "grpcpp/server_context.h"
#include "opentelemetry/common/attribute_value.h"
#include "opentelemetry/trace/scope.h"
#include "opentelemetry/trace/span.h"
#include "opentelemetry/trace/span_context.h"
#include "opentelemetry/trace/span_startoptions.h"
#include "opentelemetry/trace/tracer.h"

namespace intrinsic::stats {

// ScopedSpan is a scoped object which holds a span and a scope that ensures
// that during the ScopedSpan's lifetime, its internal span is set as the
// active span in the current context.
// Semantically, the class is identical to the following usage pattern:
// {
//   auto span = tracer->StartSpan(name, options);
//   Scope scope(span);
//   ...
//   span->End();
// }
class ScopedSpan {
 public:
  explicit ScopedSpan(std::shared_ptr<opentelemetry::trace::Span> span);
  ScopedSpan(absl::string_view name,
             const opentelemetry::trace::StartSpanOptions& options =
                 opentelemetry::trace::StartSpanOptions());
  ScopedSpan(absl::string_view name,
             const std::shared_ptr<opentelemetry::trace::Span>& parent,
             const opentelemetry::trace::StartSpanOptions& options =
                 opentelemetry::trace::StartSpanOptions());
  ScopedSpan(absl::string_view name, grpc::ServerContext* context);

  // Add an attribute to the internal span.
  void AddAttribute(absl::string_view key,
                    const opentelemetry::common::AttributeValue& value) const;

  ~ScopedSpan();

  ScopedSpan(const ScopedSpan&) = delete;
  ScopedSpan& operator=(const ScopedSpan&) = delete;

  ScopedSpan(ScopedSpan&&) = default;
  ScopedSpan& operator=(ScopedSpan&&) = default;

  // Returns the SpanContext associated with the internal Span.
  opentelemetry::trace::SpanContext context() const;

  // Returns the internal span to be used for parenting new spans.
  std::shared_ptr<opentelemetry::trace::Span> span() const { return span_; }

 private:
  std::shared_ptr<opentelemetry::trace::Span> span_;
  std::unique_ptr<opentelemetry::trace::Scope> scope_;
};

}  // namespace intrinsic::stats

#endif  // INTRINSIC_STATS_SCOPED_SPAN_H_
