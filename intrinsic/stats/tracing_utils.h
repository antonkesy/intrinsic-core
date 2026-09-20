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

#ifndef INTRINSIC_STATS_TRACING_UTILS_H_
#define INTRINSIC_STATS_TRACING_UTILS_H_

#include <memory>
#include <string>

#include "absl/base/attributes.h"
#include "absl/strings/string_view.h"
#include "grpcpp/server_context.h"
#include "opentelemetry/trace/context.h"
#include "opentelemetry/trace/provider.h"
#include "opentelemetry/trace/span.h"
#include "opentelemetry/trace/span_context.h"
#include "opentelemetry/trace/span_id.h"
#include "opentelemetry/trace/tracer.h"

namespace intrinsic::stats {

// Returns the active OpenTelemetry span from the given gRPC server context.
//
// This is used instead of `opentelemetry::trace::Tracer::GetCurrentSpan()`
// because the gRPC OpenTelemetry plugin does not yet propagate active span
// context properly (see https://github.com/grpc/grpc/issues/19092).
//
// Returns a non-owning `std::shared_ptr` wrapping the span attached to
// `context`. Callers must not store or retain this pointer beyond the lifetime
// of the RPC request.
std::shared_ptr<opentelemetry::trace::Span> GetSpanFromServerContext(
    grpc::ServerContext* context ABSL_ATTRIBUTE_LIFETIME_BOUND);

// Returns a default global OpenTelemetry tracer ("intrinsic-c++").
//
// In OpenTelemetry, the preferred practice is for each individual component or
// library to instantiate its own named tracer from the provider (e.g., via
// `opentelemetry::trace::Provider::GetTracerProvider()->GetTracer("my_lib")`).
// However, for simplicity and shared infrastructure, callers may use this
// global convenience method.
std::shared_ptr<opentelemetry::trace::Tracer> GetTracer();

std::string GetSpanIdHex(const opentelemetry::trace::SpanId& span_id);

std::string GetSpanIdHex(const opentelemetry::trace::SpanContext& ctx);
std::string GetTraceIdHex(const opentelemetry::trace::SpanContext& ctx);

std::shared_ptr<opentelemetry::trace::Span> GetCurrentSpan();

// Starts a root trace span with a forced sampling decision override.
//
// By default (`sample_decision = true`), this forces the root span and its
// descendants to be sampled and recorded. Setting `sample_decision = false` is
// configurable for cases when spans are disabled by default and we only want to
// conditionally enable specific trace sub-trees (e.g., for targeted debugging).
std::shared_ptr<opentelemetry::trace::Span> StartSampledRootSpan(
    absl::string_view name, bool sample_decision = true);

}  // namespace intrinsic::stats

#endif  // INTRINSIC_STATS_TRACING_UTILS_H_
