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

#include "intrinsic/stats/tracing_utils.h"

#include <array>
#include <memory>
#include <string>

#include "intrinsic/stats/internal_constants.h"
#include "opentelemetry/context/context.h"
#include "opentelemetry/trace/default_span.h"
#include "opentelemetry/trace/span.h"
#include "opentelemetry/trace/span_id.h"
#include "opentelemetry/trace/trace_id.h"

namespace intrinsic::stats {

constexpr char kTracerName[] = "intrinsic-c++";

namespace trace_api = opentelemetry::trace;

std::shared_ptr<trace_api::Span> GetSpanFromServerContext(
    grpc::ServerContext* context ABSL_ATTRIBUTE_LIFETIME_BOUND) {
  if (context == nullptr || context->census_context() == nullptr) {
    // Create empty, non-recording span with invalid context
    return std::make_shared<trace_api::DefaultSpan>(
        trace_api::SpanContext::GetInvalid());
  }
  // gRPC's ServerContext currently reuses the legacy census_context() storage
  // to store the active OpenTelemetry span (see
  // https://github.com/grpc/grpc/issues/36639). Casting census_context to
  // opentelemetry::trace::Span is safe as long as only the OpenTelemetry gRPC
  // plugin is enabled.
  trace_api::Span* raw_span = reinterpret_cast<trace_api::Span*>(
      const_cast<census_context*>(context->census_context()));
  // Returns a non-owning shared_ptr with a no-op deleter because the underlying
  // span is owned by the gRPC ServerContext for the duration of the request.
  return std::shared_ptr<trace_api::Span>(raw_span, [](trace_api::Span*) {});
}

std::shared_ptr<trace_api::Tracer> GetTracer() {
  return trace_api::Provider::GetTracerProvider()->GetTracer(kTracerName);
}

std::string GetSpanIdHex(const trace_api::SpanId& span_id) {
  std::array<char, 2 * trace_api::SpanId::kSize> span_id_hex;
  span_id.ToLowerBase16(span_id_hex);
  return std::string(span_id_hex.data(), span_id_hex.size());
}

std::string GetSpanIdHex(const trace_api::SpanContext& ctx) {
  return GetSpanIdHex(ctx.span_id());
}

std::string GetTraceIdHex(const trace_api::SpanContext& ctx) {
  std::array<char, 2 * trace_api::TraceId::kSize> trace_id_hex;
  ctx.trace_id().ToLowerBase16(trace_id_hex);
  return std::string(trace_id_hex.data(), trace_id_hex.size());
}

std::shared_ptr<trace_api::Span> GetCurrentSpan() {
  return trace_api::Tracer::GetCurrentSpan();
}

std::shared_ptr<opentelemetry::trace::Span> StartSampledRootSpan(
    absl::string_view name, bool sample_decision) {
  // In OpenTelemetry C++, creating a root span (ignoring any active parent span
  // in the current thread context) requires passing an explicit parent context
  // with `kIsRootSpanKey` set to true. See API example in
  // https://github.com/open-telemetry/opentelemetry-cpp/tree/5c60fa85fd1088dfa09af6f8edd0ea4aba654d16/examples/explicit_parent
  return GetTracer()->StartSpan(
      name, {{kSamplingOverrideAttributeKey, sample_decision}},
      {.parent = opentelemetry::context::Context().SetValue(
           trace_api::kIsRootSpanKey, true)});
}

}  // namespace intrinsic::stats
