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

#ifndef INTRINSIC_STATS_OPENCENSUS_INTERNAL_H_
#define INTRINSIC_STATS_OPENCENSUS_INTERNAL_H_

#include "absl/flags/declare.h"
#include "opencensus/stats/stats.h"
#include "opentelemetry/sdk/trace/sampler.h"
#include "opentelemetry/trace/span_context.h"

// Exposed for testing.
ABSL_DECLARE_FLAG(int, opencensus_metrics_port);
ABSL_DECLARE_FLAG(bool, opencensus_tracing);

namespace intrinsic {
namespace internal {

// Exposed for testing.
::opencensus::stats::ViewDescriptor RpcCountViewDescriptor();

class InPlaceSampler : public opentelemetry::sdk::trace::Sampler {
 public:
  explicit InPlaceSampler(
      std::shared_ptr<opentelemetry::sdk::trace::Sampler> default_sampler);

  opentelemetry::sdk::trace::SamplingResult ShouldSample(
      const opentelemetry::trace::SpanContext& parent_context,
      opentelemetry::trace::TraceId trace_id, std::string_view name,
      opentelemetry::trace::SpanKind span_kind,
      const opentelemetry::common::KeyValueIterable& attributes,
      const opentelemetry::trace::SpanContextKeyValueIterable& links) noexcept
      override;

  std::string_view GetDescription() const noexcept override;

 private:
  const std::shared_ptr<opentelemetry::sdk::trace::Sampler> default_sampler_;
  const std::string description_;
};

}  // namespace internal
}  // namespace  intrinsic

#endif  // INTRINSIC_STATS_OPENCENSUS_INTERNAL_H_
