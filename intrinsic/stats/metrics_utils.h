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

#ifndef INTRINSIC_STATS_METRICS_UTILS_H_
#define INTRINSIC_STATS_METRICS_UTILS_H_

#include <memory>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "opentelemetry/metrics/meter.h"

namespace intrinsic::stats {

// Returns the default global OpenTelemetry meter for Intrinsic C++ code.
//
// Must be called AFTER the global MeterProvider has been initialized (e.g.,
// after OpenCensusPlugin is constructed in main()).
std::shared_ptr<opentelemetry::metrics::Meter> GetMeter();

// Registers a custom View for one or more histogram instruments to customize
// their bucket boundaries and/or rename them.
//
// Must be called AFTER the global SDK MeterProvider is initialized (otherwise
// this will silently do nothing) and BEFORE the targeted instruments are
// created (i.e., before the first Record() or getter call).
//
// This helper supports two primary use cases:
// 1. Single instrument renaming & customization: Match a specific instrument
//    name and rename it (e.g., match "my_metric" and rename to
//    "my_metric/distribution" with custom boundaries).
// 2. Wildcard boundary customization (no rename): Match multiple instruments
//    using wildcards and apply custom boundaries while preserving their
//    original names (e.g., match "grpc.*.*.duration" and pass an empty
//    `view_name`).
//
// Args:
//   instrument_name: The name of the instrument(s) to match. Can contain
//     wildcards (e.g., "grpc.*.*.duration").
//   view_name: The desired name of the exported metric. If empty, the original
//     instrument name is preserved (essential when using wildcards to avoid
//     merging them into a single metric). If non-empty, the metric is exported
//     under the new name instead of the original one.
//   boundaries: The custom bucket boundaries to apply to the matched
//   histograms.
void RegisterHistogramView(absl::string_view instrument_name,
                           absl::string_view view_name,
                           absl::Span<const double> boundaries);

// Creates histogram boundaries with num_finite_buckets each 'width' wide,
// as well as an underflow and overflow bucket. 'offset' is the lower bound of
// the first finite bucket, so finite bucket i (1 <= i <= num_finite_buckets)
// covers the interval [offset + (i - 1) * width, offset + i * width). The
// underflow bucket covers [-inf, offset) and the overflow bucket [offset +
// num_finite_buckets * width, inf].
std::vector<double> LinearBucketBoundaries(int num_finite_buckets,
                                           double offset, double width);

// Creates histogram boundaries with num_finite_buckets with exponentially
// increasing boundaries starting at zero (governed by growth_factor and
// scale), as well as an underflow and overflow bucket. Finite bucket 0
// covers [0, scale), and bucket i (1 <= i < num_finite_buckets) covers
// the interval [scale * growth_factor ^ (i - 1), scale * growth_factor ^ i).
// The underflow bucket covers [-inf, 0) and the overflow bucket
// [scale * growth_factor ^ (num_finite_buckets - 1), inf].
std::vector<double> ExponentialBucketBoundaries(int num_finite_buckets,
                                                double scale,
                                                double growth_factor);

}  // namespace intrinsic::stats

#endif  // INTRINSIC_STATS_METRICS_UTILS_H_
