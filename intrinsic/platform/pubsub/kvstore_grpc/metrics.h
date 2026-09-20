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

#ifndef INTRINSIC_PLATFORM_PUBSUB_KVSTORE_GRPC_METRICS_H_
#define INTRINSIC_PLATFORM_PUBSUB_KVSTORE_GRPC_METRICS_H_

#include "absl/status/status.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/support/status.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "opencensus/stats/stats.h"
#include "opencensus/tags/tag_key.h"

namespace intrinsic::kvstore {

// Tag key to represent the status of a gRPC operation.
opencensus::tags::TagKey StatusCodeKey();

// Tag key to represent whether the Set request required high consistency.
opencensus::tags::TagKey HighConsistencyKey();

// Base context for RPC metrics.
struct KvstoreGrpcBaseRpcMetricsContext {
  KvstoreGrpcBaseRpcMetricsContext() : start_time(absl::Now()) {}

  // Delete copy and move constructors to prevent accidental double-recording.
  KvstoreGrpcBaseRpcMetricsContext(const KvstoreGrpcBaseRpcMetricsContext&) =
      delete;
  KvstoreGrpcBaseRpcMetricsContext& operator=(
      const KvstoreGrpcBaseRpcMetricsContext&) = delete;
  KvstoreGrpcBaseRpcMetricsContext(KvstoreGrpcBaseRpcMetricsContext&&) = delete;
  KvstoreGrpcBaseRpcMetricsContext& operator=(
      KvstoreGrpcBaseRpcMetricsContext&&) = delete;

  absl::Time start_time;
  absl::Status status = absl::UnknownError("Pending");

  // Stores the status and returns it as a grpc::Status for elegant 1-liners.
  grpc::Status CaptureAndReturnGrpcStatus(const absl::Status& s) {
    status = s;
    return intrinsic::ToGrpcStatus(status);
  }
};

struct GetMetricsScope : public KvstoreGrpcBaseRpcMetricsContext {
  GetMetricsScope() = default;
  ~GetMetricsScope();

  static opencensus::stats::MeasureInt64 RequestsReceived();
  static opencensus::stats::MeasureInt64 DurationNanos();

  static void RegisterViews();
};

struct SetMetricsScope : public KvstoreGrpcBaseRpcMetricsContext {
  explicit SetMetricsScope(bool high_consistency_val)
      : high_consistency(high_consistency_val) {}
  ~SetMetricsScope();

  bool high_consistency = false;
  int64_t payload_bytes = 0;

  void RecordPayloadSize(int64_t bytes) { payload_bytes = bytes; }

  static opencensus::stats::MeasureInt64 RequestsReceived();
  static opencensus::stats::MeasureInt64 DurationNanos();
  static opencensus::stats::MeasureInt64 PayloadSizeBytes();

  static void RegisterViews();
};

struct DeleteMetricsScope : public KvstoreGrpcBaseRpcMetricsContext {
  DeleteMetricsScope() = default;
  ~DeleteMetricsScope();

  static opencensus::stats::MeasureInt64 RequestsReceived();
  static opencensus::stats::MeasureInt64 DurationNanos();

  static void RegisterViews();
};

struct ListMetricsScope : public KvstoreGrpcBaseRpcMetricsContext {
  ListMetricsScope() = default;
  ~ListMetricsScope();

  static opencensus::stats::MeasureInt64 RequestsReceived();
  static opencensus::stats::MeasureInt64 DurationNanos();

  static void RegisterViews();
};

}  // namespace intrinsic::kvstore

#endif  // INTRINSIC_PLATFORM_PUBSUB_KVSTORE_GRPC_METRICS_H_
