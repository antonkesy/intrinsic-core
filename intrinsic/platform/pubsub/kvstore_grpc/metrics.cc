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

#include "intrinsic/platform/pubsub/kvstore_grpc/metrics.h"

#include <string>

#include "absl/status/status.h"
#include "intrinsic/production/external/googleinit/googleinit.h"
#include "opencensus/stats/stats.h"
#include "opencensus/tags/tag_map.h"

namespace intrinsic::kvstore {

using ::opencensus::stats::Aggregation;
using ::opencensus::stats::BucketBoundaries;
using ::opencensus::stats::MeasureInt64;
using ::opencensus::stats::Record;
using ::opencensus::stats::ViewDescriptor;
using ::opencensus::tags::TagMap;

constexpr char kNanoSeconds[] = "ns";
constexpr char kBytes[] = "bytes";

namespace {

ViewDescriptor BaseViewWithStatus() {
  return ViewDescriptor().add_column(StatusCodeKey());
}

}  // namespace

opencensus::tags::TagKey StatusCodeKey() {
  static const opencensus::tags::TagKey key =
      opencensus::tags::TagKey::Register("grpc_status");
  return key;
}

opencensus::tags::TagKey HighConsistencyKey() {
  static const opencensus::tags::TagKey key =
      opencensus::tags::TagKey::Register("high_consistency");
  return key;
}

// Get

constexpr char kGetRequestsReceivedName[] =
    "intrinsic/kvstore_grpc/get_requests";
constexpr char kGetRequestsReceivedDescription[] = "Number of Get requests";

constexpr char kGetDurationNanosName[] =
    "intrinsic/kvstore_grpc/get_duration_nanos";
constexpr char kGetDurationNanosDescription[] =
    "Time spent in KVStore Get request in nanoseconds";

MeasureInt64 GetMetricsScope::RequestsReceived() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kGetRequestsReceivedName, kGetRequestsReceivedDescription, "1");
  return measure;
}

MeasureInt64 GetMetricsScope::DurationNanos() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kGetDurationNanosName, kGetDurationNanosDescription, kNanoSeconds);
  return measure;
}

GetMetricsScope::~GetMetricsScope() {
  std::string status_str = absl::StatusCodeToString(status.code());
  const TagMap tags = {{StatusCodeKey(), status_str}};

  Record(
      {{RequestsReceived(), 1},
       {DurationNanos(), absl::ToInt64Nanoseconds(absl::Now() - start_time)}},
      tags);
}

void GetMetricsScope::RegisterViews() {
  GetMetricsScope::RequestsReceived();
  GetMetricsScope::DurationNanos();

  BaseViewWithStatus()
      .set_name(kGetRequestsReceivedName)
      .set_measure(kGetRequestsReceivedName)
      .set_description(kGetRequestsReceivedDescription)
      .set_aggregation(Aggregation::Count())
      .RegisterForExport();

  BaseViewWithStatus()
      .set_name(kGetDurationNanosName)
      .set_measure(kGetDurationNanosName)
      .set_description(kGetDurationNanosDescription)
      .set_aggregation(Aggregation::Distribution(BucketBoundaries::Exponential(
          /* num_finite_buckets= */ 15, /* scale= */ 50000,
          /* growth_factor= */ 2.0)))
      .RegisterForExport();
}

// Set

constexpr char kSetRequestsReceivedName[] =
    "intrinsic/kvstore_grpc/set_requests";
constexpr char kSetRequestsReceivedDescription[] = "Number of Set requests";

constexpr char kSetDurationNanosName[] =
    "intrinsic/kvstore_grpc/set_duration_nanos";
constexpr char kSetDurationNanosDescription[] =
    "Time spent in KVStore Set request in nanoseconds";

constexpr char kSetPayloadSizeBytesName[] =
    "intrinsic/kvstore_grpc/set_payload_size_bytes";
constexpr char kSetPayloadSizeBytesDescription[] =
    "Size of KVStore Set request payload in bytes";

MeasureInt64 SetMetricsScope::RequestsReceived() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kSetRequestsReceivedName, kSetRequestsReceivedDescription, "1");
  return measure;
}

MeasureInt64 SetMetricsScope::DurationNanos() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kSetDurationNanosName, kSetDurationNanosDescription, kNanoSeconds);
  return measure;
}

MeasureInt64 SetMetricsScope::PayloadSizeBytes() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kSetPayloadSizeBytesName, kSetPayloadSizeBytesDescription, kBytes);
  return measure;
}

SetMetricsScope::~SetMetricsScope() {
  std::string status_str = absl::StatusCodeToString(status.code());
  std::string high_consistency_str = high_consistency ? "true" : "false";

  const TagMap tags = {{StatusCodeKey(), status_str},
                       {HighConsistencyKey(), high_consistency_str}};

  Record(
      {{RequestsReceived(), 1},
       {DurationNanos(), absl::ToInt64Nanoseconds(absl::Now() - start_time)}},
      tags);

  if (payload_bytes > 0) {
    Record({{PayloadSizeBytes(), payload_bytes}}, tags);
  }
}

void SetMetricsScope::RegisterViews() {
  SetMetricsScope::RequestsReceived();
  SetMetricsScope::DurationNanos();
  SetMetricsScope::PayloadSizeBytes();

  BaseViewWithStatus()
      .add_column(HighConsistencyKey())
      .set_name(kSetRequestsReceivedName)
      .set_measure(kSetRequestsReceivedName)
      .set_description(kSetRequestsReceivedDescription)
      .set_aggregation(Aggregation::Count())
      .RegisterForExport();

  BaseViewWithStatus()
      .add_column(HighConsistencyKey())
      .set_name(kSetDurationNanosName)
      .set_measure(kSetDurationNanosName)
      .set_description(kSetDurationNanosDescription)
      .set_aggregation(Aggregation::Distribution(BucketBoundaries::Exponential(
          /* num_finite_buckets= */ 15, /* scale= */ 50000,
          /* growth_factor= */ 2.0)))
      .RegisterForExport();

  BaseViewWithStatus()
      .add_column(HighConsistencyKey())
      .set_name(kSetPayloadSizeBytesName)
      .set_measure(kSetPayloadSizeBytesName)
      .set_description(kSetPayloadSizeBytesDescription)
      .set_aggregation(Aggregation::Distribution(BucketBoundaries::Exponential(
          /* num_finite_buckets= */ 20, /* scale= */ 100,
          /* growth_factor= */ 2.0)))
      .RegisterForExport();
}

// Delete

constexpr char kDeleteRequestsReceivedName[] =
    "intrinsic/kvstore_grpc/delete_requests";
constexpr char kDeleteRequestsReceivedDescription[] =
    "Number of Delete requests";

constexpr char kDeleteDurationNanosName[] =
    "intrinsic/kvstore_grpc/delete_duration_nanos";
constexpr char kDeleteDurationNanosDescription[] =
    "Time spent in KVStore Delete request in nanoseconds";

MeasureInt64 DeleteMetricsScope::RequestsReceived() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kDeleteRequestsReceivedName, kDeleteRequestsReceivedDescription, "1");
  return measure;
}

MeasureInt64 DeleteMetricsScope::DurationNanos() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kDeleteDurationNanosName, kDeleteDurationNanosDescription, kNanoSeconds);
  return measure;
}

DeleteMetricsScope::~DeleteMetricsScope() {
  std::string status_str = absl::StatusCodeToString(status.code());
  const TagMap tags = {{StatusCodeKey(), status_str}};

  Record(
      {{RequestsReceived(), 1},
       {DurationNanos(), absl::ToInt64Nanoseconds(absl::Now() - start_time)}},
      tags);
}

void DeleteMetricsScope::RegisterViews() {
  DeleteMetricsScope::RequestsReceived();
  DeleteMetricsScope::DurationNanos();

  BaseViewWithStatus()
      .set_name(kDeleteRequestsReceivedName)
      .set_measure(kDeleteRequestsReceivedName)
      .set_description(kDeleteRequestsReceivedDescription)
      .set_aggregation(Aggregation::Count())
      .RegisterForExport();

  BaseViewWithStatus()
      .set_name(kDeleteDurationNanosName)
      .set_measure(kDeleteDurationNanosName)
      .set_description(kDeleteDurationNanosDescription)
      .set_aggregation(Aggregation::Distribution(BucketBoundaries::Exponential(
          /* num_finite_buckets= */ 15, /* scale= */ 50000,
          /* growth_factor= */ 2.0)))
      .RegisterForExport();
}

// List

constexpr char kListRequestsReceivedName[] =
    "intrinsic/kvstore_grpc/list_requests";
constexpr char kListRequestsReceivedDescription[] = "Number of List requests";

constexpr char kListDurationNanosName[] =
    "intrinsic/kvstore_grpc/list_duration_nanos";
constexpr char kListDurationNanosDescription[] =
    "Time spent in KVStore List request in nanoseconds";

MeasureInt64 ListMetricsScope::RequestsReceived() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kListRequestsReceivedName, kListRequestsReceivedDescription, "1");
  return measure;
}

MeasureInt64 ListMetricsScope::DurationNanos() {
  static const MeasureInt64 measure = MeasureInt64::Register(
      kListDurationNanosName, kListDurationNanosDescription, kNanoSeconds);
  return measure;
}

ListMetricsScope::~ListMetricsScope() {
  std::string status_str = absl::StatusCodeToString(status.code());
  const TagMap tags = {{StatusCodeKey(), status_str}};

  Record(
      {{RequestsReceived(), 1},
       {DurationNanos(), absl::ToInt64Nanoseconds(absl::Now() - start_time)}},
      tags);
}

void ListMetricsScope::RegisterViews() {
  ListMetricsScope::RequestsReceived();
  ListMetricsScope::DurationNanos();

  BaseViewWithStatus()
      .set_name(kListRequestsReceivedName)
      .set_measure(kListRequestsReceivedName)
      .set_description(kListRequestsReceivedDescription)
      .set_aggregation(Aggregation::Count())
      .RegisterForExport();

  BaseViewWithStatus()
      .set_name(kListDurationNanosName)
      .set_measure(kListDurationNanosName)
      .set_description(kListDurationNanosDescription)
      .set_aggregation(Aggregation::Distribution(BucketBoundaries::Exponential(
          /* num_finite_buckets= */ 15, /* scale= */ 50000,
          /* growth_factor= */ 2.0)))
      .RegisterForExport();
}

REGISTER_MODULE_INITIALIZER(kvstore_grpc_metrics, {
  GetMetricsScope::RegisterViews();
  SetMetricsScope::RegisterViews();
  DeleteMetricsScope::RegisterViews();
  ListMetricsScope::RegisterViews();
});

}  // namespace intrinsic::kvstore
