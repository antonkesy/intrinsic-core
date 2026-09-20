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

#include "intrinsic/motion_planning/service/debug/timing_debug_logger.h"

#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/protobuf/json/json.h"
#include "google/protobuf/struct.pb.h"
#include "intrinsic/logging/data_logger_client.h"
#include "intrinsic/logging/proto/log_item.pb.h"

namespace intrinsic {
namespace {

std::string TimeToString(absl::Time time) {
  return absl::FormatTime(absl::RFC3339_full, time, absl::LocalTimeZone());
}

}  // namespace

TimingDebugLogger::TimingDebugLogger(std::string name,
                                     std::string starting_interval_name) {
  last_interval_start_ = absl::Now();
  log_.set_start_time(TimeToString(last_interval_start_));
  log_.mutable_interval()->set_name(std::move(name));

  // Start the first interval.
  interval_.set_name(std::move(starting_interval_name));
}

TimingDebugLogger::~TimingDebugLogger() {
  // Arbitrary string - will not be logged.
  Start("<end>");

  // Log the final result.
  google::protobuf::json::PrintOptions options;
  options.add_whitespace = false;
  options.preserve_proto_field_names = true;

  std::string out;
  auto status = google::protobuf::json::MessageToJsonString(log_, &out);
  if (!status.ok()) {
    LOG(INFO) << "Failed to convert timing debug to json: " << status;
  } else {
    LOG(INFO) << "TIMING-DEBUG-LOG:" << out;
  }

}

void TimingDebugLogger::Start(std::string interval_name) {
  // Add the time and data to the current interval object.
  absl::Time interval_end = absl::Now();
  interval_.set_seconds(
      absl::ToDoubleSeconds(interval_end - last_interval_start_));

  log_.mutable_interval()->mutable_intervals()->Add(std::move(interval_));

  // Reset the interval_obj_ and other state.
  last_interval_start_ = interval_end;
  interval_.Clear();
  interval_.set_name(std::move(interval_name));
}

void TimingDebugLogger::Attach(std::string key, std::string value) {
  (*interval_.mutable_data()->mutable_fields())[std::move(key)]
      .set_string_value(std::move(value));
}

void TimingDebugLogger::AttachTop(std::string key, std::string value) {
  (*log_.mutable_interval()->mutable_data()->mutable_fields())[std::move(key)]
      .set_string_value(std::move(value));
}

}  // namespace intrinsic
