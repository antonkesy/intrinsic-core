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

#include "intrinsic/logging/log_item_builder.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "google/longrunning/operations.pb.h"
#include "google/protobuf/timestamp.pb.h"
#include "intrinsic/executive/proto/log_items.pb.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/logging/errors/proto/error_report.pb.h"
#include "intrinsic/logging/proto/blob.pb.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/logging/proto/log_item.pb.h"
#include "intrinsic/motion_planning/path_planning/path_planner.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/util/status/status_conversion_rpc.h"
#include "intrinsic/world/service/world_compatibility_service.pb.h"
#include "intrinsic/world/world.pb.h"

namespace intrinsic::data_logger {

// Builder.

Builder::Builder(intrinsic_proto::data_logger::LogItem&& item)
    : item_(std::move(item)) {}

Builder Builder::From(const google::protobuf::Any& any_proto) {
  intrinsic_proto::data_logger::LogItem log;
  *log.mutable_payload()->mutable_any() = any_proto;
  return Builder(std::move(log));
}

Builder Builder::From(
    const intrinsic_proto::executive::LoggedOperation& logged_operation,
    bool is_for_history) {
  intrinsic_proto::data_logger::LogItem log;
  if (is_for_history) {
    log.mutable_metadata()->set_event_source("executive.operation_history");
  } else {
    log.mutable_metadata()->set_event_source("executive.operation_state");
  }
  *log.mutable_payload()->mutable_executive_operation() = logged_operation;
  return Builder(std::move(log));
}

Builder Builder::From(const intrinsic_proto::icon::JointState& joint_state) {
  intrinsic_proto::data_logger::LogItem log;
  *log.mutable_payload()->mutable_icon_l1_joint_state() = joint_state;

  return Builder(std::move(log));
}

Builder Builder::From(const intrinsic_proto::icon::RobotStatus& robot_status) {
  intrinsic_proto::data_logger::LogItem log;
  *log.mutable_payload()->mutable_icon_robot_status() = robot_status;
  return Builder(std::move(log));
}

Builder Builder::From(
    const intrinsic_proto::skills::ExecutionSummary& summary) {
  intrinsic_proto::data_logger::LogItem log;
  log.mutable_metadata()->set_event_source("skills.execution_summary");
  *log.mutable_payload()->mutable_skills_execution_summary() = summary;
  return Builder(std::move(log));
}

Builder Builder::From(const intrinsic_proto::status::ExtendedStatus& status) {
  intrinsic_proto::data_logger::LogItem log;
  log.mutable_metadata()->set_event_source("executive.extended_status");
  *log.mutable_payload()->mutable_executive_process_status() = status;
  return Builder(std::move(log));
}

Builder& Builder::WithEventSource(std::string_view event_source) & {
  item_.mutable_metadata()->set_event_source(event_source);
  return *this;
}

Builder&& Builder::WithEventSource(std::string_view event_source) && {
  item_.mutable_metadata()->set_event_source(event_source);
  return std::move(*this);
}

Builder& Builder::WithContext(
    const intrinsic_proto::data_logger::Context& context) & {
  *item_.mutable_context() = context;
  return *this;
}

Builder&& Builder::WithContext(
    const intrinsic_proto::data_logger::Context& context) && {
  *item_.mutable_context() = context;
  return std::move(*this);
}

Builder& Builder::WithUid(uint64_t uid) & {
  item_.mutable_metadata()->set_uid(uid);
  return *this;
}

Builder&& Builder::WithUid(uint64_t uid) && {
  item_.mutable_metadata()->set_uid(uid);
  return std::move(*this);
}

const intrinsic_proto::data_logger::LogItem& Builder::Item() const& {
  return item_;
}

intrinsic_proto::data_logger::LogItem&& Builder::Item() && {
  return std::move(item_);
}

// ErrorBuilder.

ErrorBuilder ErrorBuilder::From(const absl::Status& status,
                                std::string_view human_readable_summary) {
  intrinsic_proto::error::ErrorReport report;
  *report.mutable_description()->mutable_status() =
      SaveStatusAsRpcStatus(status);
  report.mutable_description()->set_human_readable_summary(
      human_readable_summary);
  return ErrorBuilder(report);
}

absl::Status ErrorBuilder::Valid() const {
  if (item_.payload()
          .error_report()
          .description()
          .human_readable_summary()
          .empty()) {
    return absl::InvalidArgumentError(
        "Report should contain a non-empty 'human_readable_summary'.");
  }
  if (std::vector<std::string>(absl::StrSplit(item_.payload()
                                                  .error_report()
                                                  .description()
                                                  .human_readable_summary(),
                                              ' '))
          .size() >= 10) {
    return absl::InvalidArgumentError(
        "'human_readable_summary' should be shorter than 10 words.");
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::data_logger
