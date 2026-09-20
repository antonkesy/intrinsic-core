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

#include "intrinsic/icon/control/parts/realtime_part.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/thread/rt_trace.h"

namespace intrinsic::icon {

RealtimePart::RealtimePart(absl::string_view name,
                           std::unique_ptr<RealtimePartInterface> part)
    : name_(name), part_(std::move(part)) {}

RealtimeStatusOr<RealtimeOperationalStatus> RealtimePart::GetOperationalStatus()
    const {
  return part_->GetOperationalStatus();
}

absl::string_view RealtimePart::GetName() const { return name_; }

RealtimeStatus RealtimePart::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  INTRINSIC_TRACE_SCOPED(FixedStrCat<tracing::kMaxNameLength>(
      "RealtimePart::ReadStatus for part ", GetName()));
  RealtimeStatus status = part_->ReadStatus(params);
  if (!status.ok()) {
    return {status.code(), FixedStrCat<RealtimeStatus::kMaxMessageLength>(
                               status.message(), " part=", GetName())};
  }

  return OkStatus();
}

RealtimeStatus RealtimePart::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters params) {
  INTRINSIC_TRACE_SCOPED(FixedStrCat<tracing::kMaxNameLength>(
      "RealtimePart::ApplyCommand for part ", GetName()));
  RealtimeStatus status = part_->ApplyCommand(params);
  if (!status.ok()) {
    return {status.code(), FixedStrCat<RealtimeStatus::kMaxMessageLength>(
                               status.message(), " part=", GetName())};
  }
  return OkStatus();
}

FeatureInterfaceRegistry& RealtimePart::GetFeatureInterfaces() {
  return part_->GetFeatureInterfaces();
}

const FeatureInterfaceRegistry& RealtimePart::GetFeatureInterfaces() const {
  return part_->GetFeatureInterfaces();
}

HardwareGroupSet RealtimePart::GetHardwareDependencies() const {
  return part_->GetHardwareDependencies();
}

}  // namespace intrinsic::icon
